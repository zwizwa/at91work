/* ISO7816-3 TPDU T=0 slave side protocol.
 * (C) 2010 by Harald Welte <hwelte@hmw-consulting.de>
 * (C) 2013 by Tom Schouten <tom@getbeep.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Basic design notes.

   - Separate platform/board parts from protocol part for UART and
     control lines.  Allow reuse as MITM or full SIM-side
     implementation (e.g. COS).

   - Don't write blocking code; Structure as a non-blocking state
     machine to avoid dependency on platform thread support, or
     busy-waiting single-threaded implementation.

 */

#include "iso7816_slave.h"
#include <stdint.h>
#include <string.h>
#include <utility/trace.h>
#include "errno.h"

#ifndef TRACE_WARNING
#define TRACE_WARNING printf
#endif

#include "iso7816_port.h"


// FIXME: share with master
enum iso7816_ins {
    INS_DEACTIVATE_FILE        = 0x04,
    INS_ERASE_BINARY           = 0x0E,
    INS_TERMINAL_PROFILE       = 0x10,
    INS_FETCH                  = 0x12,
    INS_TERMINAL_RESPONSE      = 0x14,
    INS_VERIFY                 = 0x20,
    INS_CHANGE_PIN             = 0x24,
    INS_DISABLE_PIN            = 0x26,
    INS_ENABLE_PIN             = 0x28,
    INS_UNBLOCK_PIN            = 0x2C,
    INS_INCREASE               = 0x32,
    INS_ACTIVATE_FILE          = 0x44,
    INS_MANAGE_CHANNEL         = 0x70,
    INS_MANAGE_SECURE_CHANNEL  = 0x73,
    INS_TRANSACT_DATA          = 0x75,
    INS_EXTERNAL_AUTHENTICATE  = 0x82,
    INS_GET_CHALLENGE          = 0x84,
    INS_INTERNAL_AUTHENTICATE  = 0x88, // also 0x89 ??
    INS_SEARCH_RECORD          = 0xA2,
    INS_SELECT_FILE            = 0xA4,
    INS_TERMINAL_CAPABILITY    = 0xAA,
    INS_READ_BINARY            = 0xB0,
    INS_READ_RECORD            = 0xB2,
    INS_GET_RESPONSE           = 0xC0,   /* Transmission-oriented APDU */
    INS_ENVELOPE               = 0xC2,
    INS_RETRIEVE_DATA          = 0xCB,
    INS_GET_DATA               = 0xCA,
    INS_WRITE_BINARY           = 0xD0,
    INS_WRITE_RECORD           = 0xD2,
    INS_UPDATE_BINARY          = 0xD6,
    INS_PUT_DATA               = 0xDA,
    INS_SET_DATA               = 0xDB,
    INS_UPDATE_DATA            = 0xDC,
    INS_APPEND_RECORD          = 0xE2,
    INS_STATUS                 = 0xF2,
};
// FIXME: share with master
struct tpdu {
    uint8_t cla;
    uint8_t ins;
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
    uint8_t data[];
} __attribute__((__packed__));

// FIXME: share with master
struct pts {
    uint8_t ptss;
    uint8_t pts[5]; // PTS0, optional PTS1-PTS3, PCK
} __attribute__((__packed__));



/* BYTE-LEVEL SLAVE-SIDE PROTOCOL */

enum iso7816_state {
    /* STARTUP */
    S_HALT = 0, // wait for VCC low
    S_OFF,      // wait for VCC high
    S_ON,       // wait for RST low
    S_RESET,    // wait for RST high, then sending ATR
    S_ATR,      // ATR is out, start waiting for PTS
    S_PTS_HEAD, // first 2 PTS bytes are in, determine PTS length and receive.
    S_PTS_TAIL, // full PTS is in, send PTS ack
    S_PTS_ACK,  // PTS ack is out, start listening for TPDU header.

    /* MAIN LOOP */
    S_TPDU,
    S_TPDU_PROT,
    S_TPDU_DATA,
    S_TPDU_WAIT_REPLY,
    S_TPDU_REPLY,
    S_TPDU_RESP,

    /* LOW-LEVEL RX/TX: io_next contains next state after I/O is done */
    S_RX,
    S_TX,

};


struct iso7816_slave {
    struct iso7816_port *port;
    iso7816_slave_c_apdu_t c_apdu_cb;
    void *c_apdu_ctx;
    enum iso7816_state state;
    enum iso7816_state io_next; // next state after I/O is done
    uint8_t *io_ptr;            // current read(RX) or write(TX) index
    uint8_t *io_endx;           // current(TX) or expected(RX) size of message
    union {
        struct pts pts;
        struct tpdu tpdu;
        uint8_t buf[256 + 8]; // receive/send buffer (round up from 5 + 256 + 2)
    } msg;
    int c_apdu_size;  // c_apdu = tpdu header + c_apdu_size bytes
    int r_apdu_size;  // r_apdu = r_apdu_size bytes after c_apdu
    int skip_power;
    uint8_t *pts1, *pck;
};




/* Table 6 from ISO 7816-3 */
static const uint16_t fi_table[] = {
	372, 372, 558, 744, 1116, 1488, 1860, 0,
	0, 512, 768, 1024, 1536, 2048, 0, 0
};

/* Table 7 from ISO 7816-3 */
static const uint8_t di_table[] = {
	0, 1, 2, 4, 8, 16, 32, 64,
	12, 20, 2, 4, 8, 16, 32, 64,
};

#define ARRAY_SIZE(a) (sizeof(a) / sizeof(a[0]))

/* compute the F/D ratio based on Fi and Di values */
static int compute_fidi_ratio(uint8_t fi, uint8_t di)
{
	uint16_t f, d;
	int ret;

	if (fi >= ARRAY_SIZE(fi_table) ||
	    di >= ARRAY_SIZE(di_table))
		return -EINVAL;

	f = fi_table[fi];
	if (f == 0)
		return -EINVAL;

	d = di_table[di];
	if (d == 0)
		return -EINVAL;

	/* See table 7 of ISO 7816-3: From 1000 on we divide by 1/d,
	 * which equals a multiplication by d */
	if (di < 8) 
		ret = f / d;
	else
		ret = f * d;

	return ret;
}



/* Start S_RX/S_TX transfer and continue at io_next */
static void next_io_start(struct iso7816_slave *s,
                          enum iso7816_state io_next,
                          uint8_t *buf, int size,
                          enum iso7816_state transfer_state) {
    if (!size) {
        s->state = io_next;
    }
    else {
        s->state = transfer_state;
        s->io_next = io_next;
        s->io_ptr = buf;
        s->io_endx = s->io_ptr + size;
    }
}
static void next_receive(struct iso7816_slave *s,
                         enum iso7816_state io_next,
                         uint8_t *buf,
                         int size) {
    next_io_start(s, io_next, buf, size, S_RX);
}
static void next_send(struct iso7816_slave *s,
                      enum iso7816_state io_next,
                      const uint8_t *buf,
                      int size) {
    next_io_start(s, io_next, (uint8_t*)buf, size, S_TX);
}

/* Continue or transfer and jump to io_next when done. */
static void next_io(struct iso7816_slave *s, int rv) {
    switch (rv) {
    case -EAGAIN:
        break;
    case ENOERR:
        s->io_ptr++;
        if (s->io_ptr >= s->io_endx) {
            s->state = s->io_next;
        }
        break;
    default:
        TRACE_WARNING("i/o error %d\n\r", rv);
        s->state = S_RESET;
        break;
    }
}

static void next_receive_S_TPDU(struct iso7816_slave *s) {
    memset(s->msg.buf, 0x55, sizeof(s->msg.buf));  // init simplifies debugging
    next_receive(s, S_TPDU, s->msg.buf, sizeof(s->msg.tpdu));
}



int iso7816_slave_c_apdu_read(struct iso7816_slave *s) {
    /* During the request delegation phase, we provide C-APDU, one
       byte at a time.  Delegate will call iso7816_slave_r_apdu() to
       continue processing. */
    if (s->state != S_TPDU_WAIT_REPLY) {
        TRACE_ERROR("iso7816_slave_c_apdu_read state %d\n", s->state);
        return -EIO;
    }
    if (s->io_ptr >= (s->msg.buf + s->c_apdu_size)) {
        return -EIO; // FIXME: what error to use?
    }
    else {
        return *(s->io_ptr)++;
    }
}


int iso7816_slave_r_apdu_write(struct iso7816_slave *s, const uint8_t *buf, int size) {
    TRACE_WARNING("r_apdu %d\n\r", size);
    if (s->state != S_TPDU_WAIT_REPLY) return -EAGAIN; // not waiting for data
    if (size != s->r_apdu_size) {
        TRACE_WARNING("R-APDU data size incorrect size:%d != r_apdu_size:%d\n",
                    size, s->r_apdu_size);
        return -EIO; // FIXME: what should this be?
    }
    memcpy(s->msg.buf + s->c_apdu_size, buf, size);
    s->state = S_TPDU_REPLY;
    return ENOERR;
}



/* Non-blocking state machine for ISO7816, slave side. */


void iso7816_slave_tick(struct iso7816_slave *s) {

    int rst, vcc;
    iso7816_port_get_rst_vcc(s->port, &rst, &vcc);


    if (!vcc) s->state = S_OFF;


    switch(s->state) {
        /* These states are "high level" states executed mostly in
           sequence, interleaved with message RX/TX states. */

        /**** STARTUP ****/

    case S_HALT:
        /* Wait until next power cycle. */
        break;
    case S_OFF:
        if (vcc) {
            TRACE_WARNING("->S_ON\n\r");
            s->state = S_ON;
        }
        break;
    case S_ON:
        if (s->skip_power) {
            TRACE_WARNING("->S_ON_SKIP %d\n\r", s->skip_power);
            s->state = S_HALT;
            s->skip_power--;
        }
        else if (!rst) {
            TRACE_WARNING("->S_RESET\n\r");
            s->state = S_RESET;
        }
        break;
    case S_RESET: {
        /* Boot sync: wait for RST release. */
        if (rst) {
            TRACE_WARNING("->S_ATR\n\r");
            s->port = iso7816_port_init(1);
            static const uint8_t atr[] =
                {0x3B, 0x98, 0x96, 0x00, 0x93, 0x94,
                 0x03, 0x08, 0x05, 0x03, 0x03, 0x03};
            next_send(s, S_ATR, atr, sizeof(atr));
        }
        break;
    }
    case S_ATR:
        /* ATR is out, wait for Protocol Type Selection (PTS)
           http://www.cardwerk.com/smartcards/smartcard_standard_ISO7816-3.aspx
           PTSS FF
           PTS0 b5=PTS1, b6=PTS2, b7=PTS3 preset
           PTS1-3 : optional
           PCK  checksum
        */
        TRACE_DEBUG("S_ATR\n\r");
        next_receive(s, S_PTS_HEAD, s->msg.buf, 2);  // receive PTSS, PTS0
        break;
    case S_PTS_HEAD: {
        /* Determine length of remainder of PTS and receive. */
        // TRACE_DEBUG("S_PTS_HEAD"); // debug print interferes with reception
        uint8_t ptss = s->msg.pts.ptss;
        uint8_t pts0 = s->msg.pts.pts[0];
        TRACE_DEBUG("S_PTS_HEAD %02x %02x\n\r", ptss, pts0);
        uint8_t *p = &s->msg.pts.pts[1];
        uint8_t *p_start = p;
        if (pts0 & (1<<4)) { s->pts1 =     p++; } else { s->pts1 = NULL; }
        if (pts0 & (1<<5)) { /*s->pts2 =*/ p++; }
        if (pts0 & (1<<6)) { /*s->pts3 =*/ p++; }
        s->pck = p; p++;
        // receive optionally PTS1-3 and PCK.
        next_receive(s, S_PTS_TAIL, p_start, p - p_start);
        break;
    }
    case S_PTS_TAIL: {
        /* PTS is in, send PTS ack */
        int i, n = 1 + s->pck - s->msg.buf;
        TRACE_DEBUG("S_PTS_TAIL\n\r");
        for (i=0; i<n; i++) TRACE_DEBUG("%02X\n\r", s->msg.buf[i]);
        next_send(s, S_PTS_ACK, s->msg.buf, n);
        break;
    }
    case S_PTS_ACK:
        /* PTS ack is out.  Switch rate and start waiting for TPDU header */
        TRACE_DEBUG("S_PTS_ACK\n\r");
        if (s->pts1) {
            uint8_t fi = (*s->pts1) >> 4;
            uint8_t di = (*s->pts1) & 0x0F;
            int fidi = compute_fidi_ratio(fi, di);
            iso7816_port_set_fidi(s->port, fidi);
        }
        else {
            TRACE_DEBUG("No FIDI set.\n\r");
        }
        next_receive_S_TPDU(s);
        break;


        /**** MAIN LOOP ****/

    case S_TPDU:
        /* TPDU header is in, send protocol byte. */
        TRACE_DEBUG("S_TPDU %02X %02X %02X %02X %02X\n\r",
                      s->msg.tpdu.cla,
                      s->msg.tpdu.ins,
                      s->msg.tpdu.p1,
                      s->msg.tpdu.p2,
                      s->msg.tpdu.p3);
        next_send(s, S_TPDU_PROT, &s->msg.tpdu.ins, 1);
        break;
    case S_TPDU_PROT: {
        /* Protocol byte is out.
           P3 contains data size, INS determines direction. */
        //TRACE_DEBUG("S_TPDU_PROT\n\r");
        int p3_len = (s->msg.tpdu.p3 != 0) ? s->msg.tpdu.p3 : 0x100;
        switch(s->msg.tpdu.ins) {
        case INS_GET_RESPONSE:
        case INS_READ_BINARY:
        case INS_READ_RECORD:
        case INS_STATUS:
            /* data in R-APDU */
            /* FIXME: there are probably morec cases.  Go over specs */
            s->c_apdu_size = sizeof(struct tpdu);
            s->r_apdu_size = p3_len + 2;
            break;
        default:
            /* data in C-APDU */
            s->c_apdu_size = sizeof(struct tpdu) + p3_len;
            s->r_apdu_size = 2;
        }
        next_receive(s, S_TPDU_DATA, &s->msg.tpdu.data[0],
                     s->c_apdu_size - sizeof(struct tpdu));
        break;
    }

    case S_TPDU_DATA:
        /* All C-APDU data is in.  Pass it to handler. */
        TRACE_DEBUG("S_TPDU_DATA\n\r");
        s->state = S_TPDU_WAIT_REPLY;
        s->io_ptr = s->msg.buf;
        s->c_apdu_cb(s->c_apdu_ctx, s->c_apdu_size);
        break;
    case S_TPDU_WAIT_REPLY:
        /* Wait for data provided by is7816_slave_r_apdu() */
        break;
    case S_TPDU_REPLY:
        /* Send response data + SW */
        TRACE_DEBUG("S_TPDU_REPLY\n\r");
        next_send(s, S_TPDU_RESP,
                  s->msg.buf + s->c_apdu_size,
                  s->r_apdu_size);
        break;
    case S_TPDU_RESP:
        /* Response sent, wait for next TPDU */
        TRACE_DEBUG("S_TPDU_RESP\n\r");
        next_receive_S_TPDU(s);
        break;


        /**** LOW LEVEL ****/

        /* Message transfer */
    case S_RX: next_io(s, iso7816_port_rx(s->port, s->io_ptr));    break;
    case S_TX: next_io(s, iso7816_port_tx(s->port, s->io_ptr[0])); break;

    default:
        TRACE_WARNING("unknown state %d\n\r", s->state);
        s->state = S_HALT;
        break;
    }
}



/* BOARD-SPECIFIC: SIMTRACE */

static struct iso7816_slave phone;

struct iso7816_slave *iso7816_slave_init(iso7816_slave_c_apdu_t c_apdu_cb, void *c_apdu_ctx) {
    struct iso7816_slave *s = &phone;
    s->c_apdu_cb  = c_apdu_cb;
    s->c_apdu_ctx = c_apdu_ctx;
    s->port = iso7816_port_init(1);
    s->state = S_HALT;


    /* Power cycle skipping: To make sure the phone selects the correct
       operating voltage, it might be necessary to skip a number of power
       cycles before sending out an ATR.

       EX: For Nexus One, skip_power=2 seems to work.

       Pulse 1 = SIMtrace VCC pullup (phone doesn't assert VCC line)
       Pulse 2 = Phone asserts 2 V
       Pulse 3 = Phone asserts 3 V
    */
    s->skip_power = 2;
    return s;
}

