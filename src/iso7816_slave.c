/* License: GPL
   (c) 2010 by Harald Welte <hwelte@hmw-consulting.de>
   (c) 2013 by Tom Schouten <tom@zwizwa.be>
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

#ifndef TRACE_WARN
#define TRACE_WARN printf
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
    INS_EXTERNAL_AUTHOENTICATE = 0x82,
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


/* BYTE-LEVEL SLAVE-SIDE PROTOCOL */

enum iso7816_state {
    S_INIT = 0, // wait for RST low
    S_RESET,    // wait for RST high and start sending ATR
    S_ATR,      // ATR is out, start waiting for PTS (FIXME: hardcoded to 3 bytes)

    S_PTS,      // PTS is in, start sending PTS ack
    S_PTS_ACK,  // PTS ack is out, start listening for TPDU header.

    S_TPDU,
    S_TPDU_PROT,
    S_TPDU_DATA,
    S_TPDU_WAIT_REPLY,
    S_TPDU_REPLY,
    S_TPDU_RESP,

    /* RX/TX: io_next contains next state after I/O is done */
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
        struct tpdu tpdu;
        uint8_t buf[8 + 256];   // receive/send buffer
    } msg;
    uint8_t c_apdu_size;  // c_apdu = tpdu header + c_apdu_size bytes
    uint8_t r_apdu_size;  // r_apdu = r_apdu_size bytes after c_apdu
    int skip_reset;
};


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
        s->io_ptr = buf ? buf : s->msg.buf;
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
        TRACE_WARN("i/o error %d\n\r", rv);
        s->state = S_RESET;
        break;
    }
}

static void next_receive_tpdu_header(struct iso7816_slave *s) {
    memset(s->msg.buf, 0x55, sizeof(s->msg.buf));  // init simplifies debugging
    next_receive(s, S_TPDU, s->msg.buf, sizeof(s->msg.tpdu));
}



int iso7816_slave_c_apdu_read(struct iso7816_slave *s) {
    /* During the request delegation phase, we only provide C-APDU
     * bytes.  Delegate will call iso7816_slave_r_apdu() to continue
     * processing. */
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




/* Connect to C/R APDU */
int iso7816_slave_r_apdu(struct iso7816_slave *s, const uint8_t *buf, int size) {
    TRACE_WARN("r_apdu %d\n\r", size);
    if (s->state != S_TPDU_WAIT_REPLY) return -EAGAIN; // not waiting for data
    if (size != s->r_apdu_size) {
        TRACE_WARN("R-APDU data size incorrect size:%d != r_apdu_size:%d\n",
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

    // vcc = 1; // HACK: effectively ignore VCC, just look at RST

    /* Monitor RST and VCC */
    if (!(rst && vcc)) {
        if (s->state != S_RESET) {
            TRACE_WARN("%d -> S_RESET VCC:%d RST:%d\n\r", s->state, vcc, rst);
            s->state = S_RESET;
        }
    }


    switch(s->state) {
        /* PROTOCOL:
           These states are "high level" states executed in sequence. */

        /**** STARTUP ****/
    case S_INIT:
        /* Don't talk to iso7816 master in unknown state. */
        break;
    case S_RESET: {
        /* Boot sync: wait for RST release. */
        if (rst && vcc) {
            if (s->skip_reset) {
                s->skip_reset--;
                TRACE_WARN("Skipping reset (%d more)\n\r", s->skip_reset);
                s->state = S_INIT;
            }
            else {
                TRACE_WARN("S_RESET -> S_ATR\n\r");
                s->port = iso7816_port_init(1);
                static const uint8_t atr[] =
                    {0x3B, 0x98, 0x96, 0x00, 0x93, 0x94,
                     0x03, 0x08, 0x05, 0x03, 0x03, 0x03};
                next_send(s, S_ATR, atr, sizeof(atr));
            }
        }
        break;
    }
    case S_ATR:
        /* ATR is out, wait for Protocol Type Selection (PTS) */
        TRACE_WARN("S_ATR\n\r");
        next_receive(s, S_PTS, NULL, 3);  // FIXME: hardcoded to 3 bytes from BTU phone
        break;
    case S_PTS:
        /* PTS is in, send PTS ack */
        TRACE_WARN("S_PTS %02x %02x %02x\n\r",
                    s->msg.buf[0], s->msg.buf[1], s->msg.buf[2]);
        next_send(s, S_PTS_ACK, NULL, 3);
        break;
    case S_PTS_ACK:
        /* PTS ack is out, Start waiting for TPDU header */
        TRACE_WARN("S_PTS_ACK\n\r");
        next_receive_tpdu_header(s);
        break;


        /**** MAIN LOOP ****/

    case S_TPDU:
        /* TPDU header is in, send protocol byte. */
        TRACE_WARN("S_TPDU\n\r");
        next_send(s, S_TPDU_PROT, &s->msg.tpdu.ins, 1);
        break;
    case S_TPDU_PROT:
        /* Protocol byte is out.
           P3 contains data size, INS determines direction. */
        TRACE_WARN("S_TPDU_PROT\n\r");
        switch(s->msg.tpdu.ins) {
        case INS_GET_RESPONSE:
        case INS_READ_BINARY:
        case INS_STATUS:
            /* data in R-APDU */
            s->c_apdu_size = sizeof(struct tpdu);
            s->r_apdu_size = s->msg.tpdu.p3 + 2;
            break;
        default:
            /* data in C-APDU */
            s->c_apdu_size = sizeof(struct tpdu) + s->msg.tpdu.p3;
            s->r_apdu_size = 2;
        }
        next_receive(s, S_TPDU_DATA, &s->msg.tpdu.data[0],
                     s->c_apdu_size - sizeof(struct tpdu));
        break;

    case S_TPDU_DATA:
        /* All C-APDU data is in.  Pass it to handler. */
        TRACE_WARN("S_TPDU_DATA\n\r");
        s->state = S_TPDU_WAIT_REPLY;
        s->io_ptr = s->msg.buf;
        s->c_apdu_cb(s->c_apdu_ctx, s->c_apdu_size);
        break;
    case S_TPDU_WAIT_REPLY:
        /* Wait for data provided by is7816_slave_r_apdu() */
        break;
    case S_TPDU_REPLY:
        /* Send response data + SW */
        TRACE_WARN("S_TPDU_REPLY\n\r");
        next_send(s, S_TPDU_RESP,
                  s->msg.buf + s->c_apdu_size,
                  s->r_apdu_size);
        break;
    case S_TPDU_RESP:
        /* Response sent, wait for next TPDU */
        TRACE_WARN("S_TPDU_RESP\n\r");
        next_receive_tpdu_header(s);
        break;


        /**** LOW LEVEL ****/

        /* Message transfer */
    case S_RX: next_io(s, iso7816_port_rx(s->port, s->io_ptr));    break;
    case S_TX: next_io(s, iso7816_port_tx(s->port, s->io_ptr[0])); break;

    default:
        TRACE_WARN("unknown state %d\n\r", s->state);
        s->state = S_INIT;
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
    s->state = S_INIT;
    s->skip_reset = 1;
    return s;
}

