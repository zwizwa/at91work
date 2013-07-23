/* Adapted from iso7816_4.h */

/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#include <stdint.h>
#include <string.h>
#include "errno.h"
#include <utility/trace.h>

/* LOW-LEVEL non-blocking I/O */

struct iso7816_port; // Abstract. implementation = platform-specific
struct iso7816_port *iso7816_port_init(int port_nb); // port_nb is board-specific
int iso7816_port_tx(struct iso7816_port *p, uint8_t c);
int iso7816_port_rx(struct iso7816_port *p, uint8_t *c);

void iso7816_port_get_rst_vcc(struct iso7816_port *p, int *rst, int *vcc);





/* BYTE-LEVEL PROTOCOL */

enum iso7816_state {
    S_INIT = 0, // wait for RST low
    S_RESET,    // wait for RST high and start sending ATR
    S_ATR,      // ATR is out, start waiting for PTS (FIXME: hardcoded to 3 bytes)

    S_PTS,      // PTS is in, start sending PTS ack
    S_PTS_ACK,  // PTS ack is out, start listening for TPDU header.

    S_TPDU,
    S_TPDU_PROT,
    S_TPDU_PAYLOAD,

    S_HALT,     // scaffolding: break loop

    /* RX/TX: io_next contains next state after I/O is done */
    S_RX,
    S_TX,

};

struct iso7816_slave {
    struct iso7816_port *port;
    enum iso7816_state state;
    enum iso7816_state io_next;  // next state after I/O is done
    uint8_t msg[5 + 256]; // FIXME: check max message size
    int msg_index;  // current read(RX) or write(TX) index
    int msg_size;   // current(TX) or expected(RX) size of message
};


/* Start S_RX/S_TX transfer and continue at io_next */
static void next_io_start(struct iso7816_slave *s,
                          enum iso7816_state transfer_state,
                          enum iso7816_state io_next,
                          const uint8_t *buf, int size) {
    if (!size) {
        s->state = io_next;
    }
    else {
        s->state = transfer_state;
        s->io_next = io_next;
        s->msg_index = 0;
        s->msg_size = size;
        if (buf) {
            memcpy(s->msg, buf, s->msg_size);
        }
    }
}
static void next_receive(struct iso7816_slave *s,
                         enum iso7816_state io_next,
                         int size) {
    next_io_start(s, S_RX, io_next, NULL, size);
}
static void next_send(struct iso7816_slave *s,
                      enum iso7816_state io_next,
                      const uint8_t *buf,
                      int size) {
    next_io_start(s, S_TX, io_next, buf, size);
}

/* Continue or transfer and jump to io_next when done. */
static void next_io(struct iso7816_slave *s, int rv) {
    switch (rv) {
    case -EAGAIN:
        break;
    case ENOERR:
        s->msg_index++;
        if (s->msg_index >= s->msg_size) {
            s->state = s->io_next;
        }
        break;
    default:
        TRACE_DEBUG("i/o error %d\n\r", rv);
        s->state = S_RESET;
        break;
    }
}

/* Non-blocking state machine for ISO7816, slave side. */


int iso7816_slave_tick(struct iso7816_slave *s) {
    switch(s->state) {
        /* PROTOCOL:
           These states are "high level" states executed in sequence. */

        /**** STARTUP ****/

    case S_INIT: {
        /* Boot sync: wait for RST */
        int rst;
        iso7816_port_get_rst_vcc(s->port, &rst, NULL);
        if (!rst) s->state = S_RESET;
        break;
    }
    case S_RESET: {
        /* Boot sync: wait for RST release. */
        int rst, vcc;
        iso7816_port_get_rst_vcc(s->port, &rst, &vcc);
        if (rst && vcc) {
            static const uint8_t atr[] =
                {0x3B, 0x98, 0x96, 0x00, 0x93, 0x94,
                 0x03, 0x08, 0x05, 0x03, 0x03, 0x03};
            next_send(s, S_ATR, atr, sizeof(atr));
        }
        break;
    }
    case S_ATR:
        /* ATR is out, wait for Protocol Type Selection (PTS) */
        next_receive(s, S_PTS, 3);  // FIXME: hardcoded to 3 bytes from BTU phone
        break;
    case S_PTS:
        /* PTS is in, send PTS ack */
        TRACE_DEBUG("rx PTS %02x %02x %02x\n\r",
                    s->msg[0], s->msg[1], s->msg[2]);
        next_send(s, S_PTS_ACK, NULL, s->msg_size);
        break;
    case S_PTS_ACK:
        /* PTS ack is out, Start waiting for TPDU header */
        next_receive(s, S_TPDU, 5);
        break;


        /**** MAIN LOOP ****/

    case S_TPDU:
        /* TPDU header is in, send protocol byte. */
        TRACE_DEBUG("rx TPDU_header: %02x %02x %02x %02x %02x\n\r",
                    s->msg[0],s->msg[1],s->msg[2],s->msg[3],s->msg[4]);
        next_send(s, S_TPDU_PROT, &s->msg[1], 1);
        break;
    case S_TPDU_PROT:
        /* Protocol byte is in.  Receive data payload. */
        next_receive(s, S_TPDU_PAYLOAD, s->msg[4]);
        break;
    case S_TPDU_PAYLOAD:
        /* Payload is in. */
        TRACE_DEBUG("TPDU_payload: %02x %02x\n\r", s->msg[0], s->msg[1]);
        s->state = S_INIT; // FIXME
        break;



        /**** LOW LEVEL ****/

        /* Message transfer */
    case S_RX: next_io(s, iso7816_port_rx(s->port, &s->msg[s->msg_index])); break;
    case S_TX: next_io(s, iso7816_port_tx(s->port,  s->msg[s->msg_index])); break;

    default:
        TRACE_DEBUG("unknown state %d\n\r", s->state);
        s->state = S_INIT;
        break;
        /* Scaffolding: end loop */
    case S_HALT:
        return ENOERR;
    }
    return -EAGAIN; // state != S_HALT
}

// FIXME: scaffolding: run state machine
int iso7816_slave_run(struct iso7816_slave *s) {
    int rv;
    while (-EAGAIN == (rv = iso7816_slave_tick(s)));
    return rv;
}

#if 1

void iso7816_slave_mainloop(struct iso7816_slave *s) {
    s->state = S_INIT;
    iso7816_slave_run(s);
}


#else


// FIXME: scaffolding: replace with non-blocking state machines later.
static int iso7816_slave_send(struct iso7816_slave *s) {
    s->msg_index = 0;
    s->state     = S_TX;
    s->io_next   = S_HALT;
    return iso7816_run(s);
}
static int iso7816_slave_receive(struct iso7816_slave *s) {
    s->msg_index = 0;
    s->state     = S_RX;
    s->io_next   = S_HALT;
    return iso7816_run(s);
}
static void iso7816_slave_wait_off(struct iso7816_slave *s) {
    int vcc, rst;
    do { iso7816_port_get_rst_vcc(s->port, &vcc, &rst); }
    while (rst);
}
static void iso7816_slave_wait_reset(struct iso7816_slave *s) {
    int vcc, rst;
    do { iso7816_port_get_rst_vcc(s->port, &vcc, &rst); }
    while (!(vcc && rst));
}
static void iso7816_slave_send_atr(struct iso7816_slave *s) {
    TRACE_DEBUG("tx ATR\n\r");
    const uint8_t atr[] = {0x3B, 0x98, 0x96, 0x00,
                           0x93, 0x94, 0x03, 0x08,
                           0x05, 0x03, 0x03, 0x03};
    s->msg_size = sizeof(atr);
    memcpy(s->msg, atr, s->msg_size);
    iso7816_slave_send(s);
}
static void iso7816_slave_receive_pts(struct iso7816_slave *s) {
    s->msg_size = 3; // hardcoded for PTS from BLU phone
    iso7816_slave_receive(s);
    TRACE_DEBUG("rx PTS %02x %02x %02x\n\r", s->msg[0], s->msg[1], s->msg[2]);
}
static void iso7816_slave_send_pts_ack(struct iso7816_slave *s) {
    TRACE_DEBUG("tx PTS ACK\n\r");
    s->msg_size = 3; // just send it back
    iso7816_slave_send(s);
}

int iso7816_slave_transact_apdu(struct iso7816_slave *s) {
    // HEADER
    s->msg_size = 5;
    iso7816_slave_receive(s);
    TRACE_DEBUG("TPDU_header: %02x %02x %02x %02x %02x\n\r",
                s->msg[0],s->msg[1],s->msg[2],s->msg[3],s->msg[4]);

    // protocol byte (ack)
    s->msg[0] = s->msg[1];
    s->msg_size = 1;
    iso7816_slave_send(s);

    // payload
    s->msg_size = 2;
    iso7816_slave_receive(s);
    TRACE_DEBUG("TPDU_payload: %02x %02x\n\r", s->msg[0], s->msg[1]);

    return -1;
}

void iso7816_slave_mainloop(struct iso7816_slave *s) {

    while(1) {
        iso7816_slave_wait_off(s);
        iso7816_slave_wait_reset(s);

        iso7816_slave_send_atr(s);

        iso7816_slave_receive_pts(s);
        iso7816_slave_send_pts_ack(s);

        while (0 == iso7816_slave_transact_apdu(s));
    }
}

#endif





/* BOARD-SPECIFIC: SIMTRACE */

static struct iso7816_slave phone;

struct iso7816_slave *iso7816_slave_init(void) {
    struct iso7816_slave *p = &phone;
    p->port = iso7816_port_init(1);
    return p;
}

