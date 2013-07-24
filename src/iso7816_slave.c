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
    S_TPDU_RESP,

    S_HALT,     // scaffolding: break loop

    /* RX/TX: io_next contains next state after I/O is done */
    S_RX,
    S_TX,

};

struct tpdu {
    uint8_t cla;
    uint8_t ins;
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
    uint8_t data[];
} __attribute__((__packed__));

struct iso7816_slave {
    struct iso7816_port *port;
    enum iso7816_state state;
    enum iso7816_state io_next; // next state after I/O is done
    uint8_t *io_ptr;        // current read(RX) or write(TX) index
    uint8_t *io_endx;       // current(TX) or expected(RX) size of message
    union {
        struct tpdu tpdu;
        uint8_t buf[8 + 256];   // receive/send buffer
    } msg;
};


/* Start S_RX/S_TX transfer and continue at io_next */
static void next_io_start(struct iso7816_slave *s,
                          enum iso7816_state transfer_state,
                          enum iso7816_state io_next,
                          uint8_t *buf, int size) {
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
    next_io_start(s, S_RX, io_next, buf, size);
}
static void next_send(struct iso7816_slave *s,
                      enum iso7816_state io_next,
                      const uint8_t *buf,
                      int size) {
    next_io_start(s, S_TX, io_next, (uint8_t*)buf, size);
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
        TRACE_DEBUG("i/o error %d\n\r", rv);
        s->state = S_RESET;
        break;
    }
}

static void next_receive_tpdu_header(struct iso7816_slave *s) {
    next_receive(s, S_TPDU, s->msg.buf, sizeof(s->msg.tpdu));
}

static void print_c_apdu(struct iso7816_slave *s) {
    printf("C-APDU: ");
    int i, len = sizeof(s->msg.tpdu) + s->msg.tpdu.p3;
    for (i = 0; i < len; i++) printf("%02X", s->msg.buf[i]);
    printf("\n\r");
}


/* Non-blocking state machine for ISO7816, slave side. */


int iso7816_slave_tick(struct iso7816_slave *s) {

    int rst, vcc;
    iso7816_port_get_rst_vcc(s->port, &rst, &vcc);

    /* Monitor RST and VCC */
    if (!(rst && vcc)) {
        if (s->state != S_RESET) {
            TRACE_DEBUG("RST=%d VCC=%d\n\r", rst, vcc)
            s->state = S_RESET;
        }
    }


    switch(s->state) {
        /* PROTOCOL:
           These states are "high level" states executed in sequence. */

        /**** STARTUP ****/
    case S_INIT:
        /* Don't talk to iso7816 in an unknown state. */
        break;
    case S_RESET: {
        /* Boot sync: wait for RST release. */
        if (rst && vcc) {
            TRACE_DEBUG("RST=VCC=1\n\r");
            static const uint8_t atr[] =
                {0x3B, 0x98, 0x96, 0x00, 0x93, 0x94,
                 0x03, 0x08, 0x05, 0x03, 0x03, 0x03};
            next_send(s, S_ATR, atr, sizeof(atr));
        }
        break;
    }
    case S_ATR:
        /* ATR is out, wait for Protocol Type Selection (PTS) */
        next_receive(s, S_PTS, NULL, 3);  // FIXME: hardcoded to 3 bytes from BTU phone
        break;
    case S_PTS:
        /* PTS is in, send PTS ack */
        TRACE_DEBUG("rx PTS %02x %02x %02x\n\r",
                    s->msg.buf[0], s->msg.buf[1], s->msg.buf[2]);
        next_send(s, S_PTS_ACK, NULL, 3);
        break;
    case S_PTS_ACK:
        /* PTS ack is out, Start waiting for TPDU header */
        next_receive_tpdu_header(s);
        break;


        /**** MAIN LOOP ****/

    case S_TPDU:
        /* TPDU header is in, send protocol byte. */
        next_send(s, S_TPDU_PROT, &s->msg.tpdu.ins, 1);
        break;
    case S_TPDU_PROT:
        /* Protocol byte is in.  Receive data payload. */
        next_receive(s, S_TPDU_PAYLOAD, &s->msg.tpdu.data[0], s->msg.tpdu.p3);
        break;
    case S_TPDU_PAYLOAD: {
        /* TPDU Payload is in, completing C-APDU. */
        print_c_apdu(s);
        TRACE_DEBUG("tx TPDU_response\n\r");
        static const uint8_t resp[] = {0x9F, 0x1A};
        next_send(s, S_TPDU_RESP, resp, sizeof(resp));
        break;
    }
    case S_TPDU_RESP:
        /* Response sent, wait for next TPDU */
        next_receive_tpdu_header(s);
        break;


        /**** LOW LEVEL ****/

        /* Message transfer */
    case S_RX: next_io(s, iso7816_port_rx(s->port,  s->io_ptr));    break;
    case S_TX: next_io(s, iso7816_port_tx(s->port,  s->io_ptr[0])); break;

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


void iso7816_slave_mainloop(struct iso7816_slave *s) {
    s->state = S_INIT;
    iso7816_slave_run(s);
}



/* BOARD-SPECIFIC: SIMTRACE */

static struct iso7816_slave phone;

struct iso7816_slave *iso7816_slave_init(void) {
    struct iso7816_slave *p = &phone;
    p->port = iso7816_port_init(1);
    return p;
}

