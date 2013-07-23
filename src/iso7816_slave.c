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
#include <errno.h>
#include <utility/trace.h>

/* LOW-LEVEL non-blocking I/O */

struct iso7816_port; // Abstract. implementation = platform-specific
struct iso7816_port *iso7816_port_init(int port_nb); // port_nb is board-specific
int iso7816_port_tx(struct iso7816_port *p, uint8_t c);
int iso7816_port_rx(struct iso7816_port *p, uint8_t *c);

void iso7816_port_get_rst_vcc(struct iso7816_port *p, int *rst, int *vcc);

// FIXME: scaffolding: replace with non-blocking state machines later.
int iso7816_port_txn(struct iso7816_port *p, uint8_t *c, int n) {
    uint32_t rv = 0;
    while(!rv && n-- ) {
        do { rv = iso7816_port_tx(p, *c); } while (-EAGAIN == rv);
        c++;
    }
    return rv;
}
int iso7816_port_rxn(struct iso7816_port *p, uint8_t *c, int n) {
    uint32_t rv = 0;
    while(!rv && n--) {
        do { rv = iso7816_port_rx(p, c); } while (-EAGAIN == rv);
        c++;
    }
    return rv;
}




/* BYTE-LEVEL PROTOCOL */

struct iso7816_slave {
    struct iso7816_port *port;
};


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
    TRACE_DEBUG("ATR\n\r");
    uint8_t atr[] = {0x3B, 0x98, 0x96, 0x00,
                     0x93, 0x94, 0x03, 0x08,
                     0x05, 0x03, 0x03, 0x03};
    iso7816_port_txn(s->port, atr, sizeof(atr));
}

void hexdump(uint8_t *c, int n) {
    while(n--) TRACE_DEBUG(" %02x", *c++);
}


static void iso7816_slave_receive_pts(struct iso7816_slave *s) {
    uint8_t c[3]; // hardcoded for PTS from BLU phone
    iso7816_port_rxn(s->port, c, sizeof(c));
    TRACE_DEBUG("PTS:"); hexdump(c, sizeof(c)); TRACE_DEBUG("\n\r");
}
static void iso7816_slave_send_pts_ack(struct iso7816_slave *s) {
    TRACE_DEBUG("PTS ACK\n\r");
    uint8_t c[] = {0xFF, 0x00, 0xFF};
    iso7816_port_txn(s->port, c, sizeof(c));
}

int iso7816_slave_transact_apdu(struct iso7816_slave *s) {
    // HEADER
    {
        uint8_t c[5];
        iso7816_port_rxn(s->port, c, sizeof(c));
        TRACE_DEBUG("TPDU_header:"); hexdump(c, sizeof(c)); TRACE_DEBUG("\n\r");
    }

    // protocol byte (ack)
    iso7816_port_tx(s->port, 0xA4);

    // payload
    {
        uint8_t c[2];
        iso7816_port_rxn(s->port, c, sizeof(c));
        TRACE_DEBUG("TPDU_payload:"); hexdump(c, sizeof(c)); TRACE_DEBUG("\n\r");
    }
    return -1;
}

void iso7816_slave_mainloop(struct iso7816_slave *p) {

    while(1) {
        iso7816_slave_wait_off(p);
        iso7816_slave_wait_reset(p);

        iso7816_slave_send_atr(p);

        iso7816_slave_receive_pts(p);
        iso7816_slave_send_pts_ack(p);

        while (0 == iso7816_slave_transact_apdu(p));
    }
}





/* BOARD-SPECIFIC: SIMTRACE */

static struct iso7816_slave phone;

struct iso7816_slave *iso7816_slave_init(void) {
    struct iso7816_slave *p = &phone;
    p->port = iso7816_port_init(1);
    return p;
}

