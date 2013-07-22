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

#include <usart/usart.h>
#include <stdint.h>
#include <utility/trace.h>


// Pins: as annotated on schematics
#define  IO_PHONE  AT91C_PA22_TXD1
#define CLK_PHONE  AT91C_PA23_SCK1
#define RST_PHONE  AT91C_PIO_PA24
#define VCC_PHONE  AT91C_PIO_PA25

enum {
    PHONE_INVALID = 0,
    PHONE_RX,
    PHONE_TX,
} phone_state = PHONE_INVALID;

// phone side usart.
static AT91S_USART * const phone_usart = AT91C_BASE_US1;

uint32_t phone_status(void) {
    /* Check */
    uint32_t mask = AT91C_US_OVRE
        | AT91C_US_FRAME
        | AT91C_US_PARE
        | AT91C_US_TIMEOUT
        | AT91C_US_NACK
        | (1<<10);
    uint32_t csr = phone_usart->US_CSR;
    uint32_t status = mask & csr;
    if (status != 0) {
        TRACE_DEBUG("csr: %08x %08x\n\r", csr, status);
        phone_usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTNACK;
    }
    return status;
}

uint32_t phone_tx(uint8_t c) {

    /* Switch RX/TX state if necessary. */
    if (phone_state != PHONE_TX) {
        phone_usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        // phone_usart->US_CR = AT91C_US_TXEN;
        phone_state = PHONE_TX;
        TRACE_DEBUG("PHONE_TX\n\r");
    }
    /* Flush */
    int timeout = 5000; //FIXME
    while((phone_usart->US_CSR & AT91C_US_TXRDY) == 0) {
        if (!timeout--) {
            TRACE_DEBUG("phone_tx timeout\n\r");
            timeout = 5000;
        }
        // TRACE_DEBUG("!TXRDY\n\r");
    }
    // TRACE_DEBUG("TXRDY\n\r");
    /* Send */
    phone_usart->US_THR = c;
    return phone_status();
}

uint32_t phone_txn(uint8_t *c, int n) {
    uint32_t rv = 0;
    while(n--) rv = phone_tx(*c++);
    return rv;
}

uint32_t phone_rx(uint8_t *c) {
    /* Switch RX/TX state if necessary. */
    if (phone_state != PHONE_RX) {
        while((phone_usart->US_CSR & AT91C_US_TXEMPTY) == 0) {}
        phone_usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        phone_state = PHONE_RX;
    }

    /* Wait */
    int timeout = 5000; //FIXME
    while( ((phone_usart->US_CSR & AT91C_US_RXRDY) == 0) ) {
        if (!timeout--) {
            TRACE_DEBUG("phone_rx timeout\n\r");
            timeout = 5000;
        }
    }
    /* Read */
    *c = ((phone_usart->US_RHR) & 0xFF);
    return phone_status();
}

void phone_init(void) {

    /* See at91lib/components/iso7816/iso7816_4.c
       and 31.6.4 in AT91SAM7S128 data sheet.

       For phone socket, clock is always slave. AT91C_US_CKLO is off:
       we never drive the clock pin.  This was on in openpcd-based
       SIMtrace FW.
    */

    /* Enable usart peripheral clock */
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_US1);

    /* PHONE I/O and CLK controlled by peripheral A (Table 10-3) */
    AT91C_BASE_PIOA->PIO_ASR = IO_PHONE | CLK_PHONE;
    AT91C_BASE_PIOA->PIO_PDR = IO_PHONE | CLK_PHONE;

    /* PHONE RST and VCC are input.  This pin is tied to 3V3,100K.  */
    AT91C_BASE_PIOA->PIO_ODR = RST_PHONE | VCC_PHONE;
    AT91C_BASE_PIOA->PIO_PER = RST_PHONE | VCC_PHONE;

    /* Reset and disable receiver & transmitter */
    phone_usart->US_CR =
        AT91C_US_RSTRX
        | AT91C_US_RSTTX
        | AT91C_US_RXDIS
        | AT91C_US_TXDIS;

    phone_usart->US_CR =
        AT91C_US_RXDIS
        | AT91C_US_TXDIS;

    /* Set mode to ISO7816 T0, external clock source */
    phone_usart->US_MR =
        AT91C_US_USMODE_ISO7816_0  // T0
        | AT91C_US_CLKS_EXT        // external clock source
        | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_EVEN
        | AT91C_US_NBSTOP_1_BIT
        | AT91C_US_CKLO
        | AT91C_US_INACK
        | (3<<24);

    /* In 7816 mode, Baud Rate = Selected Clock / CD / FI_DI_RATIO
       Since we use external clock, CD=1 */
    phone_usart->US_BRGR = 1;

    /* Set ATR clock div to 372 */
    phone_usart->US_FIDI = 372 & 0x3FF;

    /* Disable Receiver Time-out */
    phone_usart->US_RTOR = 0;

    /* Disable Transmitter Timeguard */
    phone_usart->US_TTGR = 0;

    /* Enable TX */
    phone_usart->US_CR = AT91C_US_TXEN;
    phone_usart->US_CR = AT91C_US_RXEN;

    /* FIXME: read RHR once to flush? */
    phone_usart->US_RHR;


    uint32_t prev = 0;
    int cycles = 0;

    while(0) {
        // uint32_t mask = RST_PHONE | VCC_PHONE;

        uint32_t mask = CLK_PHONE;
        uint32_t cur = AT91C_BASE_PIOA->PIO_PDSR & mask;
        // if ((prev != cur) && (cur == 0)) {
            // TRACE_DEBUG("%d ",cycles);
            // TRACE_DEBUG("PIOA & PA24 = %08x\n", cur);
        // }
        prev = cur;
        cycles++;
    }


    while(1) {
        uint32_t mask =  RST_PHONE | VCC_PHONE;
        while(mask != (AT91C_BASE_PIOA->PIO_PDSR & mask));
        TRACE_DEBUG("ATR\n\r");
        uint8_t atr[] = {0x3B, 0x98, 0x96, 0x00,
                         0x93, 0x94, 0x03, 0x08,
                         0x05, 0x03, 0x03, 0x03};
        phone_txn(atr, sizeof(atr));

        uint8_t c;
        phone_rx(&c); TRACE_DEBUG("rx %02x\n\r", c);
        phone_rx(&c); TRACE_DEBUG("rx %02x\n\r", c);
        phone_rx(&c); TRACE_DEBUG("rx %02x\n\r", c);

        mask = RST_PHONE;
        while(mask == (AT91C_BASE_PIOA->PIO_PDSR & mask));
        TRACE_DEBUG("OFF\n\r");
    }
}


#if 0

/* Phone-side.  This takes care of:

 - power/reset monitoring
 - sequencing ATR / PTS / PTS ack
 - invoking APDU callback

*/


// same as sniffer
enum iso7816_3_state {
    PHONE_S_RESET,
    PHONE_S_WAIT_ATR,
    PHONE_S_TX_ATR,
    PHONE_S_RX_PTS_0,
    PHONE_S_TX_PTS_ACK,
};

 struct phone_state {
     enum iso7816_3_state state;
     uint32_t time;
     uint8_t *buf;
     uint32_t buf_size;

     uint8_t *atr;
     uint32_t atr_size;
};

/* Environment access is abstracted to facilitate testing.
   This is a deeply embedded target: compile-time binding is sufficient. */
static inline int phone_rst(void)    { return AT91C_BASE_PIOA->PIO_PDSR & RST_PHONE; }
static inline int phone_vcc(void)    { return AT91C_BASE_PIOA->PIO_PDSR & VCC_PHONE; }
static inline int phone_rdy_tx(void) { return phone_usart->US_CSR & AT91C_US_TXRDY; }

struct sm_sched {
    /* Read-only */
    uint32_t time;    // current time

    /* Write only */
    uint32_t delay;   // allow for time delay
};

/* Straightforward state machine. Machine-independent. */
void phone_update(struct sm_sched *sched,
                  struct phone_state *s) {
    uint32_t rv = 0;

    if (!(phone_rst() && phone_vcc())) {
        s->state = PHONE_S_RESET;
        return;
    }

    switch(s->state) {
    case PHONE_S_RESET:
        if (phone_rst() && phone_vcc()) {
            s->state = ISO7816_S_WAIT_ATR;
        }
        break;
    case PHONE_S_WAIT_ATR:
        if (s->time >= sched->time) {
            s->state = ISO7816_S_IN_ATR;
            s->buf = s->atr;
            s->buf_size = s->atr_size;
        }
        break;
    case PHONE_S_TX_ATR:
        if (!phone_bus_rdy_tx()) break;
        if (0 != (rv = phone_tx(*(s->buf++)))) {
            // FIXME: ???
        }
        s->buf_size--;
        if (!s->buf_size) {
            s->state = PHONE_S_RX_PTS;
        }
        break;
    case PHONE_S_RX_PTS_0:
        if (!phone_bus_rdy_rx()) break;
        phone_rx(); // FIXME
        break;
    }

}


#endif
