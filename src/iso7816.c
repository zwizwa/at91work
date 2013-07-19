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

enum {
    PHONE_INVALID = 0,
    PHONE_RX,
    PHONE_TX,
} phone_state;

// phone side usart.
static AT91S_USART * const phone_usart = AT91C_BASE_US1;

uint32_t phone_tx(uint8_t c) {
    /* Switch RX/TX state if necessary. */
    if (phone_state != PHONE_TX) {
        phone_usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        phone_state = PHONE_TX;
    }
    /* Flush */
    while((phone_usart->US_CSR & AT91C_US_TXRDY) == 0) {}
    /* Send */
    phone_usart->US_THR = c;
    /* Check */
    uint32_t status = phone_usart->US_CSR
        &(AT91C_US_OVRE|AT91C_US_FRAME|
          AT91C_US_PARE|AT91C_US_TIMEOUT|AT91C_US_NACK|
          (1<<10));
    if (status != 0) {
        TRACE_DEBUG("E:0x%X\n\r",  phone_usart->US_CSR);
        TRACE_DEBUG("Nb:0x%X\n\r", phone_usart->US_NER );
        phone_usart->US_CR =  AT91C_US_RSTSTA;
    }
    return status;
}

uint32_t phone_rx(uint8_t *c) {
    /* Switch RX/TX state if necessary. */
    if (phone_state != PHONE_RX) {
        while((phone_usart->US_CSR & AT91C_US_TXEMPTY) == 0) {}
        phone_usart->US_CR = AT91C_US_RSTSTA | AT91C_US_RSTIT | AT91C_US_RSTNACK;
        phone_state = PHONE_RX;
    }
    return 0;
}

void phone_init(void) {

    /* See at91lib/components/iso7816/iso7816_4.c
       and 31.6.4 in AT91SAM7S128 data sheet.

       For phone socket, clock is always slave. AT91C_US_CKLO is off:
       we never drive the clock pin.  This was on in openpcd-based
       SIMtrace FW.
    */

    /* Enable usart peripheral clock */
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_US0;

    /* I/O and CLK controlled by peripheral A (Table 10-3) */
    AT91C_BASE_PIOA->PIO_ASR = AT91C_PA22_TXD1 | AT91C_PA23_SCK1;
    /* nRST is input */
    AT91C_BASE_PIOA->PIO_ODR = AT91C_PIO_PA24;
    AT91C_BASE_PIOA->PIO_PER = AT91C_PIO_PA24;

    /* Reset and disable receiver & transmitter */
    phone_usart->US_CR =
        AT91C_US_RSTRX
        | AT91C_US_RSTTX
        | AT91C_US_RXDIS
        | AT91C_US_TXDIS;

    /* Set mode to ISO7816 T0, external clock source */
    phone_usart->US_MR =
        AT91C_US_USMODE_ISO7816_0  // T0
        | AT91C_US_CLKS_EXT        // external clock source
        | AT91C_US_CHRL_8_BITS
        | AT91C_US_PAR_EVEN
        | AT91C_US_NBSTOP_1_BIT
        | (3<<24);

    /* In 7816 mode, Baud Rate = Selected Clock / CD / FI_DI_RATIO
       Since we use external clock, CD=1 */
    phone_usart->US_BRGR = 1;

    /* FIXME: Set ATR clock div to 372 */
    phone_usart->US_FIDI = 372 & 0x3FF;

    uint32_t prev = 0;
    while(1) {
        uint32_t cur = AT91C_BASE_PIOA->PIO_PDSR & AT91C_PIO_PA24;
        if (prev != cur)
            printf("PIOA & PA24 = %08x\n", cur);
        prev = cur;
    }

    while(0) {
        uint8_t c = 0x55;
        printf("sending %c\n\r", c);
        phone_tx(c);
    }
}
