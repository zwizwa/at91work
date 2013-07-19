/* Adapted from  "USB device CCID project with AT91SAM Microcontrollers" */

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

//------------------------------------------------------------------------------ 
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <aic/aic.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <utility/trace.h>
#include <utility/assert.h>
#include <utility/led.h>
// #include <usb/device/core/USBD.h>
#include <dbgu/dbgu.h>
//#include <usb/device/ccid/cciddriver.h>
// #include <iso7816/iso7816_4.h>
// #include <pmc/pmc.h>

#include <string.h>

//-----------------------------------------------------------------------------
//         Local variables
//-----------------------------------------------------------------------------

/// List of pins that must be configured for use by the application.
static const Pin pinsISO7816[]    = {PINS_ISO7816};
static const Pin pinIso7816RstMC  = PIN_ISO7816_RSTMC;

static const Pin pinsPower[] = {
	/* Swlf Power (LDO): low = off */
	{1 << 5, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_0, PIO_DEFAULT},
	/* Phone pass-through power: high = off */
	{1 << 26, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT},
};



void phone_init(void);


//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes the CCID driver and runs it.
/// \return Unused (ANSI-C compatibility)
//------------------------------------------------------------------------------
int main( void )
{
    // Initialize traces
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- SIMtrace PHONE side driver \n\r");
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);

    // Configure ISO7816 driver
    PIO_Configure(pinsISO7816, PIO_LISTSIZE(pinsISO7816));
    PIO_Configure(pinsPower, PIO_LISTSIZE(pinsPower));

    /* power up the card */
    PIO_Set(&pinsPower[0]);

    /* Init phone IS7816 USART */
    phone_init();

    // Infinite loop
    while (1) {
    }
    return 0;
}

