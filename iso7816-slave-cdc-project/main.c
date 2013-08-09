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

//-----------------------------------------------------------------------------
/// derived from "USB CDC serial converter"

//-----------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>
#include <string.h>
#include "hexin.h"
#include "iso7816_slave.h"


//------------------------------------------------------------------------------
//      Definitions
//------------------------------------------------------------------------------
#ifndef AT91C_ID_TC0
    #define AT91C_ID_TC0 AT91C_ID_TC
#endif

/// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

/// Use for power management
#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

//------------------------------------------------------------------------------
//      Internal variables
//------------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

/// List of pins that must be configured for use by the application.
static const Pin pins[] = {PIN_USART1_TXD, PIN_USART1_RXD};


/// Buffer for storing incoming USB data.
static unsigned char usbBuffer[DATABUFFERSIZE];

//------------------------------------------------------------------------------
//         VBus monitoring (optional)
//------------------------------------------------------------------------------

#define VBUS_CONFIGURE()    USBD_Connect()

//------------------------------------------------------------------------------
/// Put the CPU in 32kHz, disable PLL, main oscillator
/// Put voltage regulator in standby mode
//------------------------------------------------------------------------------
void LowPowerMode(void)
{
    // MCK=48MHz to MCK=32kHz
    // MCK = SLCK/2 : change source first from 48 000 000 to 18. / 2 = 9M
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // MCK=SLCK : then change prescaler
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_CSS_SLOW_CLK;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // disable PLL
    AT91C_BASE_PMC->PMC_PLLR = 0;
    // Disable Main Oscillator
    AT91C_BASE_PMC->PMC_MOR = 0;

    // Voltage regulator in standby mode : Enable VREG Low Power Mode
    AT91C_BASE_VREG->VREG_MR |= AT91C_VREG_PSTDBY;

    PMC_DisableProcessorClock();
}

//------------------------------------------------------------------------------
/// Put voltage regulator in normal mode
/// Return the CPU to normal speed 48MHz, enable PLL, main oscillator
//------------------------------------------------------------------------------
void NormalPowerMode(void)
{
    // Voltage regulator in normal mode : Disable VREG Low Power Mode
    AT91C_BASE_VREG->VREG_MR &= ~AT91C_VREG_PSTDBY;

    // MCK=32kHz to MCK=48MHz
    // enable Main Oscillator
    AT91C_BASE_PMC->PMC_MOR = (( (AT91C_CKGR_OSCOUNT & (0x06 <<8)) | AT91C_CKGR_MOSCEN ));
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS ) );

    // enable PLL@96MHz
    AT91C_BASE_PMC->PMC_PLLR = ((AT91C_CKGR_DIV & 0x0E) |
         (AT91C_CKGR_PLLCOUNT & (28<<8)) |
         (AT91C_CKGR_MUL & (0x48<<16)));
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK ) );
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;
    // MCK=SLCK/2 : change prescaler first
    AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );
    // MCK=PLLCK/2 : then change source
    AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK  ;
    while( !( AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY ) );

}


//------------------------------------------------------------------------------
//         Callbacks re-implementation
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// Invoked when the USB device leaves the Suspended state. By default,
/// configures the LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
    // Initialize LEDs
    LED_Configure(USBD_LEDPOWER);
    LED_Set(USBD_LEDPOWER);
    LED_Configure(USBD_LEDUSB);
    LED_Clear(USBD_LEDUSB);
    USBState = STATE_RESUME;
}

//------------------------------------------------------------------------------
/// Invoked when the USB device gets suspended. By default, turns off all LEDs.
//------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
    // Turn off LEDs
    LED_Clear(USBD_LEDPOWER);
    LED_Clear(USBD_LEDUSB);
    USBState = STATE_SUSPEND;
}


/* Use newline-terminated ASCII hex on CDC I/O to wrap a binary packet
   interface in the simplest way possible.

   This ad-hoc protocol was easiest to implement considering a single
   use case: a python program tying into pyscard.

   Replacing the packet transport with something else is trivial: see
   is7816*_apdu_* calls. */

static uint8_t hexin_buf[512];
static struct hexin h = {.buf = hexin_buf};
static struct iso7816_slave *iso7816_slave;

static char hexout_buf[DATABUFFERSIZE];


//------------------------------------------------------------------------------
/// Callback invoked when data has been received on the USB.
//------------------------------------------------------------------------------
static void UsbDataReceived(unsigned int unused,
                            unsigned char status,
                            unsigned int received,
                            unsigned int remaining)
{
    // Check that data has been received successfully
    if (status == USBD_STATUS_SUCCESS) {

        //usbBuffer[received] = 0; // FIXME: hack!
        //TRACE_DEBUG("usb_in: %s\n\r", usbBuffer);

        int i;
        for (i=0; i<received; i++) {
            int rv = hexin_push(&h, usbBuffer[i]);
            if (rv > 0) {
                /* Binary packet received: route it. */
                if (rv < 2) {
                    TRACE_ERROR("Short packet, size %d\n\r", rv);
                }
                else {
                    uint16_t sw = hexin_buf[rv-1] + (hexin_buf[rv-2]<<8);
                    switch(sw) {
                    case 0xFFFF:
                        /* Unused R-APDU status word: we use this as protocol escape.
                           FIXME: Might want to do this better */
                        iso7816_slave_command(iso7816_slave, hexin_buf, rv-2);
                        break;
                    default:
                        iso7816_slave_r_apdu_write(iso7816_slave, hexin_buf, rv);
                        break;
                    }
                }

                /* Prepare to receive next. */
                hexin_reset(&h);
                bzero(hexin_buf, sizeof(hexin_buf));
            }
        }

        // Check if bytes have been discarded
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {
            TRACE_WARNING(
                      "UsbDataReceived: %u bytes discarded\n\r",
                      remaining);
        }
    }
    else {

        TRACE_WARNING( "UsbDataReceived: Transfer error\n\r");
    }


    // Make sure we get activated next time.
    CDCDSerialDriver_Read(usbBuffer,
                          DATABUFFERSIZE,
                          (TransferCallback) UsbDataReceived,
                          0);
}



/* Transfer next chunk of HEX data until done. */
static void UsbDataSent(void *pArg,
                        unsigned char status,
                        unsigned int transferred,
                        unsigned int remaining)
{
    // FIXME: handle errors.  Function args are ignored.

    int char_index = 0;
    int rv = 0;
    while((char_index < (DATABUFFERSIZE-1)) &&
          ((rv = iso7816_slave_c_apdu_getc(iso7816_slave)) >= 0)) {
        sprintf(hexout_buf + char_index, "%02X", (uint8_t)rv);
        char_index += 2;
    }
    TransferCallback callback;
    if (rv < 0) {
        // Don't print \r -> python readline() don't like
        sprintf(hexout_buf + char_index, "\n");
        callback = NULL;
    }
    else {
        // Send more later.
        callback = UsbDataSent;
    }
    CDCDSerialDriver_Write(hexout_buf, strlen(hexout_buf),
                           callback, 0);
}

static void c_apdu_cb(void *ctx, int size) {
    UsbDataSent(0, 0, 0, 0);
}

/* SIMtrace PHONE interface uses VENDOR requests.
   Patched in CDCDSerialDriver.c : USBDCallbacks_RequestReceived.
   FIXME: once CCID driver is done, patch in the same way.
   FIXME: should this use INTERFACE requests and define a second interface?
*/

static const uint8_t dummy[] = {1,2,3,4};

void Vendor_RequestHandler(const USBGenericRequest *request) {
    TRACE_WARNING("Vendor_RequestHandler\n\r");
    switch(request->bmRequestType >> 7 & 1) {
    case USBGenericRequest_OUT:
        TRACE_WARNING("USBGenericRequest_OUT\n\r");
        break;
    case USBGenericRequest_IN:  // e.g. 0xC1
        TRACE_WARNING("USBGenericRequest_IN\n\r");
        USBD_Write(0, dummy, sizeof(dummy), 0, 0);
        break;
    }
    // USBD_Stall(0);
}



//------------------------------------------------------------------------------
//          Main
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes drivers and start the USB <-> Serial bridge.
//------------------------------------------------------------------------------
int main()
{
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Device CDC Serial Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);


    // BOT driver initialization
    CDCDSerialDriver_Initialize();

    // connect if needed
    VBUS_CONFIGURE();

    /* Init phone ISO7816 USART */
    iso7816_slave = iso7816_slave_init(c_apdu_cb, NULL);

    // Driver loop
    while (1) {
        // Poll I/O state machine.  FIXME: disable interrupts as there
        // is contention with USB callbacks.
        iso7816_slave_tick(iso7816_slave);

        // Device is not configured
        if (USBD_GetState() < USBD_STATE_CONFIGURED) {

            // Connect pull-up, wait for configuration
            USBD_Connect();
            while (USBD_GetState() < USBD_STATE_CONFIGURED);

            // Start receiving data on the USB
            CDCDSerialDriver_Read(usbBuffer,
                                  DATABUFFERSIZE,
                                  (TransferCallback) UsbDataReceived,
                                  0);
        }
        if( USBState == STATE_SUSPEND ) {
            TRACE_DEBUG("suspend  !\n\r");
            LowPowerMode();
            USBState = STATE_IDLE;
        }
        if( USBState == STATE_RESUME ) {
            // Return in normal MODE
            TRACE_DEBUG("resume !\n\r");
            NormalPowerMode();
            USBState = STATE_IDLE;
        }
    }
}

