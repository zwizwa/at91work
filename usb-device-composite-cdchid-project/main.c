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
/// \dir "USB COMPOSITE CDC+HID project"
///
/// !!!Purpose
///
/// The USB COMPOSITE Project will help you to get familiar with the
/// USB Device Port(UDP)interface and also some of the other interfaces in
/// AT91SAM microcontrollers. Also it can help you to be familiar with the USB
/// Framework that is used for rapid development of USB-compliant class
/// drivers such as USB Communication Device class (CDC), and how to combine
/// two USB functions to a single composite device (such as CDC + MSD).
///
/// You can find following information depends on your needs:
/// - Sample usage of USB Device Framework.
/// - USB COMPOSITE device and functions driver development based on the AT91
///   USB Device Framework and other re-usable class driver code.
/// - USB enumerate sequence, the standard and class-specific descriptors and
///   requests handling.
/// - The initialize sequence and usage of UDP interface.
///
/// !See
/// - pio: Pin configurations and peripheral configure.
/// - memories: Storage Media interface for MSD
/// - ssc: SSC interface driver
/// - "dac-at73c213": I2S codec at73c213 driver
/// - usb: USB Device Framework, USB CDC driver and UDP interface driver
///    - "AT91 USB device framework"
///        - "USBD API"
///    - "composite"
///       - "USB COMPOSITE Device"
///    - "hid-keyboard"
///       - "USB HID Keyboard"
///    - "cdc-serial"
///       - "USB CDC Serial Device"
/// - projects:
///    - "usb-device-hid-keyboard-project"
///    - "usb-device-cdc-serial-project"
///
/// !!!Requirements
///
/// This package can be used with some of Atmel evaluation kits that have UDP
/// interface, depending on the functions included.
///
/// The current supported board list:
///    - at91sam7s-ek (exclude at91sam7s32)
///    - at91sam7x-ek
///    - at91sam7xc-ek
///    - at91sam7a3-ek
///    - at91sam7se-ek
///    - at91sam9260-ek
///    - at91sam9263-ek
///
///  !!!Windows Driver Update
///
/// The composite device is generally supported by Microsoft windows, but some
/// patches are needed for muti-interface functions such as CDC & Audio. The
/// example composite devices are tested under windows XP (SP3). For CDC
/// serial port, additional windows driver file (CompositeCDCSerial.inf) can
/// be found at at91lib\usb\device\composite\drv.
///
/// The following is alternate update to fix the composite device support
/// on windows XP:
///
/// !!Install Windows Service Pack 3 (SP3)
///
/// All the fixes for USB generic driver are included in window XP service pack
/// 3. It can be found at
/// http://technet.microsoft.com/zh-cn/windows/bb794714(en-us).aspx .
///
/// !!Install Windows Hot Fixes
///
/// Two hot fixes are necessary for window to recognize the composite device
/// correctly:
///
/// -# http://support.microsoft.com/kb/814560
/// -# http://support.microsoft.com/kb/918365
/// 
/// !!!Description
///
/// When an EK running this program connected to a host (PC for example), with
/// USB cable, host will notice the attachment of a USB %device. No %device
/// driver offered for the %device now.
///
/// !!!Usage
///
/// -# Build the program and download it inside the evaluation board. Please
///    refer to the
///    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">
///    SAM-BA User Guide</a>, the
///    <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">
///    GNU-Based Software Development</a> application note or to the
///    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
///    IAR EWARM User Guide</a>, depending on your chosen solution.
/// -# On the computer, open and configure a terminal application
///    (e.g. HyperTerminal on Microsoft Windows) with these settings:
///   - 115200 bauds
///   - 8 bits of data
///   - No parity
///   - 1 stop bit
///   - No flow control
/// -# Start the application.
/// -# In the terminal window, the following text should appear:
///     \code
///     -- USB Composite Device Project xxx --
///     -- AT91xxxxxx-xx
///     -- Compiled: xxx xx xxxx xx:xx:xx --
///     \endcode
/// -# When connecting USB cable to windows, the LED blinks, and the host
///    reports a new USB %device attachment.
/// -# For the windows driver installation and the test functions, please
///      refer to "USB CDC serial converter" &
///      "USB HID Keyboard Project".
/// -# You can use the inf file
///    at91lib\\usb\\device\\composite\\drv\\CompositeCDCSerial.inf
///    to install the CDC serial  port.
///
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains all the specific code for the
/// usb-device-composite-cdchid-project
///
/// !Contents
///
/// The code can be roughly broken down as follows:
///    - Configuration functions
///       - VBus_Configure
///       - PIO configurations in start of main
///    - Interrupt handlers
///       - ISR_Vbus
///    - Callback functions
///       - USBDCallbacks_RequestReceived
///    - The main function, which implements the program behavior
///
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <pit/pit.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/common/hid/HIDKeypad.h>
#include <utility/led.h>
#include <pmc/pmc.h>

#include <usb/device/composite/COMPOSITEDDriver.h>

#include <string.h>

//-----------------------------------------------------------------------------
//      Definitions
//-----------------------------------------------------------------------------
#ifndef AT91C_ID_TC0
#if defined(AT91C_ID_TC012)
    #define AT91C_ID_TC0 AT91C_ID_TC012
#elif defined(AT91C_ID_TC)
    #define AT91C_ID_TC0 AT91C_ID_TC
#else
    #error Pb define ID_TC
#endif
#endif

/// Master clock frequency in Hz
#define MCK             BOARD_MCK

/// Number of keys used in the example.
#define NUM_KEYS                    4

/// Number of non-modifiers keys.
#define NUM_NORMAL_KEYS             3

/// Number of modifier keys.
#define NUM_MODIFIER_KEYS           (NUM_KEYS - NUM_NORMAL_KEYS)

/// Num lock LED index.
#define LED_NUMLOCK                 USBD_LEDOTHER

/// Delay for pushbutton debouncing (ms)
#define DEBOUNCE_TIME      10

/// PIT period value (useconds)
#define PIT_PERIOD        1000

/// Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE    BOARD_USB_ENDPOINTS_MAXPACKETSIZE(2)

/// Use for power management
#define STATE_IDLE    0
/// The USB device is in suspend state
#define STATE_SUSPEND 4
/// The USB device is in resume state
#define STATE_RESUME  5

//-----------------------------------------------------------------------------
//      Internal variables
//-----------------------------------------------------------------------------
/// State of USB, for suspend and resume
unsigned char USBState = STATE_IDLE;

//- CDC
/// List of pins that must be configured for use by the application.
static const Pin pinsUsart[] = {PIN_USART0_TXD, PIN_USART0_RXD};

/// Double-buffer for storing incoming USART data.
static unsigned char usartBuffers[2][DATABUFFERSIZE];

/// Current USART buffer index.
static unsigned char usartCurrentBuffer = 0;

/// Buffer for storing incoming USB data.
static unsigned char usbSerialBuffer0[DATABUFFERSIZE];
//static unsigned char usbSerialBuffer1[DATABUFFERSIZE];

//- HID
/// List of pinsPushButtons to configure for the applicatino.
static Pin pinsPushButtons[] = {PINS_PUSHBUTTONS};

/// Array of key codes produced by each button.
static unsigned char keyCodes[NUM_KEYS] = {
    HIDKeypad_A,
    HIDKeypad_NUMLOCK,
    HIDKeypad_9,
    HIDKeypad_RIGHTSHIFT
};

/// Current status (pressed or not) for each key.
static unsigned char keyStatus[NUM_KEYS];

//------------------------------------------------------------------------------
//         Remote wake-up support (optional)
//------------------------------------------------------------------------------ 
#if (BOARD_USB_BMATTRIBUTES == USBConfigurationDescriptor_BUSPOWERED_RWAKEUP) \
    || (BOARD_USB_BMATTRIBUTES == USBConfigurationDescriptor_SELFPOWERED_RWAKEUP)

#define WAKEUP_CONFIGURE()  ConfigureWakeUp()

/// Button for Wake-UP the USB device.
static const Pin pinWakeUp = PIN_PUSHBUTTON_1;

//------------------------------------------------------------------------------
/// Interrupt service routine for the PIT. Debounces the wake-up pin input.
//------------------------------------------------------------------------------
static void ISR_Pit(void)
{
    static unsigned long debounceCounter = DEBOUNCE_TIME;
    unsigned long pisr = 0;

    // Read the PISR
    pisr = PIT_GetStatus() & AT91C_PITC_PITS;

    if (pisr != 0) {

        // Read the PIVR. It acknowledges the IT
        PIT_GetPIVR();
    }

    // Button released
    if (PIO_Get(&pinWakeUp)) {

        debounceCounter = DEBOUNCE_TIME;
    }
    // Button still pressed
    else {

        debounceCounter--;
    }

    // End of debounce time
    if (debounceCounter == 0) {

        debounceCounter = DEBOUNCE_TIME;
        PIT_DisableIT();
        AT91C_BASE_PITC->PITC_PIMR &= ~AT91C_PITC_PITEN;
        COMPOSITEDDriver_RemoteWakeUp();
    }
}

//------------------------------------------------------------------------------
/// Configures the PIT to generate 1ms ticks.
//------------------------------------------------------------------------------
static void ConfigurePit(void)
{
    // Initialize and enable the PIT
    PIT_Init(PIT_PERIOD, BOARD_MCK / 1000000);

    // Disable the interrupt on the interrupt controller
    AIC_DisableIT(AT91C_ID_SYS);

    // Configure the AIC for PIT interrupts
    AIC_ConfigureIT(AT91C_ID_SYS, 0, ISR_Pit);

    // Enable the interrupt on the interrupt controller
    AIC_EnableIT(AT91C_ID_SYS);

    // Enable the interrupt on the pit
    PIT_EnableIT();

    // Enable the pit
    PIT_Enable();
}

//------------------------------------------------------------------------------
/// Interrupt service routine for the remote wake-up pin. Starts the debouncing
/// sequence.
//------------------------------------------------------------------------------
static void WakeUpHandler(const Pin *pin)
{
    TRACE_DEBUG("Wake-up handler\n\r");

    // Check current level on the remote wake-up pin
    if (!PIO_Get(&pinWakeUp)) {

        ConfigurePit();
    }
}

//------------------------------------------------------------------------------
/// Configures the wake-up pin to generate interrupts.
//------------------------------------------------------------------------------
static void ConfigureWakeUp(void)
{
    TRACE_INFO("Wake-up configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinWakeUp, 1);
    PIO_ConfigureIt(&pinWakeUp, WakeUpHandler);
    PIO_EnableIt(&pinWakeUp);
}

#else
    #define WAKEUP_CONFIGURE()
#endif

//-----------------------------------------------------------------------------
//         VBus monitoring (optional)
//-----------------------------------------------------------------------------
#if defined(PIN_USB_VBUS)

#define VBUS_CONFIGURE()  VBus_Configure()

/// VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

//-----------------------------------------------------------------------------
/// Handles interrupts coming from PIO controllers.
//-----------------------------------------------------------------------------
static void ISR_Vbus(const Pin *pPin)
{
    TRACE_INFO("VBUS ");

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else {

        TRACE_INFO("discon\n\r");
        USBD_Disconnect();
    }
}

//-----------------------------------------------------------------------------
/// Configures the VBus pin to trigger an interrupt when the level on that pin
/// changes.
//-----------------------------------------------------------------------------
static void VBus_Configure( void )
{
    TRACE_INFO("VBus configuration\n\r");

    // Configure PIO
    PIO_Configure(&pinVbus, 1);
    PIO_ConfigureIt(&pinVbus, ISR_Vbus);
    PIO_EnableIt(&pinVbus);

    // Check current level on VBus
    if (PIO_Get(&pinVbus)) {

        // if VBUS present, force the connect
        TRACE_INFO("conn\n\r");
        USBD_Connect();
    }
    else {
        USBD_Disconnect();
    }           
}

#else
    #define VBUS_CONFIGURE()    USBD_Connect()
#endif //#if defined(PIN_USB_VBUS)

#if defined (CP15_PRESENT)
//------------------------------------------------------------------------------
/// Put the CPU in 32kHz, disable PLL, main oscillator
/// Put voltage regulator in standby mode
//------------------------------------------------------------------------------
void LowPowerMode(void)
{
    PMC_CPUInIdleMode();
}
//------------------------------------------------------------------------------
/// Put voltage regulator in normal mode
/// Return the CPU to normal speed 48MHz, enable PLL, main oscillator
//------------------------------------------------------------------------------
void NormalPowerMode(void)
{
}

#elif defined(at91sam7a3)
//------------------------------------------------------------------------------
/// Put the CPU in 32kHz, disable PLL, main oscillator
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

    PMC_DisableProcessorClock();
}
//------------------------------------------------------------------------------
/// Return the CPU to normal speed 48MHz, enable PLL, main oscillator
//------------------------------------------------------------------------------
void NormalPowerMode(void)
{
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

#elif defined (at91sam7se)
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

#elif defined (at91sam7s)
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

#elif defined (at91sam7x) || defined (at91sam7xc)
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

#endif

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



//-----------------------------------------------------------------------------
//         Internal functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Handles interrupts coming from Timer #0.
//-----------------------------------------------------------------------------
static void ISR_Timer0()
{
    unsigned char size;
    unsigned int status = AT91C_BASE_TC0->TC_SR;

    if ((status & AT91C_TC_CPCS) != 0) {
    
        // Flush PDC buffer
        size = DATABUFFERSIZE - AT91C_BASE_US0->US_RCR;
        if (size == 0) {

            AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
            return;
        }
        AT91C_BASE_US0->US_RCR = 0;
    
        // Send current buffer through the USB
        while (CDCDSerialDriver_Write(0, usartBuffers[usartCurrentBuffer],
                                      size, 0, 0) != USBD_STATUS_SUCCESS);
    
        // Restart read on buffer
        USART_ReadBuffer(AT91C_BASE_US0,
                         usartBuffers[usartCurrentBuffer],
                         DATABUFFERSIZE);
        usartCurrentBuffer = 1 - usartCurrentBuffer;
        AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }
}

//-----------------------------------------------------------------------------
/// Callback invoked when data has been received on the USB.
//-----------------------------------------------------------------------------
static void UsbDataReceived0(unsigned int unused,
                             unsigned char status,
                             unsigned int received,
                             unsigned int remaining)
{
    // Check that data has been received successfully
    if (status == USBD_STATUS_SUCCESS) {

        // Send data through USART
        while (!USART_WriteBuffer(AT91C_BASE_US0, usbSerialBuffer0, received));
        AT91C_BASE_US0->US_IER = AT91C_US_TXBUFE;

        // Check if bytes have been discarded
        if ((received == DATABUFFERSIZE) && (remaining > 0)) {

            TRACE_WARNING(
                      "UsbDataReceived: %u bytes discarded\n\r",
                      remaining);
        }
    }
    else {

        TRACE_WARNING("UsbDataReceived: Transfer error\n\r");
    }
}

//-----------------------------------------------------------------------------
/// Handles interrupts coming from USART #0.
//-----------------------------------------------------------------------------
static void ISR_Usart0()
{
    unsigned int status = AT91C_BASE_US0->US_CSR;
    unsigned short serialState;

    // If USB device is not configured, do nothing
    if (USBD_GetState() != USBD_STATE_CONFIGURED) {

        AT91C_BASE_US0->US_IDR = 0xFFFFFFFF;
        return;
    }

    // Buffer has been read successfully
    if ((status & AT91C_US_ENDRX) != 0) {

        // Disable timer
        AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;

        // Send buffer through the USBSerial0
        while (CDCDSerialDriver_Write(0, usartBuffers[usartCurrentBuffer],
                                 DATABUFFERSIZE, 0, 0) != USBD_STATUS_SUCCESS);

        // Restart read on buffer
        USART_ReadBuffer(AT91C_BASE_US0,
                         usartBuffers[usartCurrentBuffer],
                         DATABUFFERSIZE);
        usartCurrentBuffer = 1 - usartCurrentBuffer;

        // Restart timer
        AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;
    }

    // Buffer has been sent
    if ((status & AT91C_US_TXBUFE) != 0) {

        // Restart USB read
        CDCDSerialDriver_Read(0, usbSerialBuffer0,
                              DATABUFFERSIZE,
                              (TransferCallback) UsbDataReceived0,
                              0);
        AT91C_BASE_US0->US_IDR = AT91C_US_TXBUFE;
    }

    // Errors
    serialState = CDCDSerialDriver_GetSerialState(0);

    // Overrun
    if ((status & AT91C_US_OVER) != 0) {

        TRACE_WARNING("ISR_Usart0: Overrun\n\r");
        serialState |= CDCD_STATE_OVERRUN;
    }

    // Framing error
    if ((status & AT91C_US_FRAME) != 0) {

        TRACE_WARNING("ISR_Usart0: Framing error\n\r");
        serialState |= CDCD_STATE_FRAMING;
    }

    CDCDSerialDriver_SetSerialState(0, serialState);
}

//-----------------------------------------------------------------------------
/// Invoked when the status of the keyboard LEDs changes. Turns the num. lock
/// LED on or off.
/// \param numLockStatus Indicates the current status of the num. lock key.
/// \param capsLockStatus Indicates the current status of the caps lock key.
/// \param scrollLockStatus Indicates the current status of the scroll lock key
//-----------------------------------------------------------------------------
void HIDDKeyboardCallbacks_LedsChanged(
    unsigned char numLockStatus,
    unsigned char capsLockStatus,
    unsigned char scrollLockStatus)
{
    // Num. lock
    if (numLockStatus) {

        LED_Set(LED_NUMLOCK);
    }
    else {

        LED_Clear(LED_NUMLOCK);
    }
}

//-----------------------------------------------------------------------------
//         Internal functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Monitor keyboard buttons & Update key status in HID driver
//-----------------------------------------------------------------------------
static void HIDDKeyboardProcessKeys(void)
{
    unsigned int i;
    unsigned char pressedKeys[NUM_KEYS];
    unsigned char pressedKeysSize = 0;
    unsigned char releasedKeys[NUM_KEYS];
    unsigned char releasedKeysSize = 0;
    
    // Monitor buttons
    for (i=0; i < PIO_LISTSIZE(pinsPushButtons); i++) {
    
        // Check if button state has changed
        unsigned char isButtonPressed = PIO_Get(&(pinsPushButtons[i]));
        if (isButtonPressed != keyStatus[i]) {
    
            // Update button state
            if (!isButtonPressed) {
    
                // Key has been pressed
                TRACE_INFO("-I- Key %u has been pressed\n\r", i);
                keyStatus[i] = 0;
                pressedKeys[pressedKeysSize] = keyCodes[i];
                pressedKeysSize++;
                HIDDKeyboardDriver_RemoteWakeUp();
            }
            else {
    
                // Key has been released
                TRACE_INFO("-I- Key %u has been released\n\r", i);
                keyStatus[i] = 1;
                releasedKeys[releasedKeysSize] = keyCodes[i];
                releasedKeysSize++;
            }
        }
    }
    
    // Update key status in the HID driver if necessary
    if ((pressedKeysSize != 0) || (releasedKeysSize != 0)) {
    
        unsigned char status;

        do {
        
            status = HIDDKeyboardDriver_ChangeKeys(pressedKeys,
                                                   pressedKeysSize,
                                                   releasedKeys,
                                                   releasedKeysSize);
        }
        while (status != USBD_STATUS_SUCCESS);
    }
}

//-----------------------------------------------------------------------------
//         Exported function
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
//          Main
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Initializes drivers and start the USB composite device.
//-----------------------------------------------------------------------------
int main()
{
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Composite Device Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);

    // ----- HID Function Initialize
    // Initialize key statuses and configure push buttons
    PIO_Configure(pinsPushButtons, PIO_LISTSIZE(pinsPushButtons));
    memset(keyStatus, 1, NUM_KEYS);

    // Configure LEDs
    LED_Configure(LED_NUMLOCK);

    // ----- CDC Function Initialize
    // Configure USART
    PIO_Configure(pinsUsart, PIO_LISTSIZE(pinsUsart));
    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_US0;
    AT91C_BASE_US0->US_IDR = 0xFFFFFFFF;
    USART_Configure(AT91C_BASE_US0,
                    USART_MODE_ASYNCHRONOUS,
                    115200,
                    MCK);
    USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
    USART_SetReceiverEnabled(AT91C_BASE_US0, 1);
    AIC_ConfigureIT(AT91C_ID_US0, 0, ISR_Usart0);
    AIC_EnableIT(AT91C_ID_US0);

    // Configure timer 0
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_TC0);
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV5_CLOCK
                             | AT91C_TC_CPCSTOP
                             | AT91C_TC_CPCDIS
                             | AT91C_TC_WAVESEL_UP_AUTO
                             | AT91C_TC_WAVE;
    AT91C_BASE_TC0->TC_RC = 0x00FF;
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;
    AIC_ConfigureIT(AT91C_ID_TC0, 0, ISR_Timer0);
    AIC_EnableIT(AT91C_ID_TC0);

    // USB COMPOSITE driver initialization
    COMPOSITEDDriver_Initialize();

    WAKEUP_CONFIGURE();

    // connect if needed
    VBUS_CONFIGURE();

    // Driver loop
    while (1) {

        // Device is not configured
        if (USBD_GetState() < USBD_STATE_CONFIGURED) {

            // Connect pull-up, wait for configuration
            USBD_Connect();
            while (USBD_GetState() < USBD_STATE_CONFIGURED);

            // Start receiving data on the USART
            usartCurrentBuffer = 0;
            USART_ReadBuffer(AT91C_BASE_US0, usartBuffers[0], DATABUFFERSIZE);
            USART_ReadBuffer(AT91C_BASE_US0, usartBuffers[1], DATABUFFERSIZE);
            AT91C_BASE_US0->US_IER = AT91C_US_ENDRX
                                     | AT91C_US_FRAME
                                     | AT91C_US_OVER;
            AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN | AT91C_TC_SWTRG;

            // Start receiving data on the USB
            CDCDSerialDriver_Read(0, usbSerialBuffer0,
                                  DATABUFFERSIZE,
                                  (TransferCallback) UsbDataReceived0,
                                  0);
        }
        else {

            HIDDKeyboardProcessKeys();
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

