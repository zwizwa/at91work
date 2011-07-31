#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <pit/pit.h>
#include <usb/common/core/USBConfigurationDescriptor.h>
#include <usb/device/core/USBD.h>
#include <utility/led.h>
#include <pmc/pmc.h>

#include <usb/device/multiconf/MULTIDriver.h>

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

#define WAKEUP_CONFIGURE()
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
//          Main
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
/// Initializes drivers and start the USB composite device.
//-----------------------------------------------------------------------------
int main()
{
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- USB Multi Device Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // If they are present, configure Vbus & Wake-up pins
    PIO_InitializeInterrupts(0);
#if 0
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
#endif

    // USB COMPOSITE driver initialization
    MULTIDriver_Initialize();

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

#if 0
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
#endif
        }

        if( USBState == STATE_SUSPEND ) {
            TRACE_DEBUG("suspend  !\n\r");
            //LowPowerMode();
            USBState = STATE_IDLE;
        }
        if( USBState == STATE_RESUME ) {
            // Return in normal MODE
            TRACE_DEBUG("resume !\n\r");
            //NormalPowerMode();
            USBState = STATE_IDLE;
        }
    }
}

