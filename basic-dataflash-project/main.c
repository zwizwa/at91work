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
/// \dir "Basic-dataflash-project"
///
/// !!!Purpose
///
/// The Basic Dataflash project will help new users get familiar with SPI interface 
/// on Atmel's AT91 family of microcontrollers. This project gives you an AT45 
/// Dataflash programming code so that can help develop your own SPI devices
/// applications with maximum efficiency.
///
/// You can find following information depends on your needs:
/// - A Spi low level driver performs SPI device Initializes, data transfer and 
/// receive. It can be used by upper SPI driver such as AT45 %dataflash.
/// - A Dataflash driver is based on top of the corresponding Spi driver.
/// It allow user to do operations with %dataflash in a unified way.
///
/// !See also
///    - "spi-flash": Dataflash interface driver.
///
/// !!!Requirements
///
/// This package can be used with all Atmel evaluation kits that have SPI
/// interface and on-board or external Serialflash connected. The package runs at 
/// SRAM or SDRAM, so SDRAM device is needed if you want to run this package in SDRAM.
///
///
/// !!!Description
///
/// The demonstration program tests the dataflash present on the evaluation kit by 
/// erasing and writing each one of its pages.
///
/// !!!Usage
///
/// -# Build the program and download it inside the evaluation board. Please
///    refer to the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6224.pdf">SAM-BA User Guide</a>,
///    the <a href="http://www.atmel.com/dyn/resources/prod_documents/doc6310.pdf">GNU-Based Software Development</a>
///    application note or to the <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">IAR EWARM User Guide</a>,
///    depending on your chosen solution.
/// -# On the computer, open and configure a terminal application
///    (e.g. HyperTerminal on Microsoft Windows) with these settings:
///   - 115200 bauds
///   - 8 bits of data
///   - No parity
///   - 1 stop bit
///   - No flow control
/// -# Start the application.
/// -# Upon startup, the application will output the following lines on the DBGU.
///    \code
///    -- Basic Dataflash Project xxx --
///    -- AT91xxxxxx-xx
///    -- Compiled: xxx xx xxxx xx:xx:xx --
///    -I- Initializing the SPI and AT45 drivers
///    -I- At45 enabled
///    -I- SPI interrupt enabled
///    -I- Waiting for a dataflash to be connected ...
///    \endcode
/// -# As soon as a dataflash is connected, the tests will start. Eventually, 
///    the test result (pass or fail) will be output on the DBGU.
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains all the specific code for the basic-dataflash-project.
/// It tests the dataflash present on the evaluation kit by erasing and writing 
/// each one of its pages
/// 
/// !Contents
/// The code can be roughly broken down as follows:
///    - AT45 Dataflash write data function.
///    - AT45 Dataflash read data function.
///    - AT45 Dataflash erase function.
///    - Other AT45 functions (such as AT45_GetStatus())
///    - The main() function, which implements the program behavior.
///       - Initializes an AT45 instance and configures SPI chip select pin.
///       - Config SPI Interrupt Service Routine.
///       - Identifier the AT45 device connected to the evaluation kit.
///       - Test the dataflash by erasing and writing each one of its pages.
/// 
/// !See also
///    - "spi-flash": Dataflash interface driver.
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <aic/aic.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <spi-flash/at45.h>

#include <string.h>

//------------------------------------------------------------------------------
//         Local definitions
//------------------------------------------------------------------------------

/// SPI clock frequency, in Hz.
#define SPCK        1000000

/// If there is no on-board dataflash socket, use SPI0/NPCS0.
#ifndef BOARD_AT45_A_SPI_PINS
    #ifdef AT91C_BASE_SPI0
        #define BOARD_AT45_A_SPI_BASE           AT91C_BASE_SPI0
        #define BOARD_AT45_A_SPI_ID             AT91C_ID_SPI0
        #define BOARD_AT45_A_SPI_PINS           PINS_SPI0
        #define BOARD_AT45_A_NPCS_PIN           PIN_SPI0_NPCS0
    #else
        #define BOARD_AT45_A_SPI_BASE           AT91C_BASE_SPI
        #define BOARD_AT45_A_SPI_ID             AT91C_ID_SPI
        #define BOARD_AT45_A_SPI_PINS           PINS_SPI
        #define BOARD_AT45_A_NPCS_PIN           PIN_SPI_NPCS0
    #endif
    #define BOARD_AT45_A_SPI                    0
    #define BOARD_AT45_A_NPCS                   0
#endif

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/// SPI driver instance.
static Spid spid;

/// AT45 driver instance.
static At45 at45;

/// Pins used by the application.
static const Pin pins[]  = {BOARD_AT45_A_SPI_PINS, BOARD_AT45_A_NPCS_PIN};

/// Page buffer.
static unsigned char pBuffer[2112];

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// SPI interrupt handler. Invokes the SPI driver handler to check for pending
/// interrupts.
//------------------------------------------------------------------------------
static void ISR_Spi(void)
{
    SPID_Handler(&spid);
}

//------------------------------------------------------------------------------
/// Retrieves and returns the At45 current status, or 0 if an error
/// happened.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT45_GetStatus(At45 *pAt45)
{
    unsigned char error;
    unsigned char status;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_GetStatus: pAt45 is null\n\r");

    // Issue a status register read command
    error = AT45_SendCommand(pAt45, AT45_STATUS_READ, 1, &status, 1, 0, 0, 0);
    ASSERT(!error, "-F- AT45_GetStatus: Failed to issue command.\n\r");

    // Wait for command to terminate
    while (AT45_IsBusy(pAt45));

    return status;
}

//------------------------------------------------------------------------------
/// Waits for the At45 to be ready to accept new commands.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static void AT45_WaitReady(At45 *pAt45) 
{
    unsigned char ready = 0;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_WaitUntilReady: pAt45 is null\n\r");

    // Poll device until it is ready
    while (!ready) {

        ready = AT45_STATUS_READY(AT45_GetStatus(pAt45));
    }
}

//------------------------------------------------------------------------------
/// Reads and returns the JEDEC identifier of a At45.
/// \param pAt45  Pointer to a At45 driver instance.
//------------------------------------------------------------------------------
static unsigned int AT45_GetJedecId(At45 *pAt45)
{
    unsigned char error;
    unsigned int id;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_GetJedecId: pAt45 is null\n\r");

    // Issue a manufacturer and device ID read command
    error = AT45_SendCommand(pAt45, AT45_ID_READ, 1, (void *) &id, 4, 0, 0, 0);
    ASSERT(!error, "-F- AT45_GetJedecId: Could not issue command.\n\r");

    // Wait for transfer to finish
    while (AT45_IsBusy(pAt45));

    return id;
}

//------------------------------------------------------------------------------
/// Reads data from the At45 inside the provided buffer. Since a continuous
/// read command is used, there is no restriction on the buffer size and read
/// address.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param pBuffer  Data buffer.
/// \param size  Number of bytes to read.
/// \param address  Address at which data shall be read.
//------------------------------------------------------------------------------
static void AT45_Read(
    At45 *pAt45,
    unsigned char *pBuffer,
    unsigned int size,
    unsigned int address) 
{
    unsigned char error;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_Read: pAt45 is null\n\r");
    ASSERT(pBuffer, "-F- AT45_Read: pBuffer is null\n\r");

    // Issue a continuous read array command
    error = AT45_SendCommand(pAt45, AT45_CONTINUOUS_READ_LEG, 8, pBuffer, size, address, 0, 0);
    ASSERT(!error, "-F- AT45_Read: Failed to issue command\n\r");

    // Wait for the read command to execute
    while (AT45_IsBusy(pAt45));
}

//------------------------------------------------------------------------------
/// Writes data on the At45 at the specified address. Only one page of
/// data is written that way; if the address is not at the beginning of the
/// page, the data is written starting from this address and wraps around to
/// the beginning of the page.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param pBuffer  Buffer containing the data to write.
/// \param size  Number of bytes to write.
/// \param address  Destination address on the At45.
//------------------------------------------------------------------------------
static void AT45_Write(
    At45 *pAt45,
    unsigned char *pBuffer,
    unsigned int size,
    unsigned int address) 
{
    unsigned char error;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_Write: pAt45 is null.\n\r");
    ASSERT(pBuffer, "-F- AT45_Write: pBuffer is null.\n\r");
    ASSERT(size <= pAt45->pDesc->pageSize, "-F- AT45_Write: Size too big\n\r");

    // Issue a page write through buffer 1 command
    error = AT45_SendCommand(pAt45, AT45_PAGE_WRITE_BUF1, 4, pBuffer, size, address, 0, 0);
    ASSERT(!error, "-F- AT45_Write: Could not issue command.\n\r");

    // Wait until the command is sent
    while (AT45_IsBusy(pAt45));
    
    // Wait until the At45 becomes ready again
    AT45_WaitReady(pAt45);
}

//------------------------------------------------------------------------------
/// Erases a page of data at the given address in the At45.
/// \param pAt45  Pointer to a At45 driver instance.
/// \param address  Address of page to erase.
//------------------------------------------------------------------------------
static void AT45_Erase(At45 *pAt45, unsigned int address) 
{
    unsigned char error;

    // Sanity checks
    ASSERT(pAt45, "-F- AT45_Erase: pAt45 is null\n\r");
    
    // Issue a page erase command.
    error = AT45_SendCommand(pAt45, AT45_PAGE_ERASE, 4, 0, 0, address, 0, 0);
    ASSERT(!error, "-F- AT45_Erase: Could not issue command.\n\r");

    // Wait for end of transfer
    while (AT45_IsBusy(pAt45));

    // Poll until the At45 has completed the erase operation
    AT45_WaitReady(pAt45);
}

//------------------------------------------------------------------------------
//         Exported functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Tests the At45 connected to the board by performing several command
/// on each of its pages.
//------------------------------------------------------------------------------
int main()
{
    unsigned int i;
    unsigned int page;
    unsigned char testFailed;
    const At45Desc *pDesc;

    // Configure pins
    PIO_Configure(pins, PIO_LISTSIZE(pins));

    // Configure traces
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- Basic Dataflash Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
    
    // SPI and At45 driver initialization
    TRACE_INFO("Initializing the SPI and AT45 drivers\n\r");
    AIC_ConfigureIT(BOARD_AT45_A_SPI_ID, 0, ISR_Spi);
    SPID_Configure(&spid, BOARD_AT45_A_SPI_BASE, BOARD_AT45_A_SPI_ID);
    SPID_ConfigureCS(&spid, BOARD_AT45_A_NPCS, AT45_CSR(BOARD_MCK, SPCK));
    AT45_Configure(&at45, &spid, BOARD_AT45_A_NPCS);
    TRACE_INFO("At45 enabled\n\r");
    AIC_EnableIT(BOARD_AT45_A_SPI_ID);
    TRACE_INFO("SPI interrupt enabled\n\r");

    // Identify the At45 device
    TRACE_INFO("Waiting for a dataflash to be connected ...\n\r");
    pDesc = 0;
    while (!pDesc) {

        pDesc = AT45_FindDevice(&at45, AT45_GetStatus(&at45));
    }
    TRACE_INFO("%s detected\n\r", at45.pDesc->name);

    // Output JEDEC identifier of device
    TRACE_INFO("Device identifier: 0x%08X\n\r", AT45_GetJedecId(&at45));

    // Test all pages
    testFailed = 0;
    page = 0;
    while (!testFailed && (page < AT45_PageNumber(&at45))) {

        TRACE_INFO("Test in progress on page: %6u\r", page);
        
        // Erase page
        AT45_Erase(&at45, page * AT45_PageSize(&at45));

        // Verify that page has been erased correctly
        memset(pBuffer, 0, AT45_PageSize(&at45));
        AT45_Read(&at45, pBuffer, AT45_PageSize(&at45), page * AT45_PageSize(&at45));
        for (i=0; i < AT45_PageSize(&at45); i++) {
        
            if (pBuffer[i] != 0xff) {

                TRACE_ERROR("Could not erase page %u\n\r", page);
                testFailed = 1;
                break;
            }
        }

        // Write page
        for (i=0; i < AT45_PageSize(&at45); i++) {
        
            pBuffer[i] = i & 0xFF;
        }
        AT45_Write(&at45, pBuffer, AT45_PageSize(&at45), page * AT45_PageSize(&at45));

        // Check that data has been written correctly
        memset(pBuffer, 0, AT45_PageSize(&at45));
        AT45_Read(&at45, pBuffer, AT45_PageSize(&at45), page * AT45_PageSize(&at45));
        for (i=0; i < AT45_PageSize(&at45); i++) {

            if (pBuffer[i] != (i & 0xFF)) {

                TRACE_ERROR("Could not write page %u\n\r", page);
                testFailed = 1;
                break;
            }
        }

        page++;    
    }

    // Display test result
    if (testFailed) {
    
        TRACE_ERROR("Test failed.\n\r");
    }
    else {
    
         TRACE_INFO("Test passed.\n\r");
    }
    
    return 0;
}

