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
/// \dir "Basic-serialflash-project"
///
/// !!!Purpose
///
/// The Basic Serialflash project will help new users get familiar with SPI interface 
/// on Atmel's AT91 family of microcontrollers. This project gives you an AT26 
/// serial firmware dataflash programming code so that can help develop your own 
/// SPI devices applications with maximum efficiency.
///
/// You can find following information depends on your needs:
/// - A Spi low level driver performs SPI device Initializes, data transfer and 
/// receive. It can be used by upper SPI driver such as AT26 %dataflash.
/// - A Dataflash driver is based on top of the corresponding Spi driver.
/// It allow user to do operations with %dataflash in a unified way.
///
/// !See also
///    - "spi-flash": Dataflash interface driver.
///
/// !!!Requirements
///
/// This package can be used with all Atmel evaluation kits that have SPI
/// interface and on-board or external Dataflash connected. The package runs at 
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
/// -# Upon startup, the application will output the following lines on the DBGU: 
///    \code
///     -- Basic Serial Firmware Dataflash Project xxx --
///     -- AT91xxxxxx-xx
///     -- Compiled: xxx xx xxxx xx:xx:xx --
///     -I- SPI and At26 initialized 
///    \endcode
/// -# The program will connect to the serial firmware dataflash through the SPI 
///    and start sending commands to it. It will perform the following: 
///    - Read the JEDEC identifier of the device to autodetect it 
///      The next line should indicate if the serial dataflash has been 
///      correctly identified. For example, this is what appears when an 
///      AT26DF321 chip is recognized: 
///    \code
///    -I- AT26DF321 Serial Flash detected 
///    \endcode
///    - Erase the chip 
///    - Check that each page is blank 
///    - Write a "Walking one" pattern on each page: 
///    \code
///    Byte 0 = 00000001
///    Byte 1 = 00000010
///    Byte 2 = 00000100
///    ........ 
///    \endcode
///    - Verify that the pattern has been correctly applied on each page 
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
/// \unit
///
/// !Purpose
///
/// This file contains all the specific code for the basic-serialflash-project.
/// It tests the serial firmware dataflash present on the evaluation kit by 
/// erasing and writing each one of its pages.
/// 
/// !Contents
/// The code can be roughly broken down as follows:
///    - AT26 Dataflash write data function.
///    - AT26 Dataflash read data function.
///    - AT26 Dataflash erase function.
///    - Other AT26 functions (such as AT26_GetStatus())
///    - The main() function, which implements the program behavior.
///       - Initializes an AT26 instance and configures SPI chip select pin.
///       - Config SPI Interrupt Service Routine.
///       - Identifier the AT26 device connected to the evaluation kit.
///       - Test the dataflash by erasing and writing each one of its pages.
/// 
/// !See also
///    - "spi-flash": SPI Dataflash interface driver.
///
/// Please refer to the list of functions in the #Overview# tab of this unit
/// for more detailed information.
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#include <board.h>
#include <dbgu/dbgu.h>
#include <pio/pio.h>
#include <aic/aic.h>
#include <utility/assert.h>
#include <utility/trace.h>
#include <utility/math.h>
#include <memories/spi-flash/at26.h>

#include <string.h>

//------------------------------------------------------------------------------
//         Internal definitions
//------------------------------------------------------------------------------

/// Maximum device page size in bytes.
#define MAXPAGESIZE     256

#if defined(BOARD_AT45_A_SPI_BASE)

/// Address of the SPI peripheral connected to the AT26.
#define SPI_BASE        BOARD_AT45_A_SPI_BASE
/// Peripheral identifier of the SPI connected to the AT26.
#define SPI_ID          BOARD_AT45_A_SPI_ID
/// Chip select value used to select the AT26 chip.
#define SPI_CS          BOARD_AT45_A_NPCS
/// SPI peripheral pins to configure to access the serial flash.
#define SPI_PINS        BOARD_AT45_A_SPI_PINS, BOARD_AT45_A_NPCS_PIN

#elif defined(AT91C_BASE_SPI0)

/// Address of the SPI peripheral connected to the AT26.
#define SPI_BASE        AT91C_BASE_SPI0
/// Peripheral identifier of the SPI connected to the AT26.
#define SPI_ID          AT91C_ID_SPI0
/// Chip select value used to select the AT26 chip.
#define SPI_CS          0
/// SPI peripheral pins to configure to access the serial flash.
#define SPI_PINS        PINS_SPI0, PIN_SPI0_NPCS0

#else

/// Address of the SPI peripheral connected to the AT26.
#define SPI_BASE        AT91C_BASE_SPI
/// Peripheral identifier of the SPI connected to the AT26.
#define SPI_ID          AT91C_ID_SPI
/// Chip select value used to select the AT26 chip.
#define SPI_CS          0
/// SPI peripheral pins to configure to access the serial flash.
#define SPI_PINS        PINS_SPI, PIN_SPI_NPCS0

#endif //#if defined(AT91C_BASE_SPI0)

//------------------------------------------------------------------------------
//         Internal variables
//------------------------------------------------------------------------------

/// SPI driver instance.
static Spid spid;

/// Serial flash driver instance.
static At26 at26;

#define PIN_SPI_nWP0	{1 << 15, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}

/// Pins to configure for the application.
static Pin pins[] = {SPI_PINS, PIN_SPI_nWP0};

//------------------------------------------------------------------------------
//         Internal functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Interrupt service routine for the SPI peripheral. Forwards the interrupt
/// to the SPI driver.
//------------------------------------------------------------------------------
static void ISR_Spi(void)
{
    SPID_Handler(&spid);
}

//------------------------------------------------------------------------------
/// Reads and returns the status register of the serial flash.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_ReadStatus(At26 *pAt26)
{
    unsigned char error, status;

    SANITY_CHECK(pAt26);

    // Issue a status read command
    error = AT26_SendCommand(pAt26, AT26_READ_STATUS, 1, &status, 1, 0, 0, 0);
    ASSERT(!error, "-F- AT26_GetStatus: Failed to issue command.\n\r");

    // Wait for transfer to finish
    while (AT26_IsBusy(pAt26));

    return status;
}

//------------------------------------------------------------------------------
/// Writes the given value in the status register of the serial flash device.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param status  Status to write.
//------------------------------------------------------------------------------
static void AT26_WriteStatus(At26 *pAt26, unsigned char status)
{
    unsigned char error;

    SANITY_CHECK(pAt26);

    // Issue a write status command
    error = AT26_SendCommand(pAt26, AT26_WRITE_STATUS, 1, &status, 1, 0, 0, 0);
    ASSERT(!error, "-F- AT26_WriteStatus: Failed to issue command.\n\r");
    while (AT26_IsBusy(pAt26));
}

//------------------------------------------------------------------------------
/// Waits for the serial flash device to become ready to accept new commands.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static void AT26_WaitReady(At26 *pAt26)
{
    unsigned char ready = 0;

    SANITY_CHECK(pAt26);

    // Read status register and check busy bit
    while (!ready) {

        ready = ((AT26_ReadStatus(pAt26) & AT26_STATUS_RDYBSY) == AT26_STATUS_RDYBSY_READY);
    }
}

//------------------------------------------------------------------------------
/// Reads and returns the serial flash device ID.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned int AT26_ReadJedecId(At26 *pAt26)
{
    unsigned char error;
    unsigned int id = 0;

    SANITY_CHECK(pAt26);
 
    // Issue a read ID command
    error = AT26_SendCommand(pAt26, AT26_READ_JEDEC_ID, 1,
                             (unsigned char *) &id, 3, 0, 0, 0);
    ASSERT(!error, "-F- AT26_GetJedecId: Could not issue command.\n\r");

     // Wait for transfer to finish
    while (AT26_IsBusy(pAt26));

    return id;
}

//------------------------------------------------------------------------------
/// Enables critical writes operation on a serial flash device, such as sector
/// protection, status register, etc.
/// \para pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static void AT26_EnableWrite(At26 *pAt26)
{
    unsigned char error;

    SANITY_CHECK(pAt26);

    // Issue a write enable command
    error = AT26_SendCommand(pAt26, AT26_WRITE_ENABLE, 1, 0, 0, 0, 0, 0);
    ASSERT(!error, "-F- AT26_EnableWrite: Could not issue command.\n\r");

    // Wait for end of transfer
    while (AT26_IsBusy(pAt26));
}

//------------------------------------------------------------------------------
/// Unprotects the contents of the serial flash device.
/// Returns 0 if the device has been unprotected; otherwise returns
/// SF_PROTECTED.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_Unprotect(At26 *pAt26)
{
    unsigned char status;

    SANITY_CHECK(pAt26);

    // Get the status register value to check the current protection
    status = AT26_ReadStatus(pAt26);
    if ((status & AT26_STATUS_SWP) == AT26_STATUS_SWP_PROTNONE) {

        // Protection already disabled
        return 0;
    }
    
    // Check if sector protection registers are locked
    if ((status & AT26_STATUS_SPRL) == AT26_STATUS_SPRL_LOCKED) {

        // Unprotect sector protection registers by writing the status reg.
        AT26_EnableWrite(pAt26);
        AT26_WriteStatus(pAt26, 0);
    }
    
    // Perform a global unprotect command
      AT26_EnableWrite(pAt26);
    AT26_WriteStatus(pAt26, 0);
    
    // Check the new status
    if ((status & (AT26_STATUS_SPRL | AT26_STATUS_SWP)) != 0) {

        return AT26_ERROR_PROTECTED;
    }
    else {

        return 0;
    }
}

//------------------------------------------------------------------------------
/// Erases all the content of the memory chip.
/// \param pAt26  Pointer to an AT26 driver instance.
//------------------------------------------------------------------------------
static unsigned char AT26_EraseChip(At26 *pAt26)
{
    unsigned char status;
    unsigned char error;

    SANITY_CHECK(pAt26);

    // Check that the flash is ready an unprotected
    status = AT26_ReadStatus(pAt26);
    if ((status & AT26_STATUS_SWP) != AT26_STATUS_SWP_PROTNONE) {

        TRACE_WARNING("AT26_EraseBlock: Device is protected.\n\r");
        return AT26_ERROR_PROTECTED;
    }
    
    // Enable critical write operation
      AT26_EnableWrite(pAt26);
    
    // Erase the chip
    error = AT26_SendCommand(pAt26, AT26_CHIP_ERASE_2, 1, 0, 0, 0, 0, 0);
    ASSERT(!error, "-F- AT26_ChipErase: Could not issue command.\n\r");
     while (AT26_IsBusy(pAt26));    
    AT26_WaitReady(pAt26);

    return 0;
}

//------------------------------------------------------------------------------
/// Erases the specified 4KB block of the serial firmware dataflash.
/// Returns 0 if successful; otherwise returns AT26_ERROR_PROTECTED if the
/// device is protected or AT26_ERROR_BUSY if it is busy executing a command.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param address  Address of the block to erase.
//------------------------------------------------------------------------------
static unsigned char AT26_EraseBlock(At26 *pAt26, unsigned int address)
{
    unsigned char status;
    unsigned char error;

    SANITY_CHECK(pAt26);
 
    // Check that the flash is ready an unprotected
    status = AT26_ReadStatus(pAt26);
    if ((status & AT26_STATUS_RDYBSY) != AT26_STATUS_RDYBSY_BUSY) {

        TRACE_WARNING("AT26_EraseBlock: Device is not ready.\n\r");
        return AT26_ERROR_BUSY;
    }
    else if ((status & AT26_STATUS_SWP) != AT26_STATUS_SWP_PROTNONE) {

        TRACE_WARNING("AT26_EraseBlock: Device is protected.\n\r");
        return AT26_ERROR_PROTECTED;
    }

    // Enable critical write operation
      AT26_EnableWrite(pAt26);

    // Start the block erase command
    error = AT26_SendCommand(pAt26, AT26_BLOCK_ERASE_4K, 4, 0, 0, address, 0, 0);
    ASSERT(!error, "-F- AT26_EraseBlock: Could not issue command.\n\r");
    while (AT26_IsBusy(pAt26));
    AT26_WaitReady(pAt26);

    return 0;
}

//------------------------------------------------------------------------------
/// Writes data at the specified address on the serial firmware dataflash. The
/// page(s) to program must have been erased prior to writing. This function
/// handles page boundary crossing automatically.
/// Returns 0 if successful; otherwise, returns AT26_ERROR_PROGRAM is there has
/// been an error during the data programming.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param pData  Data buffer.
/// \param size  Number of bytes in buffer.
/// \param address  Write address.
//------------------------------------------------------------------------------
static unsigned char AT26_Write(
    At26 *pAt26,
    unsigned char *pData,
    unsigned int size,
    unsigned int address)
{
    unsigned int pageSize;
    unsigned int writeSize;
    unsigned char error;
    unsigned char status;

    SANITY_CHECK(pAt26);
    SANITY_CHECK(pData);

    // Retrieve device page size
    pageSize = AT26_PageSize(&at26);

    // Program one page after the other
    while (size > 0) {

        // Compute number of bytes to program in page
        writeSize = min(size, pageSize - (address % pageSize));
                
        // Enable critical write operation
        AT26_EnableWrite(pAt26);
     
         // Program page
          error = AT26_SendCommand(pAt26, AT26_BYTE_PAGE_PROGRAM, 4,
                           pData, writeSize, address, 0, 0);
        ASSERT(!error, "-F- AT26_WritePage: Failed to issue command.\n\r");
        while (AT26_IsBusy(pAt26));
        AT26_WaitReady(pAt26);

        // Make sure that write was without error
        status = AT26_ReadStatus(pAt26);
        if ((status & AT26_STATUS_EPE) == AT26_STATUS_EPE_ERROR) {

            return AT26_ERROR_PROGRAM;
        }
        
        size -= writeSize;
        address += writeSize;
    }

    return 0;
}

//------------------------------------------------------------------------------
/// Reads data from the specified address on the serial flash.
/// \param pAt26  Pointer to an AT26 driver instance.
/// \param pData  Data buffer.
/// \param size  Number of bytes to read.
/// \param address  Read address.
//------------------------------------------------------------------------------
static void AT26_Read(
    At26 *pAt26,
    unsigned char *pData,
    unsigned int size,
    unsigned int address)
{
    unsigned char error;
    
     // Start a read operation
      error = AT26_SendCommand(pAt26, AT26_READ_ARRAY_LF, 4, pData, size, address, 0, 0);
    ASSERT(!error, "-F- AT26_Read: Could not issue command.\n\r");
    while (AT26_IsBusy(pAt26));
}

//------------------------------------------------------------------------------
//         External functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// Initializes the serial flash and performs several tests on it.
//------------------------------------------------------------------------------
int main(void)
{
    unsigned int jedecId;
    unsigned int numPages;
    unsigned int pageSize;
    unsigned int i, j;
    unsigned int address;
    unsigned char pBuffer[MAXPAGESIZE];

    // Configure the DBGU
    TRACE_CONFIGURE(DBGU_STANDARD, 115200, BOARD_MCK);
    printf("-- Basic Serial Firmware Dataflash Project %s --\n\r", SOFTPACK_VERSION);
    printf("-- %s\n\r", BOARD_NAME);
    printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

    // Initialize the SPI and serial flash
    PIO_Configure(pins, PIO_LISTSIZE(pins));
    AIC_ConfigureIT(SPI_ID, 0, ISR_Spi);
    SPID_Configure(&spid, SPI_BASE, SPI_ID);
    AT26_Configure(&at26, &spid, SPI_CS);
    AIC_EnableIT(SPI_ID);
    TRACE_INFO("SPI and AT26 drivers initialized\n\r");
    
    // Read the JEDEC ID of the device to identify it
    jedecId = AT26_ReadJedecId(&at26);
    if (AT26_FindDevice(&at26, jedecId)) {

        TRACE_INFO("%s serial flash detected\n\r", AT26_Name(&at26));
    }
    else {

        TRACE_ERROR("Failed to recognize the device (JEDEC ID is 0x%08X).\n\r", jedecId);
        return 1;
    }
    ASSERT(MAXPAGESIZE >= AT26_PageSize(&at26), "-F- MAXPAGESIZE too small\n\r");

    // Get device parameters
    numPages = AT26_PageNumber(&at26);
    pageSize = AT26_PageSize(&at26);

    // Unprotected the flash
    AT26_Unprotect(&at26);
    TRACE_INFO("Flash unprotected\n\r");

    // Erase the chip
    TRACE_INFO("Chip is being erased...\n\r");
    AT26_EraseChip(&at26);
    TRACE_INFO("Checking erase ...\n\r");

    // Check that the chip has been erased correctly
    address = 0;
    for (i=0; i < numPages; i++) {

        TRACE_INFO("Checking page #%u\r", i);
        AT26_Read(&at26, pBuffer, pageSize, address);
        for (j=0; j < pageSize; j++) {

            if (pBuffer[j] != 0xFF) {

                TRACE_ERROR("Failed erase on page%u:byte%u\n\r", i, j);
                TRACE_ERROR(
                          "-E- Expected 0xFF, read 0x%02X\n\r",
                          pBuffer[j]);
                return 2;
            }
        }

        address += pageSize;
    }
    TRACE_INFO("Erase successful.\n\r");

    // Program a "walking one" pattern on each page
    TRACE_INFO("Programming a walking 1 on all pages ...\n\r");
    address = 0;
    for (i=0; i < numPages; i++) {

        TRACE_INFO("Programming page #%u\r", i);

        // Fill buffer
        for (j=0; j < pageSize; j++) {

            pBuffer[j] = 1 << (j & 0x7);
        }

        // Write buffer
        AT26_Write(&at26, pBuffer, pageSize, address);

        // Read page back and check result
        memset(pBuffer, 0, pageSize);
        AT26_Read(&at26, pBuffer, pageSize, address);

        for (j=0; j < pageSize; j++) {

            if (pBuffer[j] != (1 << (j & 0x7))) {

                TRACE_ERROR("Failed program on page%u:byte%u\n\r", i, j);
                TRACE_ERROR(
                          "-E- Expected 0x%02X, read 0x%02X\n\r",
                          1 << (j & 0x7),
                          pBuffer[j]);
                return 3;
            }
        }

        address += pageSize;
    }

    TRACE_INFO("Walking 1 test successful.\n\r");

    return 0;
}

