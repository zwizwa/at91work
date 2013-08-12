#include <board.h>
#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <string.h>
#include <stdint.h>

/* SIMtrace PHONE interface uses VENDOR requests.
   Patched in CDCDSerialDriver.c : USBDCallbacks_RequestReceived.
   FIXME: once CCID driver is done, patch in the same way.
   FIXME: should this use INTERFACE requests and define a second interface?
*/

static uint32_t command[32];

static void vr_read_cb(void *arg,
                       unsigned char status,
                       unsigned int transferred,
                       unsigned int remaining) {
    TRACE_WARNING("CONTROL OUT %d\n\r", transferred);
    USBD_Write(0,0,0,0,0); // STATUS
}

static void vr_write_cb(void *arg,
                        unsigned char status,
                        unsigned int transferred,
                        unsigned int remaining) {
    USBD_Read(0,0,0,0,0); // STATUS
}

/* Control  in  = SETUP IN [IN ...] STATUS(=OUT)
   Countrol out = SETUP OUT [OUT ...] STATUS(=IN) */

void SIMtrace_Vendor_RequestHandler(const USBGenericRequest *request) {
    TRACE_DEBUG("Vendor_RequestHandler %02X %d %d %d %d\n\r",
                request->bmRequestType,
                request->bRequest,
                request->wValue,
                request->wIndex,
                request->wLength);
    switch (USBGenericRequest_GetDirection(request)) {
    case USBGenericRequest_OUT: // e.g 0x40
        /* Read payload. */
        USBD_Read(0, &command, sizeof(command), vr_read_cb, 0);
        break;
    case USBGenericRequest_IN:  // e.g. 0xC0
        TRACE_WARNING("CONTROL IN %d\n\r", request->wLength);
        USBD_Write(0, &command, sizeof(command), vr_write_cb, 0);
        break;
    }
}
