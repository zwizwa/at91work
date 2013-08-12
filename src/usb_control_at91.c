#include <board.h>
#include <utility/trace.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <string.h>
#include <stdint.h>
#include "iso7816_slave.h"

/* SIMtrace PHONE interface uses VENDOR control requests on EP0 since
   on AT91SAM7 we don't have enough endpoints for dedicated IN/OUT
   EPs.

   patched cciddriver.c to pass on VENDOR control requests:
   CCID_RequestHandler() -> usb_control_vendor_request()

   FIXME: should this use INTERFACE requests together with a second
   interface to allow this to work on Windows? */

static const uint8_t idle[] = {0xFF, 0, 0, 0, 0};

static uint8_t from_host_buf[512];
static int from_host_size;

const uint8_t *msg_buf = NULL;
int msg_size = 0;

extern struct iso7816_slave *iso7816_slave;  // main.c

static void read_cb(void *arg,
                    unsigned char status,
                    unsigned int transferred,
                    unsigned int remaining) {

    if (status != USBD_STATUS_SUCCESS) {
        TRACE_ERROR( "UsbDataReceived: Transfer error\n\r");
        return;
    }

    TRACE_DEBUG("CONTROL OUT %d %d\n\r", transferred, remaining);

    /* FIXME: Expected structure is R-APDU or SIMtrace slave mode
       command encapsulated in SW=FFFF */
    if (from_host_size < 2) {
        TRACE_ERROR("Short packet, size %d\n\r", from_host_size);
    }
    else {
        int cmd_size = from_host_size - 2;
        uint8_t *sw = &from_host_buf[cmd_size];
        uint16_t sw16 = (sw[0]<<8) + sw[1]; // BE
        TRACE_DEBUG("%04X\n\r", sw16);
        switch(sw16) {
        case 0xFFFF:
            /* Unused R-APDU status word: we use this as protocol escape.
               FIXME: Might want to do this better */
            iso7816_slave_command(iso7816_slave, from_host_buf, cmd_size);
            break;
        default:
            iso7816_slave_r_apdu_write(iso7816_slave, from_host_buf, from_host_size);
            break;
        }
    }

    USBD_Write(0,0,0,0,0); // STATUS
}

static void write_cb(void *arg,
                     unsigned char status,
                     unsigned int transferred,
                     unsigned int remaining) {
    USBD_Read(0,0,0,0,0); // STATUS
}

/* Control  in  = SETUP IN [IN ...] STATUS(=OUT)
   Countrol out = SETUP OUT [OUT ...] STATUS(=IN) */

void usb_control_vendor_request(const USBGenericRequest *request) {
    TRACE_DEBUG("vendor request %02X %d %d %d %d\n\r",
                request->bmRequestType,
                request->bRequest,
                request->wValue,
                request->wIndex,
                request->wLength);
    switch (USBGenericRequest_GetDirection(request)) {

    case USBGenericRequest_OUT: // e.g 0x40
        /* Read payload. */
        from_host_size =  request->wLength;
        if (request->wLength > sizeof(from_host_buf)) {
            TRACE_ERROR("invalid length: %d\n\r", request->wLength);
            from_host_size = sizeof(from_host_buf);
        }
        USBD_Read(0, &from_host_buf, from_host_size, read_cb, 0);
        break;

    case USBGenericRequest_IN:  // e.g. 0xC0
        TRACE_DEBUG("CONTROL IN %d\n\r", request->wLength);
        if (msg_buf) {
            USBD_Write(0, msg_buf, msg_size, write_cb, 0);
            msg_buf = NULL;
            msg_size = 0;
        }
        else {
            USBD_Write(0, &idle, sizeof(idle), write_cb, 0);
        }
        break;
    }
}

void usb_control_c_apdu(const uint8_t *buf, int size) {
    // Set size first; buf != NULL is the trigger condition.
    msg_size = size;
    msg_buf  = buf;
}

