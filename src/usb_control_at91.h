#ifndef _USB_CONTROL_AT91_H_
#define _USB_CONTROL_AT91_H_

#include <stdint.h>
#include <usb/common/core/USBGenericRequest.h>

enum simtrace_usb_control_request {
    UC_INVALID = 0,
    UC_COMMAND = 1,
    UC_C_APDU  = 2,
    UC_R_APDU  = 3,
};

void usb_control_c_apdu(const uint8_t *buf, int size);
void usb_control_vendor_request(const USBGenericRequest *request);

#endif
