#ifndef _USB_DFU_DESC_H
#define _USB_DFU_DESC_H

#include <usb/common/core/USBGenericDescriptor.h>

#define CONFIG_DFU_NUM_APP_IF	1
#define CONFIG_DFU_NUM_APP_STR	4

struct USBStringDescriptor {
	USBGenericDescriptor hdr;
	unsigned short wData[];
} __attribute__((packed));


#ifdef BOARD_USB_DFU

#define DFU_NUM_IF	3
#define DFU_IF_DESCRIPTORS			{ 				\
	{									\
		.bLength 		= sizeof(USBInterfaceDescriptor),	\
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,	\
		.bInterfaceNumber	= CONFIG_DFU_NUM_APP_IF,		\
		.bAlternateSetting	= 0,					\
		.bNumEndpoints		= 0,					\
		.bInterfaceClass	= 0xFE,					\
		.bInterfaceSubClass	= 0x01,					\
		.bInterfaceProtocol	= 0x01,					\
		.iInterface		= CONFIG_DFU_NUM_APP_STR,		\
	},									\
	{									\
		.bLength 		= sizeof(USBInterfaceDescriptor),	\
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,	\
		.bInterfaceNumber	= CONFIG_DFU_NUM_APP_IF+1,		\
		.bAlternateSetting	= 0,					\
		.bNumEndpoints		= 0,					\
		.bInterfaceClass	= 0xFE,					\
		.bInterfaceSubClass	= 0x01,					\
		.bInterfaceProtocol	= 0x01,					\
		.iInterface		= CONFIG_DFU_NUM_APP_STR+1,		\
	},									\
	{									\
		.bLength 		= sizeof(USBInterfaceDescriptor),	\
		.bDescriptorType	= USBGenericDescriptor_INTERFACE,	\
		.bInterfaceNumber	= CONFIG_DFU_NUM_APP_IF+2,		\
		.bAlternateSetting	= 0,					\
		.bNumEndpoints		= 0,					\
		.bInterfaceClass	= 0xFE,					\
		.bInterfaceSubClass	= 0x01,					\
		.bInterfaceProtocol	= 0x01,					\
		.iInterface		= CONFIG_DFU_NUM_APP_STR+2,		\
	},									\
}

extern const struct USBStringDescriptor USBDFU_string1;
extern const struct USBStringDescriptor USBDFU_string2;
extern const struct USBStringDescriptor USBDFU_string3;

#define DFU_NUM_STRINGS	3
#define DFU_STRING_DESCRIPTORS	\
	&USBDFU_string1,	\
	&USBDFU_string2,	\
	&USBDFU_string3,

#else /* BOARD_USB_DFU */

/* no DFU bootloader is being used */
#define DFU_NUM_IF	0
#define DFU_IF_DESCRIPTORS

#define DFU_NUM_STRINGS	0
#define DFU_STRING_DESCRIPTORS

#endif /* BOARD_USB_DFU */

#endif
