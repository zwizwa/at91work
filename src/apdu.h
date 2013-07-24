#ifndef _APDU_H_
#define _APDU_H_


#include <stdint.h>



enum iso7816_ins {
    INS_DEACTIVATE_FILE        = 0x04,
    INS_ERASE_BINARY           = 0x0E,
    INS_TERMINAL_PROFILE       = 0x10,
    INS_FETCH                  = 0x12,
    INS_TERMINAL_RESPONSE      = 0x14,
    INS_VERIFY                 = 0x20,
    INS_CHANGE_PIN             = 0x24,
    INS_DISABLE_PIN            = 0x26,
    INS_ENABLE_PIN             = 0x28,
    INS_UNBLOCK_PIN            = 0x2C,
    INS_INCREASE               = 0x32,
    INS_ACTIVATE_FILE          = 0x44,
    INS_MANAGE_CHANNEL         = 0x70,
    INS_MANAGE_SECURE_CHANNEL  = 0x73,
    INS_TRANSACT_DATA          = 0x75,
    INS_EXTERNAL_AUTHOENTICATE = 0x82,
    INS_GET_CHALLENGE          = 0x84,
    INS_INTERNAL_AUTHENTICATE  = 0x88, // also 0x89 ??
    INS_SEARCH_RECORD          = 0xA2,
    INS_SELECT_FILE            = 0xA4,
    INS_TERMINAL_CAPABILITY    = 0xAA,
    INS_READ_BINARY            = 0xB0,
    INS_READ_RECORD            = 0xB2,
    INS_GET_RESPONSE           = 0xC0,   /* Transmission-oriented APDU */
    INS_ENVELOPE               = 0xC2,
    INS_RETRIEVE_DATA          = 0xCB,
    INS_GET_DATA               = 0xCA,
    INS_WRITE_BINARY           = 0xD0,
    INS_WRITE_RECORD           = 0xD2,
    INS_UPDATE_BINARY          = 0xD6,
    INS_PUT_DATA               = 0xDA,
    INS_SET_DATA               = 0xDB,
    INS_UPDATE_DATA            = 0xDC,
    INS_APPEND_RECORD          = 0xE2,
    INS_STATUS                 = 0xF2,
};

struct tpdu {
    uint8_t cla;
    uint8_t ins;
    uint8_t p1;
    uint8_t p2;
    uint8_t p3;
    uint8_t data[];
} __attribute__((__packed__));

struct sw {
    uint8_t sw1;
    uint8_t sw2;
} __attribute__((__packed__));

void apdu_request(struct tpdu *tpdu, struct sw *sw);


#endif
