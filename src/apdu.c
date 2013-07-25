
#include "apdu.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "errno.h"
#include <utility/trace.h>

static uint8_t digit(char c) {
    if (('0' <= c) && (c <= '9')) return c - '0';
    if (('A' <= c) && (c <= 'F')) return 0xA + c - 'A';
    if (('a' <= c) && (c <= 'f')) return 0xA + c - 'a';
    TRACE_DEBUG("Invalid HEX digit %c\n\r", c);
    return 0;
}
void unhex(uint8_t *b, int nb, const char *c) {
    int i;
    for (i=0; i<nb; i++) {
        *b++ = (digit(c[0]) << 8) + digit(c[1]);
        c+=2;
    }
}

int readline(char *buf) {
    int n = 0;
    while(1) {
        uint8_t c = DBGU_GetChar();
        if (c == 10) return n;
        *buf++ = c;
    }
}


#if 1

static void print_apdu(struct tpdu *tpdu, struct sw *sw) {
    uint8_t *msg = (uint8_t*)tpdu;
    printf("APDU:");
    int i, len = sizeof(*tpdu) + tpdu->p3;
    for (i = 0; i < len; i++) printf(" %02X", msg[i]);
    printf(" %02X %02X\n\r", sw->sw1, sw->sw2);
}

/* Handle master APDU requests. */

void apdu_request(struct tpdu *tpdu,  // header and in/out data
                  struct sw *sw) {    // status
    switch(tpdu->ins) {
    case INS_SELECT_FILE:
        TRACE_DEBUG("SELECT_FILE %02x%02x\n\r",
                    tpdu->data[0],
                    tpdu->data[1]);
        sw->sw1 = 0x9F;
        sw->sw2 = 0x1A;
        break;
    case INS_GET_RESPONSE:
        TRACE_DEBUG("GET_RESPONSE %d\n\r", tpdu->p3);
        memset(tpdu->data, 0x55, tpdu->p3);
        sw->sw1 = 0x90;
        sw->sw2 = 0x00;
        break;
    default:
        TRACE_DEBUG("bad APDU\n");
        break;
    }
    print_apdu(tpdu, sw);
}

#else



/* Poor man's delegation: print in HEX on the terminal. */
void apdu_request(struct tpdu *tpdu, struct sw *sw) {

    /* Send request */
    uint8_t *msg = (uint8_t*)tpdu;
    printf("#");
    int data_len = tpdu->ins == INS_GET_RESPONSE ? 0 : tpdu->p3;
    int i, len = sizeof(*tpdu) + data_len;
    for (i = 0; i < len; i++) printf("%02X", msg[i]);
    printf("\n\r");

    /* Read response */
#if 0
    char hex[256];
    int n = readline(hex) / 2;
#else
    char *hex;
    int n;
    switch(tpdu->ins) {
    case INS_SELECT_FILE:
        TRACE_DEBUG("SELECT_FILE %02x%02x\n\r",
                    tpdu->data[0],
                    tpdu->data[1]);
        hex = "9F1A";
        n = 2;
        break;
    case INS_GET_RESPONSE:
        TRACE_DEBUG("GET_RESPONSE %d\n\r", tpdu->p3);
        hex = "555555555555555555555555555555559000";
        n = 18;
        break;
    default:
        TRACE_DEBUG("bad APDU\n");
        break;
    }
#endif
    unhex(tpdu->data, n-2, hex);
    unhex(&sw->sw1,   2,   hex + 2*(n-2));
}

#endif
