
#include "apdu.h"
#include <stdint.h>
#include <string.h>
#include "errno.h"
#include <utility/trace.h>


static void print_apdu(struct tpdu *tpdu, struct sw *sw) {
    uint8_t *msg = (uint8_t*)tpdu;
    printf("APDU:");
    int i, len = sizeof(*tpdu) + tpdu->p3;
    for (i = 0; i < len; i++) printf(" %02X", msg[i]);
    printf(" %02X %02X\n\r", sw->sw1, sw->sw2);
}
void apdu_request(struct tpdu *tpdu, struct sw *sw) {
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
