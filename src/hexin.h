
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <utility/trace.h>

// Hex input parser.
struct hexin {
    uint8_t *buf;
    int digit_index;
};
static inline void hexin_reset(struct hexin *r) {
    r->digit_index = 0;
}
static inline int hexin_length(struct hexin *r) {
    return r->digit_index/2;
}
static int hexin_digit(char c) {
    if (('0' <= c) && (c <= '9')) return c - '0';
    if (('A' <= c) && (c <= 'F')) return 0xA + c - 'A';
    if (('a' <= c) && (c <= 'f')) return 0xA + c - 'a';
    TRACE_DEBUG("Invalid HEX digit %c\n\r", c);
    return -1;
}
static inline void hexin_print(struct hexin *r) {
    int i;
    printf("#");
    for (i = 0 ; i<hexin_length(r); i++) {
        printf("%02X", r->buf[i]);
    }
    printf("\n\r");
}
static inline int hexin_push(struct hexin *r, uint8_t c) {
    // FIXME: check overflow
    if ((c == '\r') || (c == '\n')) {
        hexin_print(r);
        return hexin_length(r);
    }
    int d0 = hexin_digit(c);
    if (d0 >= 0) {
        uint8_t d1 = r->buf[r->digit_index/2];
        r->buf[r->digit_index/2] = (d1 << 4) | (d0 & 0x0F);
        r->digit_index++;
    }
    else {
        hexin_reset(r);
    }
    return -1;
}
