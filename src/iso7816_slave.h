#ifndef _ISO7816_SLAVE_H_
#define _ISO7816_SLAVE_H_

#include <stdint.h>

/* iso7816_slave implements card-side of the ISO7816 T=0 protocol.
   This object sits inbetween:
   - platform-specific serial port from iso7816_port.h  (currently hard-coded)
   - an abstract C-APDU send / R-APDU receive packet transport mechanism

   The object is implemented as a non-blocking state machine to avoid
   dependence on thread system, e.g. to allow bare metal
   implementation.
*/

struct iso7816_slave;

// Callback to initiate C-APDU packet transfer.
typedef void (*iso7816_slave_c_apdu_t)(void *ctx, int size);
// Obtain next C-APDU byte.  rv >= 0 is byte, other is EOF.
int iso7816_slave_c_apdu_getc(struct iso7816_slave *s);


// Send R-APDU to serial port.
int iso7816_slave_r_apdu_write(struct iso7816_slave *s, const uint8_t *buf, int size);

struct iso7816_slave *iso7816_slave_init(iso7816_slave_c_apdu_t c_apdu_cb, void *c_apdu_ctx);

// State machine transition routine.  This should be called in the
// application's event loop.
void iso7816_slave_tick(struct iso7816_slave *s);

// Send a-synchronous command to state machine.
enum iso7816_slave_command_tag {
    CMD_SET_ATR = 0,
    CMD_SET_SKIP = 1,
    CMD_HALT = 2,
};
struct iso7816_slave_command {
    uint32_t command;
    union {
        uint32_t u32;
        uint8_t buf[0];
    } data;
} __attribute__((__packed__));
int iso7816_slave_command(struct iso7816_slave *s, const uint8_t *buf, int size);


#endif
