#ifndef MODEM_H
#define MODEM_H

#include "mcu.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

typedef enum modem_event_t {
    MODEM_CONNECTED,
    MODEM_DISCONNECTED,
    MODEM_DATA
} modem_event_t;

typedef void (*modem_callback_t)(void *arg, modem_event_t event, uint8_t byte);

typedef struct modem_t {
    struct {
        int p14;
        int dpe;
        int ed;
        int p34;
        int p35;
    } in;

    struct {
        int p17;
        int dp;
        int p33;
    } out;

    int prev_con;
    int tickcnt;
    uint16_t rxcur, txcur;
    int txdelay;
    uint32_t cycles;

    struct {
        uint8_t *data;
        unsigned length, capacity, position;
    } txbuf;

    void *cb_arg;
    modem_callback_t cb;

} modem_t;

void modem_init(modem_t *modem, modem_callback_t cb, void *cb_arg);
void modem_update(modem_t *modem, mcu_8051_t *mcu, uint32_t cycles);
bool modem_tx(modem_t *modem, uint8_t *buffer, unsigned length);

#endif