#include "modem.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#define CHUNK_SIZE 1024

static int calc_parity(uint8_t v) {
    v ^= v >> 4;
    v &= 0xF;
    return (0x6996 >> v) & 1;
}

static uint8_t get_port_1(modem_t *modem) {
    return 0x77 | (modem->out.p17 << 7) | (modem->out.dp << 3);
}

static uint8_t get_port_3(modem_t *modem) {
    return 0xF7 | (modem->out.p33 << 3);
}

static void modem_txbit(modem_t *modem) {
    modem->out.p33 = modem->txcur & 1;
    modem->txcur >>= 1;
    modem->txcur |= (1<<9);
}

bool modem_tx(modem_t *modem, uint8_t *buffer, unsigned length) {
    // prevents capacity == 0 crashes
    if (length == 0)
        return true;

    if (modem->txbuf.length + length > modem->txbuf.capacity) {
        size_t newcap = ((modem->txbuf.length + length + CHUNK_SIZE - 1) / CHUNK_SIZE) * CHUNK_SIZE;
        assert(newcap >= modem->txbuf.capacity);
        assert(newcap >= modem->txbuf.length + length);

        uint8_t *tmp = (uint8_t *)realloc(modem->txbuf.data, newcap);
        if (tmp == NULL)
            return false;

        modem->txbuf.data = tmp;
        
        if ((modem->txbuf.position + modem->txbuf.length) > modem->txbuf.capacity) {
            // Wrapping - need to memmove
            size_t newpos = modem->txbuf.position + newcap - modem->txbuf.capacity;
            memmove((char *)modem->txbuf.data + newpos, (char *)modem->txbuf.data + modem->txbuf.position, modem->txbuf.capacity - modem->txbuf.position);
            modem->txbuf.position = newpos;
        }

        modem->txbuf.capacity = newcap;
    }

    size_t writepos = (modem->txbuf.position + modem->txbuf.length) % modem->txbuf.capacity;
    size_t remaining = modem->txbuf.capacity - writepos;

    if (length > remaining) {
        memcpy((char *)modem->txbuf.data + writepos, buffer, remaining);
        memcpy(modem->txbuf.data, (const char *)buffer + remaining, length - remaining);

    } else {
        memcpy((char *)modem->txbuf.data + writepos, buffer, length);
    }

    modem->txbuf.length += length;
    return true;
}

static uint8_t modem_rxbit(modem_t *modem) {
    modem->rxcur = (modem->rxcur >> 1) | (modem->in.ed << 9);

    if ((modem->rxcur & 0x201) == 0x200) {
        uint8_t byte = (modem->rxcur >> 1) & 0x7F;
        if (((modem->rxcur >> 8) & 1) ^ calc_parity(byte)) {
            byte = 0x7F; // Parity error
        }

        modem->rxcur = 0x3FF;
        return byte;

    } else {
        return 0xFF; // No byte
    }
}

static void modem_setbyte(modem_t *modem, uint8_t byte) {
    modem->txcur = 0x200 | (byte << 1) | (calc_parity(byte) << 8);
    modem->txdelay = 9;
}

static void modem_flushbuf(modem_t *modem) {
    if (modem->txdelay > 0) {
        modem->txdelay--;
        return;
    }

    if (modem->txbuf.length) {
        uint8_t byte = modem->txbuf.data[modem->txbuf.position];
        modem->txbuf.position = (modem->txbuf.position + 1) % modem->txbuf.capacity;
        modem->txbuf.length--;

        modem_setbyte(modem, byte);
    }
}

static void modem_tick_75(modem_t *modem) {
    if (!modem->in.p14) {
        if (modem->in.p35) {
            modem_txbit(modem);

        } else {
            uint8_t byte = modem_rxbit(modem);
            if (byte != 0xFF) {
                modem->cb(modem->cb_arg, MODEM_DATA, byte);
            }
        }
    }
}

static void modem_tick_1200(modem_t *modem) {
    if (modem->prev_con != modem->in.p14) {
        modem->txcur = 0x3FF;
        modem->rxcur = 0x3FF;
        modem->txdelay = 2400;

        modem->prev_con = modem->in.p14;

        if (modem->in.p14) {
            modem->cb(modem->cb_arg, MODEM_DISCONNECTED, 0);
        } else {
            modem->cb(modem->cb_arg, MODEM_CONNECTED, 0);
        }
    }
    
    if (modem->in.p14) {
        // Pas connexion
        modem->out.p17 = 0;
        modem->out.dp = 0;

        uint8_t byte = modem_rxbit(modem);
        if (byte != 0xFF) {
            modem_setbyte(modem, byte);
        }

        modem_txbit(modem);

    } else {
        // Connexion
        modem->out.p17 = 1;
        modem->out.dp = 0;

        if (modem->in.p35) {
            // Inversé
            uint8_t byte = modem_rxbit(modem);
            if (byte != 0xFF) {
                modem->cb(modem->cb_arg, MODEM_DATA, byte);
            }

        } else {
            modem_flushbuf(modem);
            modem_txbit(modem);
        }
    }
}

static void modem_tick(modem_t *modem, mcu_8051_t *mcu) {
    uint8_t mcu_p1 = mcu_8051_get_port(mcu, 1);
    uint8_t mcu_p3 = mcu_8051_get_port(mcu, 3);
    modem->in.p14 = (mcu_p1 >> 4) & 1;
    modem->in.dpe = (mcu_p1 >> 5) & 1;
    modem->in.ed = (mcu_p1 >> 6) & 1;
    modem->in.p34 = (mcu_p3 >> 4) & 1;
    modem->in.p35 = (mcu_p3 >> 5) & 1;

    modem->tickcnt++;
    while (modem->tickcnt >= 16) {
        modem_tick_75(modem);
        modem->tickcnt -= 16;
    }

    modem_tick_1200(modem);

    mcu_8051_set_port(mcu, 1, get_port_1(modem));
    mcu_8051_set_port(mcu, 3, get_port_3(modem));
}

void modem_update(modem_t *modem, mcu_8051_t *mcu, uint32_t cycles) {
    modem->cycles += cycles;

    while (modem->cycles >= (11059200/1200)) {
        modem->cycles -= (11059200/1200);
        modem_tick(modem, mcu);
    }
}

void modem_init(modem_t *modem, modem_callback_t cb, void *cb_arg) {
    modem->rxcur = modem->txcur = 0x3FF;
    modem->txdelay = 0;
    modem->cycles = 0;
    modem->tickcnt = 0;
    modem->prev_con = 1;

    modem->out.dp = 0;
    modem->out.p17 = 0;
    modem->out.p33 = 1;

    modem->txbuf.capacity = modem->txbuf.length = modem->txbuf.position = 0;
    modem->txbuf.data = NULL;

    modem->cb = cb;
    modem->cb_arg = cb_arg;
}