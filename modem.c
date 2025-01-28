#include "modem.h"

static const uint8_t *modem_page;
static size_t modem_pagesize;
static uint32_t modem_cycles;
static int modem_tickcnt;
static int modem_p17, modem_dp, modem_p33; // output
static int modem_p14, modem_dpe, modem_ed, modem_p34, modem_p35; // input
static uint16_t modem_rxcur, modem_txcur;
static int modem_txdelay;

static int calc_parity(uint8_t v) {
    v ^= v >> 4;
    v &= 0xF;
    return (0x6996 >> v) & 1;
}

static uint8_t get_port_1() {
    return 0x77 | (modem_p17 << 7) | (modem_dp << 3);
}

static uint8_t get_port_3() {
    return 0xF7 | (modem_p33 << 3);
}

static int modem_txbit() {
    int ret = modem_txcur & 1;
    modem_txcur >>= 1;
    modem_txcur |= (1<<9);
    return ret;
}

static void modem_tick_75() {
}

static void modem_tick_1200() {
    modem_p17 = 0;
    modem_dp = 0;
    modem_p33 = modem_txbit();

    if (modem_pagesize != 0) {
        if (modem_txdelay <= 0) {
            uint8_t byte = modem_page[0];
            modem_page++; modem_pagesize--;
            modem_txcur = 0x200 | (byte << 1) | (calc_parity(byte) << 8);
            modem_txdelay = 9;
        } else {
            modem_txdelay--;
        }
    }
}

static void modem_tick(mcu_8051_t *mcu) {
    uint8_t mcu_p1 = mcu_8051_get_port(mcu, 1);
    uint8_t mcu_p3 = mcu_8051_get_port(mcu, 3);
    modem_p14 = (mcu_p1 >> 4) & 1;
    modem_dpe = (mcu_p1 >> 5) & 1;
    modem_ed = (mcu_p1 >> 6) & 1;
    modem_p34 = (mcu_p3 >> 4) & 1;
    modem_p35 = (mcu_p3 >> 5) & 1;

    modem_tickcnt++;
    while (modem_tickcnt >= 16) {
        modem_tick_75();
        modem_tickcnt -= 16;
    }

    modem_tick_1200();

    mcu_8051_set_port(mcu, 1, get_port_1());
    mcu_8051_set_port(mcu, 3, get_port_3());
}

void modem_update(mcu_8051_t *mcu, uint32_t cycles) {
    modem_cycles += cycles;

    while (modem_cycles >= (11059200/1200)) {
        modem_cycles -= (11059200/1200);
        modem_tick(mcu);
    }
}

void modem_set_page(const uint8_t *page, size_t pagesize) {
    modem_page = page;
    modem_pagesize = pagesize;
}

void modem_init() {
    modem_rxcur = modem_txcur = 0x3FF;
}