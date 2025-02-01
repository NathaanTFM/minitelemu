#define _GNU_SOURCE
#include <time.h>
#include <stdlib.h>
#include <string.h>

#include "minitel.h"
#include "mcu.h"
#include "graphics.h"
#include "modem.h"

#include <unistd.h>

struct minitel_t {
    mcu_8051_t *mcu;
    ef9345_state_t *gfx;
    modem_t modem;

    uint8_t vram[0x2000];
    uint8_t keydown[8];
};

static const uint8_t KB_MAP_1_REV[] = {0, 3, 1, 2, 0, 0, 0, 0, 0, 0, 0, 7, 5, 6, 4, 0};
static const uint8_t KB_MAP_2_REV[] = {7, 0, 0, 0, 0, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0};

static const uint8_t KB_MAP_1[] = {4, 2, 3, 1, 14, 12, 13, 11};
static const uint8_t KB_MAP_2[] = {15, 10, 9, 8, 7, 6, 5, 0};

static uint8_t load_xram8(void *arg, uint8_t addr) {
    minitel_t *minitel = (minitel_t *)arg;
    return ef9345_read(minitel->gfx, addr);
}

static uint8_t load_xram16(void *arg, uint16_t addr) {
    return load_xram8(arg, addr & 0xFF);
}

static void store_xram8(void *arg, uint8_t addr, uint8_t value) {
    minitel_t *minitel = (minitel_t *)arg;
    return ef9345_write(minitel->gfx, addr, value);
}

static void store_xram16(void *arg, uint16_t addr, uint8_t value) {
    store_xram8(arg, addr & 0xFF, value);
}

// That's a callback
static void set_port(void *arg, uint8_t port, uint8_t value) {
    minitel_t *minitel = (minitel_t *)arg;

    if (port == 1) {
        uint8_t mask = minitel->keydown[value & 7];
        mcu_8051_set_port(minitel->mcu, 2, 0xFF & ~mask);
    }
}

minitel_t *minitel_init(const uint8_t *rom, const uint8_t *charset, modem_callback_t modem_cb, void *cb_arg) {
    minitel_t *minitel = (minitel_t *)malloc(sizeof(minitel_t));
    if (!minitel)
        return NULL;

    mcu_8051_config_t config = {
        .rom = rom,
        .rom_mask = 0x1FFF,

        .cb_arg = (void *)minitel,
        .load_xram8_cb = load_xram8,
        .load_xram16_cb = load_xram16,
        .store_xram8_cb = store_xram8,
        .store_xram16_cb = store_xram16,

        .set_port_cb = set_port,
    };

    minitel->mcu = mcu_8051_init(&config);
    minitel->gfx = ef9345_init(minitel->vram, 0x1FFF, charset);
    modem_init(&minitel->modem, modem_cb, cb_arg);

    memset(minitel->keydown, 0, sizeof(minitel->keydown));

    return minitel;
}

void minitel_run(minitel_t *minitel, uint32_t usec) {
    uint32_t min_cycles = (11.059200/12) * usec;
    uint32_t cycles = 0;
    while (cycles < min_cycles) {
        uint32_t instr_cycles = mcu_8051_run_instr(minitel->mcu);
        cycles += instr_cycles;

        // TODO: The EF9345 is running slower (should be 12 MHz, is 11.0592 MHz)
        ef9345_update(minitel->gfx, instr_cycles * 12);
        modem_update(&minitel->modem, minitel->mcu, instr_cycles * 12);
    }
}

void minitel_keyboard_set(minitel_t *minitel, uint8_t keycode, bool keydown) {
    int msb = (keycode >> 4) & 0xF;
    int lsb = (keycode >> 0) & 0xF;

    int left = KB_MAP_1_REV[msb] | KB_MAP_1_REV[lsb];
    int right = KB_MAP_2_REV[msb] | KB_MAP_2_REV[lsb];

    if (keydown)
        minitel->keydown[left] |= (1 << right);
    else
        minitel->keydown[left] &= ~(1 << right);
}

bool minitel_modem_tx(minitel_t *minitel, uint8_t *buf, unsigned len) {
    return modem_tx(&minitel->modem, buf, len);
}

int minitel_render(minitel_t *minitel, uint8_t *buf) {
    return ef9345_render(minitel->gfx, buf);
}

void minitel_free(minitel_t *minitel) {
    free(minitel);
}
