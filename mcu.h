#ifndef MCU_H
#define MCU_H

#include <stdint.h>

typedef uint8_t (*get_port_cb_t)(void *arg, uint8_t port);
typedef void (*set_port_cb_t)(void *arg, uint8_t port, uint8_t val);
typedef uint8_t (*load_xram8_cb_t)(void *arg, uint8_t addr);
typedef uint8_t (*load_xram16_cb_t)(void *arg, uint16_t addr);
typedef void (*store_xram8_cb_t)(void *arg, uint8_t addr, uint8_t val);
typedef void (*store_xram16_cb_t)(void *arg, uint16_t addr, uint8_t val);

typedef struct mcu_8051_config_t {
    const uint8_t *rom;
    uint16_t rom_mask;

    void *cb_arg;
    get_port_cb_t get_port_cb;
    set_port_cb_t set_port_cb;
    load_xram8_cb_t load_xram8_cb;
    load_xram16_cb_t load_xram16_cb;
    store_xram8_cb_t store_xram8_cb;
    store_xram16_cb_t store_xram16_cb;

} mcu_8051_config_t;

typedef struct mcu_8051_t mcu_8051_t;

mcu_8051_t *mcu_8051_init(const mcu_8051_config_t *config);
int mcu_8051_run_instr(mcu_8051_t *mcu);
uint8_t mcu_8051_get_port(mcu_8051_t *mcu, uint8_t port);
void mcu_8051_set_port(mcu_8051_t *mcu, uint8_t port, uint8_t value);
char *mcu_8051_dump_state(mcu_8051_t *mcu);

#endif