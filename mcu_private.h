#ifndef MCU_PRIVATE_H
#define MCU_PRIVATE_H

#include "mcu.h"

#include <stdbool.h>
#include <stdint.h>

// IRQs
#define IRQ_EXTI_0 0
#define IRQ_TIMER_0 1
#define IRQ_EXTI_1 2
#define IRQ_TIMER_1 3
#define IRQ_SERIAL 4
#define IRQ_TIMER_2 5

// SFRs
#define SFR_ACC 0xE0
#define SFR_B 0xF0
#define SFR_SP 0x81
#define SFR_PSW 0xD0
#define SFR_DPL 0x82
#define SFR_DPH 0x83
#define SFR_IE 0xA8
#define SFR_IP 0xB8
#define SFR_SCON 0x98
#define SFR_SBUF 0x99
#define SFR_RCAP2L 0xCA
#define SFR_RCAP2H 0xCB
#define SFR_TCON 0x88
#define SFR_TMOD 0x89
#define SFR_P0 0x80
#define SFR_P1 0x90
#define SFR_P2 0xA0
#define SFR_P3 0xB0
#define SFR_TL0 0x8A
#define SFR_TL1 0x8B
#define SFR_TH0 0x8C
#define SFR_TH1 0x8D
#define SFR_TL2 0xCC
#define SFR_TH2 0xCD
#define SFR_T2CON 0xC8
#define SFR_T2MOD 0xC9

#define SFR_PSW_C_BIT 7
#define SFR_PSW_AC_BIT 6
#define SFR_PSW_OV_BIT 2
#define SFR_PSW_P_BIT 0

#define SFR_PSW_P (1<<SFR_PSW_P_BIT)
#define SFR_PSW_C (1<<SFR_PSW_C_BIT)
#define SFR_PSW_AC (1<<SFR_PSW_AC_BIT)
#define SFR_PSW_OV (1<<SFR_PSW_OV_BIT)

struct mcu_8051_t {
    // Registers and SFRs

    // 16-bit registers
    uint16_t pc;

    // RAM and ROM pointers
    uint8_t ram[0x100]; // 0x100
    uint8_t sfr[0x100];

    // Shortcut for register access
    uint8_t *reg;

    const uint8_t *rom;
    uint16_t rom_mask;

    bool sbuf_set;

    // Current irq
    uint8_t irq_stack[8];
    uint8_t irq_mask;
    uint8_t irq_count;

    // Input ports
    uint8_t p0_input, p1_input, p2_input, p3_input;
    uint8_t p3_prev_input;
    uint8_t sbuf_input;

    // Callbacks
    void *cb_arg;
    get_port_cb_t get_port_cb;
    set_port_cb_t set_port_cb;
    load_xram8_cb_t load_xram8_cb;
    load_xram16_cb_t load_xram16_cb;
    store_xram8_cb_t store_xram8_cb;
    store_xram16_cb_t store_xram16_cb;

};

uint8_t mcu_8051_get_iram(mcu_8051_t *mcu, uint8_t addr);
uint8_t mcu_8051_load_iram(mcu_8051_t *mcu, uint8_t addr);
void mcu_8051_store_iram(mcu_8051_t *mcu, uint8_t addr, uint8_t value);

#define get_iram mcu_8051_get_iram
#define load_iram mcu_8051_load_iram
#define store_iram mcu_8051_store_iram

// basic memory access
static uint8_t load_code(mcu_8051_t *mcu, uint16_t adrs) {
    return mcu->rom[adrs & mcu->rom_mask];
}

// stack
static void push_stack(mcu_8051_t *mcu, uint8_t val) {
    uint8_t addr = ++mcu->sfr[SFR_SP];
    mcu->ram[addr] = val;
}

static uint8_t pop_stack(mcu_8051_t *mcu) {
    uint8_t addr = mcu->sfr[SFR_SP]--;
    return mcu->ram[addr];
}

// branching
static void jump(mcu_8051_t *mcu, uint16_t dst) {
    mcu->pc = dst;
}

static void call(mcu_8051_t *mcu, uint16_t dst) {
    push_stack(mcu, mcu->pc);
    push_stack(mcu, mcu->pc >> 8);
    jump(mcu, dst);
}

static void ret(mcu_8051_t *mcu) {
    uint16_t dst = 0;
    dst |= pop_stack(mcu) << 8;
    dst |= pop_stack(mcu);
    jump(mcu, dst);
}

// irq branching
static void reti(mcu_8051_t *mcu) {
    ret(mcu);

    if (mcu->irq_count == 0) {
        fprintf(stderr, "RETI but not in interrupt!\n");
        abort();
    }

    uint8_t irq = mcu->irq_stack[--mcu->irq_count];
    mcu->irq_mask |= (1<<irq);
}

#endif