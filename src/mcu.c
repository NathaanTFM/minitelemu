#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "mcu_private.h"
#include "mcu_instr.h"

#if INT_MAX != 0x7FFFFFFF
#error INT_MAX not 32
#endif

static uint8_t calc_parity(uint8_t byte) {
    byte ^= (byte >> 4);
    byte &= 0xF;
    return (0x6996 >> byte) & 1;
}

static inline void update_reg(mcu_8051_t *mcu) {
    mcu->reg = &mcu->ram[mcu->sfr[SFR_PSW] & 0x18];
}

static bool on_irq(mcu_8051_t *mcu, uint16_t dst, uint8_t irq) {
    // Can this IRQ be reached?
    uint8_t mask = (1 << 7) | (1 << irq);
    if (((mcu->irq_mask & mcu->sfr[SFR_IE]) & mask) != mask)
        return false;
    
    mcu->irq_mask &= ~(1 << irq);
    mcu->irq_stack[mcu->irq_count++] = irq;
    call(mcu, dst);
    return true;
}

static inline uint8_t get_psw(mcu_8051_t *mcu) {
    return (mcu->sfr[SFR_PSW] & ~SFR_PSW_P) | (calc_parity(mcu->sfr[SFR_ACC]) << SFR_PSW_P_BIT);
}

// SFR access - this is only called for loading, not when the intent is to write
static uint8_t load_sfr(mcu_8051_t *mcu, uint8_t addr) {
    switch (addr) {
    case SFR_PSW:
        return get_psw(mcu);
        
    case SFR_P0: // P0
        if (mcu->get_port_cb)
            mcu->p0_input = mcu->get_port_cb(mcu->cb_arg, 0);

        return mcu->p0_input & mcu->sfr[SFR_P0];
    
    case SFR_P1: // P1
        if (mcu->get_port_cb)
            mcu->p1_input = mcu->get_port_cb(mcu->cb_arg, 1);
            
        return mcu->p1_input & mcu->sfr[SFR_P1];
    
    case SFR_P2: // P2
        if (mcu->get_port_cb)
            mcu->p2_input = mcu->get_port_cb(mcu->cb_arg, 2);
            
        return mcu->p2_input & mcu->sfr[SFR_P2];
    
    case SFR_P3: // P3
        if (mcu->get_port_cb)
            mcu->p3_input = mcu->get_port_cb(mcu->cb_arg, 3);
            
        return mcu->p3_input & mcu->sfr[SFR_P3];
    
    default:
        return mcu->sfr[addr];
    }
}

static void store_sfr(mcu_8051_t *mcu, uint8_t addr, uint8_t value) {
    mcu->sfr[addr] = value;

    switch (addr) {
    case SFR_PSW:
        update_reg(mcu);
        break;

    case SFR_SBUF: // SBUF
        mcu->sbuf_set = true;
        break;

    case SFR_P0: // P0
        if (mcu->set_port_cb)
            mcu->set_port_cb(mcu->cb_arg, 0, mcu->sfr[SFR_P0]);
        break;

    case SFR_P1: // P1
        if (mcu->set_port_cb)
            mcu->set_port_cb(mcu->cb_arg, 1, mcu->sfr[SFR_P1]);
        break;

    case SFR_P2: // P2
        if (mcu->set_port_cb)
            mcu->set_port_cb(mcu->cb_arg, 2, mcu->sfr[SFR_P2]);
        break;

    case SFR_P3: // P3
        if (mcu->set_port_cb)
            mcu->set_port_cb(mcu->cb_arg, 3, mcu->sfr[SFR_P3]);
        break;
    }
}

// IRAM access depends on SFRs
uint8_t mcu_8051_get_iram(mcu_8051_t *mcu, uint8_t addr) {
    if (addr & 0x80)
        return (addr == SFR_PSW ? get_psw(mcu) : mcu->sfr[addr]);
    else
        return mcu->ram[addr];
}

uint8_t mcu_8051_load_iram(mcu_8051_t *mcu, uint8_t addr) {
    if (addr & 0x80)
        return load_sfr(mcu, addr);
    else
        return mcu->ram[addr];
}

void mcu_8051_store_iram(mcu_8051_t *mcu, uint8_t addr, uint8_t value) {
    if (addr & 0x80)
        store_sfr(mcu, addr, value);
    else
        mcu->ram[addr] = value;
}

static bool update_timer01(mcu_8051_t *mcu, int cycles, int mode, int run, uint8_t *restrict low, uint8_t *restrict high) {
    (void)mcu;
    bool overflow = false;

    if (mode & 8) {
        fprintf(stderr, "GATEn Not implemented\n");
        abort();
    }

    if (mode & 4) {
        fprintf(stderr, "C/Tn# not implemented");
        abort();
    }

    mode &= 3;

    if (run) {
        switch (mode) {
        case 0:
            // 8-bit counter with 5-bit prescaler
            // (basically, 13-bit counter)
            *high += (*low + cycles) >> 8;
            *low += cycles;

            overflow = !!(*high & ~0x1F);
            *high &= 0x1F;
            break;

        case 1:
            // 16-bit timer/counter
            int tmp = (*low + cycles) >> 8;
            overflow = !!((*high + tmp) >> 8);

            *high += tmp;
            *low += cycles;
            break;

        case 2: {
            // 8-bit timer with 8-bit reload
            overflow = !!((*low + cycles) >> 8);
            *low += cycles;
            if (overflow)
                *low += *high;

            break;
        }

        case 3:
            fprintf(stderr, "Timer mode 3 not implemented\n");
            abort();
            break;
        }
    }

    return overflow;
}

static bool update_timer2(mcu_8051_t *mcu, int cycles) {
    bool overflow = false;
    int down_counter = mcu->sfr[SFR_T2MOD] & 1; // ???
    if (down_counter) {
        fprintf(stderr, "down_counter timer2 is not implemented\n");
        abort();
    }
    // # output_enable is not relevant yet TODO what is this

    if (mcu->sfr[SFR_T2CON] & 4) {
        if (mcu->sfr[SFR_T2CON] & 0x7B) {
            fprintf(stderr, "t2con not handled\n");
            abort();
        }
        
        uint32_t timer2 = mcu->sfr[SFR_TL2] | (mcu->sfr[SFR_TH2] << 8);
        timer2 += cycles;

        if (timer2 > 0xFFFF) {
            overflow = true;
            timer2 += mcu->sfr[SFR_RCAP2L] | (mcu->sfr[SFR_RCAP2H] << 8);
        }

        mcu->sfr[SFR_TL2] = timer2;
        mcu->sfr[SFR_TH2] = timer2 >> 8;
    }

    return overflow;
}

static void update_timers(mcu_8051_t *mcu, int cycles) {
    if (update_timer01(mcu, cycles, (mcu->sfr[SFR_TMOD] >> 0) & 15, mcu->sfr[SFR_TCON] & (1<<4), &mcu->sfr[SFR_TL0], &mcu->sfr[SFR_TH0])) {
        mcu->sfr[SFR_TCON] |= (1<<5);
    }

    if (update_timer01(mcu, cycles, (mcu->sfr[SFR_TMOD] >> 4) & 15, mcu->sfr[SFR_TCON] & (1<<6), &mcu->sfr[SFR_TL1], &mcu->sfr[SFR_TH1])) {
        mcu->sfr[SFR_TCON] |= (1<<7);
    }

    if (update_timer2(mcu, cycles)) {
        mcu->sfr[SFR_T2CON] |= (1<<7);
    }

    if (mcu->sfr[SFR_TCON] & (1<<5)) {
        if (on_irq(mcu, 0x0B, IRQ_TIMER_0)) {
            mcu->sfr[SFR_TCON] &= ~(1<<5);
        }
    }

    if (mcu->sfr[SFR_TCON] & (1<<7)) {
        if (on_irq(mcu, 0x1B, IRQ_TIMER_1)) {
            mcu->sfr[SFR_TCON] &= ~(1<<7);
        }
    }

    if (mcu->sfr[SFR_T2CON] & (1<<7)) {
        on_irq(mcu, 0x2B, IRQ_TIMER_2);
    }
}

static void update_serial(mcu_8051_t *mcu, int cycles) {
    uint8_t mode = (mcu->sfr[SFR_SCON] >> 6) & 3;
    (void)mode;
    (void)cycles;

    // TODO: sbuf_out is not None
    if (mcu->sbuf_set) {
        mcu->sbuf_set = false;
        mcu->sfr[SFR_SCON] |= (1<<1);
    }

    if (mcu->sfr[SFR_SCON] & 3) {
        on_irq(mcu, 0x23, IRQ_SERIAL);
    }
}

static void update_exti(mcu_8051_t *mcu) {
    if (mcu->get_port_cb)
        mcu->p3_input = mcu->get_port_cb(mcu->cb_arg, 3);
        
    // INT0 (P3.2) and INT1 (P3.3)
    for (int n = 0; n < 2; n++) {
        if (mcu->sfr[SFR_TCON] & (1<<(n*2))) {
            // Interrupt on falling edge
            if ((mcu->p3_prev_input & (1<<(n+2))) && !(mcu->p3_input & (1<<(n+2)))) {
                mcu->sfr[SFR_TCON] |= (1<<(1+n*2));
            }

        } else {
            // Interrupt on low level
            if (!(mcu->p3_input & (1<<(n+2)))) {
                mcu->sfr[SFR_TCON] |= (1<<(1+n*2));
            }
        }
    }

    mcu->p3_prev_input = mcu->p3_input;

    // Check IRQ
    if (mcu->sfr[SFR_TCON] & (1<<1)) {
        if (on_irq(mcu, 0x03, IRQ_EXTI_0)) {
            mcu->sfr[SFR_TCON] &= ~(1 << 0);
        }
    }

    if (mcu->sfr[SFR_TCON] & (1<<3)) {
        if (on_irq(mcu, 0x13, IRQ_EXTI_1)) {
            mcu->sfr[SFR_TCON] &= ~(1 << 2);
        }
    }
}

#ifdef MCU_DEBUG
static char *dump_state(mcu_8051_t *mcu) {
    static char buf[4096];
    char *ptr = buf;

    ptr+=sprintf(ptr, "PC: %04X  ", mcu->pc);
    ptr+=sprintf(ptr, "A: %02X  ", mcu->sfr[SFR_ACC]);
    ptr+=sprintf(ptr, "B: %02X  ", mcu->sfr[SFR_B]);
    ptr+=sprintf(ptr, "DPTR: %02X%02X  ", mcu->sfr[SFR_DPH], mcu->sfr[SFR_DPL]);
    ptr+=sprintf(ptr, "SP: 00%02X  ", mcu->sfr[SFR_SP]);
    for (int i = 0; i < 8; i++) {
        ptr+=sprintf(ptr, "R%d: %02X  ", i, mcu->reg[i]);
    }
    ptr+=sprintf(ptr, "TIMER0: %02X%02X  ", mcu->sfr[SFR_TH0], mcu->sfr[SFR_TL0]);
    ptr+=sprintf(ptr, "TIMER1: %02X%02X  ", mcu->sfr[SFR_TH1], mcu->sfr[SFR_TL1]);
    ptr+=sprintf(ptr, "TIMER2: %02X%02X  ", mcu->sfr[SFR_TH2], mcu->sfr[SFR_TL2]);
    ptr+=sprintf(ptr, "P0: %02X  ", mcu->p0_input);
    ptr+=sprintf(ptr, "P1: %02X  ", mcu->p1_input);
    ptr+=sprintf(ptr, "P2: %02X  ", mcu->p2_input);
    ptr+=sprintf(ptr, "P3: %02X  ", mcu->p3_input);
    ptr+=sprintf(ptr, "C: %d  ", !!(get_psw(mcu) & SFR_PSW_C));

    return buf;
}
#endif

int mcu_8051_run_instr(mcu_8051_t *mcu) {
#ifdef MCU_DEBUG
    static FILE *f = NULL;
    if (f == NULL) {
        f = fopen("mcu_debug.txt", "w");
    }
#endif

    // Update EXTI before
    update_exti(mcu);

    int cycles = mcu_8051_exec_instr_internal(mcu);

#ifdef MCU_DEBUG
    fprintf(f, "%d ", cycles);
    fputs(dump_state(mcu), f);
    fputc('\n', f);
    fflush(stdout);
#endif

    update_timers(mcu, cycles);

    // Serial RX could be updated before (but not TX)?
    update_serial(mcu, cycles);

    return cycles;
}

void mcu_8051_set_port(mcu_8051_t *mcu, uint8_t port, uint8_t value) {
    switch (port) {
    case 0:
        mcu->p0_input = value;
        break;
    case 1:
        mcu->p1_input = value;
        break;
    case 2:
        mcu->p2_input = value;
        break;
    case 3:
        mcu->p3_input = value;
        break;
    default:
        fprintf(stderr, "Invalid port %d\n", port);
        abort();
    }
}

uint8_t mcu_8051_get_port(mcu_8051_t *mcu, uint8_t port) {
    switch (port) {
    case 0: return mcu->sfr[SFR_P0];
    case 1: return mcu->sfr[SFR_P1];
    case 2: return mcu->sfr[SFR_P2];
    case 3: return mcu->sfr[SFR_P3];
    default:
        fprintf(stderr, "Invalid port %d\n", port);
        abort();
    }
}

mcu_8051_t *mcu_8051_init(const mcu_8051_config_t *config) {
    mcu_8051_t *mcu = (mcu_8051_t *)calloc(1, sizeof(mcu_8051_t));
    if (!mcu)
        return NULL;

    mcu->irq_mask = 0xFF; // all IRQs are reachable
    mcu->sfr[SFR_P0] = mcu->sfr[SFR_P1] = mcu->sfr[SFR_P2] = mcu->sfr[SFR_P3] = 0xFF;
    mcu->p0_input = mcu->p1_input = mcu->p2_input = mcu->p3_input = mcu->p3_prev_input = 0xFF;

    mcu->rom = config->rom;
    mcu->rom_mask = config->rom_mask;

    mcu->cb_arg = config->cb_arg;
    mcu->get_port_cb = config->get_port_cb;
    mcu->set_port_cb = config->set_port_cb;
    mcu->load_xram8_cb = config->load_xram8_cb;
    mcu->load_xram16_cb = config->load_xram16_cb;
    mcu->store_xram8_cb = config->store_xram8_cb;
    mcu->store_xram16_cb = config->store_xram16_cb;

    update_reg(mcu);
    return mcu;
}
