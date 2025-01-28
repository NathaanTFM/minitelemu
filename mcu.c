#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include "mcu.h"

#if INT_MAX != 0x7FFFFFFF
#error INT_MAX not 32
#endif

#define IRQ_EXTI_0 0
#define IRQ_TIMER_0 1
#define IRQ_EXTI_1 2
#define IRQ_TIMER_1 3
#define IRQ_SERIAL 4
#define IRQ_TIMER_2 5

typedef struct bit_addr_t {
    uint8_t addr;
    uint8_t bit;
} bit_addr_t;

struct mcu_8051_t {
    // Registers and SFRs
    uint8_t a, b;
    uint8_t psw;
    uint8_t ie, ip;
    uint8_t scon, sbuf;
    uint8_t tcon, tmod;
    uint8_t p0, p1, p2, p3;
    uint8_t t2con, t2mod;

    // 16-bit registers
    uint16_t pc, sp, dptr;
    uint16_t rcap2;
    uint16_t timer0, timer1, timer2;

    // RAM and ROM pointers
    uint8_t ram[0x100]; // 0x100

    const uint8_t *rom;
    uint16_t rom_mask;

    // Flags
    struct {
        uint8_t c : 1;
        uint8_t ac : 1;
        uint8_t ov : 1;
    } flags;

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

static uint8_t calc_parity(uint8_t byte) {
    byte ^= (byte >> 4);
    byte &= 0xF;
    return (0x6996 >> byte) & 1;
}

static uint8_t op_util(uint8_t op, uint8_t left, uint8_t right) {
    switch (op) {
        case 0: return left | right;
        case 1: return left & right;
        case 2: return left ^ right;
    }

    fprintf(stderr, "unk op %d\n", op);
    return 0xFF;
}

static bit_addr_t parse_bit_addr(uint8_t addr) {
    bit_addr_t ret;
    if (addr >= 0x80) {
        ret.addr = addr & ~7;

    } else {
        ret.addr = 0x20 + (addr >> 3);
    }

    ret.bit = addr & 7;
    return ret;
}

// basic memory access
static uint8_t load_code(mcu_8051_t *mcu, uint16_t adrs) {
    return mcu->rom[adrs & mcu->rom_mask];
}

static uint8_t load_ram(mcu_8051_t *mcu, uint8_t adrs) {
    return mcu->ram[adrs & 0xFF];
}

static void store_ram(mcu_8051_t *mcu, uint8_t adrs, uint8_t value) {
    mcu->ram[adrs & 0xFF] = value;
}

// basic register access
static uint8_t load_reg(mcu_8051_t *mcu, uint8_t index) {
    uint8_t addr = (mcu->psw & 0x18) + index;
    return load_ram(mcu, addr);
}

static void store_reg(mcu_8051_t *mcu, uint8_t index, uint8_t value) {
    uint8_t addr = (mcu->psw & 0x18) + index;
    store_ram(mcu, addr, value);
}

// stack
static void push_stack(mcu_8051_t *mcu, uint8_t val) {
    mcu->sp = (mcu->sp + 1) & 0xFF;
    store_ram(mcu, mcu->sp, val);
}

static uint8_t pop_stack(mcu_8051_t *mcu) {
    uint8_t value = load_ram(mcu, mcu->sp);
    mcu->sp = (mcu->sp - 1) & 0xFF;
    return value;
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

    } else {
        uint8_t irq = mcu->irq_stack[--mcu->irq_count];
        mcu->irq_mask &= ~(1<<irq);

        switch (irq) {
        case IRQ_TIMER_0:
            mcu->tcon &= ~(1<<5);
            break;

        case IRQ_TIMER_1:
            mcu->tcon &= ~(1<<7);
            break;
        }

    }
}

static bool on_irq(mcu_8051_t *mcu, uint16_t dst, uint8_t irq) {
    // Currently in IRQ?
    if (mcu->irq_mask & (1 << irq))
        return false;

    // IRQs enabled?
    if (!(mcu->ie & (1 << 7)))
        return false;

    // This specific IRQ enabled?
    if (!(mcu->ie & (1 << irq)))
        return false;
    
    mcu->irq_mask |= (1 << irq);
    mcu->irq_stack[mcu->irq_count++] = irq;
    call(mcu, dst);
    return true;
}

// FSR access
static uint8_t load_fsr(mcu_8051_t *mcu, uint8_t addr, bool to_write) {
    switch (addr) {
    case 0xE0: // ACC
        return mcu->a;
    
    case 0xF0: // B
        return mcu->b;
    
    case 0x81: // SP
        return mcu->sp;
    
    case 0xD0: // PSW
        uint8_t value = mcu->psw & 0x38; // keep f0, rs1, rs0
        value |= mcu->flags.c << 7;
        value |= mcu->flags.ac << 6;
        value |= mcu->flags.ov << 2;
        value |= calc_parity(mcu->a);
        return value;
    
    case 0x82: // DPL
        return mcu->dptr & 0xFF;
    
    case 0x83: // DPH
        return mcu->dptr >> 8;
    
    case 0xA8: // IE
        return mcu->ie;
    
    case 0xB8: // IP
        return mcu->ip;
    
    case 0x98: // SCON
        return mcu->scon;
    
    case 0x99: // SBUF
        return mcu->sbuf_input;
    
    case 0xCA: // RCAP2L
        return mcu->rcap2 & 0xFF;
    
    case 0xCB: // RCAP2H
        return mcu->rcap2 >> 8;
    
    case 0x88: // TCON
        return mcu->tcon;
    
    case 0x89: // TMOD
        return mcu->tmod;
    
    case 0x80: // P0
        if (to_write)
            return mcu->p0;
            
        if (mcu->get_port_cb)
            mcu->p0_input = mcu->get_port_cb(mcu->cb_arg, 0);

        return mcu->p0_input & mcu->p0;
    
    case 0x90: // P1
        if (to_write)
            return mcu->p1;

        if (mcu->get_port_cb)
            mcu->p1_input = mcu->get_port_cb(mcu->cb_arg, 1);
            
        return mcu->p1_input & mcu->p1;
    
    case 0xA0: // P2
        if (to_write)
            return mcu->p2;

        if (mcu->get_port_cb)
            mcu->p2_input = mcu->get_port_cb(mcu->cb_arg, 2);
            
        return mcu->p2_input & mcu->p2;
    
    case 0xB0: // P3
        if (to_write)
            return mcu->p3;

        if (mcu->get_port_cb)
            mcu->p3_input = mcu->get_port_cb(mcu->cb_arg, 3);
            
        return mcu->p3_input & mcu->p3;
    
    case 0x8A: // TL0
        return mcu->timer0 & 0xFF;
    
    case 0x8B: // TL1
        return mcu->timer1 & 0xFF;
    
    case 0x8C: // TH0
        return mcu->timer0 >> 8;
    
    case 0x8D: // TH1
        return mcu->timer1 >> 8;
        
    case 0xCC: // TL2
        return mcu->timer2 & 0xFF;
    
    case 0xCD: // TH2
        return mcu->timer2 >> 8;
        
    case 0xC8: // T2CON
        return mcu->t2con;
    
    case 0xC9: // T2MOD
        return mcu->t2mod;

    default:
        fprintf(stderr, "load_fsr %02X (from %04X)", addr, mcu->pc);
        return 0xFF;
    }
}

static void store_fsr(mcu_8051_t *mcu, uint8_t addr, uint8_t value) {
    switch (addr) {
    case 0xE0: // ACC
        mcu->a = value;
        break;

    case 0xF0: // B
        mcu->b = value;
        break;

    case 0x81: // SP
        mcu->sp = value;
        break;

    case 0xD0: // PSW
        mcu->flags.c = (value >> 7) & 1;
        mcu->flags.ac = (value >> 6) & 1;
        mcu->flags.ov = (value >> 2) & 1;
        mcu->psw = value;
        break;

    case 0x82: // DPL
        mcu->dptr = (mcu->dptr & 0xFF00) | value;
        break;

    case 0x83: // DPH
        mcu->dptr = (mcu->dptr & 0xFF) | (value << 8);
        break;

    case 0xA8: // IE
        mcu->ie = value;
        break;

    case 0xB8: // IP
        mcu->ip = value;
        break;

    case 0x98: // SCON
        mcu->scon = value;
        break;

    case 0x99: // SBUF
        mcu->sbuf = value;
        mcu->sbuf_set = true;
        break;

    case 0xCA: // RCAP2L
        mcu->rcap2 = (mcu->rcap2 & 0xFF00) | value;
        break;

    case 0xCB: // RCAP2H
        mcu->rcap2 = (mcu->rcap2 & 0xFF) | (value << 8);
        break;

    case 0x88: // TCON
        mcu->tcon = value;
        break;

    case 0x89: // TMOD
        mcu->tmod = value;
        break;

    case 0x80: // P0
        mcu->p0 = value;
        mcu->set_port_cb(mcu->cb_arg, 0, mcu->p0);
        break;

    case 0x90: // P1
        mcu->p1 = value;
        mcu->set_port_cb(mcu->cb_arg, 1, mcu->p1);
        break;

    case 0xA0: // P2
        mcu->p2 = value;
        mcu->set_port_cb(mcu->cb_arg, 2, mcu->p2);
        break;

    case 0xB0: // P3
        mcu->p3 = value;
        mcu->set_port_cb(mcu->cb_arg, 3, mcu->p3);
        break;

    case 0x8A: // TL0
        mcu->timer0 = (mcu->timer0 & 0xFF00) | value;
        break;

    case 0x8B: // TL1
        mcu->timer1 = (mcu->timer1 & 0xFF00) | value;
        break;

    case 0x8C: // TH0
        mcu->timer0 = (mcu->timer0 & 0xFF) | (value << 8);
        break;

    case 0x8D: // TH1
        mcu->timer1 = (mcu->timer1 & 0xFF) | (value << 8);
        break;
        
    case 0xCC: // TL2
        mcu->timer2 = (mcu->timer2 & 0xFF00) | value;
        break;

    case 0xCD: // TH2
        mcu->timer2 = (mcu->timer2 & 0xFF) | (value << 8);
        break;
        
    case 0xC8: // T2CON
        mcu->t2con = value;
        break;

    case 0xC9: // T2MOD
        mcu->t2mod = value;
        break;

    default:
        fprintf(stderr, "store_fsr %02X = %02X (from %04X)\n", addr, value, mcu->pc);
        abort();
    }
}

// IRAM access depends on SFRs
static uint8_t load_iram(mcu_8051_t *mcu, uint8_t addr, bool to_write) {
    if (addr >= 0x80) {
        return load_fsr(mcu, addr, to_write);
    } else {
        return load_ram(mcu, addr);
    }
}

static void store_iram(mcu_8051_t *mcu, uint8_t addr, uint8_t value) {
    if (addr >= 0x80) {
        store_fsr(mcu, addr, value);
    } else {
        store_ram(mcu, addr, value);
    }
}

// bit addr mcu
static uint8_t load_bit_addr(mcu_8051_t *mcu, uint8_t addr, bool to_write) {
    bit_addr_t parsed = parse_bit_addr(addr);
    uint8_t value = load_iram(mcu, parsed.addr, to_write);
    return (value >> parsed.bit) & 1;
}

static void store_bit_addr(mcu_8051_t *mcu, uint8_t addr, uint8_t value) {
    bit_addr_t parsed = parse_bit_addr(addr);

    uint8_t tmp = load_iram(mcu, parsed.addr, true);
    if (value)
        tmp |= (1 << parsed.bit);
    else
        tmp &= ~(1 << parsed.bit);

    store_iram(mcu, parsed.addr, tmp);
}

// for instructions: load and store @Rn (6-7) Rn (8-F)
static uint8_t load_rn(mcu_8051_t *mcu, uint8_t off) {
    if (off >= 6 && off <= 7) {
        return load_ram(mcu, load_reg(mcu, off - 6)); // @R0, @R1

    } else if (off >= 8 && off <= 0xF) {
        return load_reg(mcu, off - 8); // R0-R7

    } else {
        fprintf(stderr, "load_rn: unsupported %02X\n", off);
        return 0xFF;
    }    
}

static void store_rn(mcu_8051_t *mcu, uint8_t off, uint8_t value) {
    if (off >= 6 && off <= 7) {
        store_ram(mcu, load_reg(mcu, off - 6), value);

    } else if (off >= 8 && off <= 0xF) {
        store_reg(mcu, off - 8, value);

    } else {
        fprintf(stderr, "store_rn: unsupported %02X\n", off);
    }
}

// returns cycle count
static int exec_instr(mcu_8051_t *mcu) {
    uint8_t opcode = load_code(mcu, mcu->pc);

    switch (opcode) {
    case 0x00: { // NOP
        mcu->pc++;
        return 1;
    }

    case 0x01: // AJMP and ACALL
    case 0x11:
    case 0x21:
    case 0x31:
    case 0x41:
    case 0x51:
    case 0x61:
    case 0x71:
    case 0x81:
    case 0x91:
    case 0xA1:
    case 0xB1:
    case 0xC1:
    case 0xD1:
    case 0xE1:
    case 0xF1: {
        uint8_t page = opcode >> 5;
        uint8_t addr = load_code(mcu, mcu->pc + 1);
        uint16_t target = (mcu->pc & ~0x7FF) | ((uint16_t)page << 8) | addr;
        mcu->pc += 2;

        if (opcode & 0x10)
            call(mcu, target);
        else
            jump(mcu, target);

        return 2;
    }

    case 0x02: // LJMP and LCALL
    case 0x12: {
        uint16_t target = load_code(mcu, mcu->pc + 1) << 8;
        target |= load_code(mcu, mcu->pc + 2);
        mcu->pc += 3;

        if (opcode & 0x10)
            call(mcu, target);
        else
            jump(mcu, target);

        return 2;
    }

    case 0x03: { // RR A
        mcu->a = (mcu->a >> 1) | ((mcu->a & 1) << 7);
        mcu->pc += 1;
        return 1;
    }

    case 0x04: { // INC A
        mcu->a = (mcu->a + 1) & 0xFF;
        mcu->pc += 1;
        return 1;
    }
        
    case 0x05: { // INC iram addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, true);
        store_iram(mcu, addr, (value + 1) & 0xFF);
        mcu->pc += 2;
        return 1;
    }

    case 0x06:
    case 0x07:
    case 0x08:
    case 0x09:
    case 0x0A:
    case 0x0B:
    case 0x0C:
    case 0x0D:
    case 0x0E:
    case 0x0F: { // INC @Rn, INC Rn        
        uint8_t value = load_rn(mcu, opcode & 0xF);
        store_rn(mcu, opcode & 0xF, (value + 1) & 0xFF);
        mcu->pc += 1;
        return 1;
    }
        
    case 0x10: { // JBC bit addr, reladdr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        int8_t target = (int8_t)load_code(mcu, mcu->pc+2);
        uint8_t value = load_bit_addr(mcu, addr, false); // XXX TODO FIXME
        mcu->pc += 3;
        
        if (value) {
            store_bit_addr(mcu, addr, 0);
            jump(mcu, mcu->pc + target);
        }

        return 2;
    }
        
    case 0x13: { // RRC A
        uint8_t tmp = mcu->a & 1;
        mcu->a = (mcu->a >> 1) | (mcu->flags.c << 7);
        mcu->flags.c = tmp;
        mcu->pc += 1;
        return 1;
    }
        
    case 0x14: { // DEC A
        mcu->a = (mcu->a - 1) & 0xFF;
        mcu->pc += 1;
        return 1;
    }
        
    case 0x15: { // DEC iram addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, true);
        store_iram(mcu, addr, (value - 1) & 0xFF);
        mcu->pc += 2;
        return 1;
    }

    case 0x16:
    case 0x17:
    case 0x18:
    case 0x19:
    case 0x1A:
    case 0x1B:
    case 0x1C:
    case 0x1D:
    case 0x1E:
    case 0x1F: { // DEC @Rn, DEC Rn
        uint8_t value = load_rn(mcu, opcode & 0xF);
        store_rn(mcu, opcode & 0xF, (value - 1) & 0xFF);
        mcu->pc += 1;
        return 1;
    }
        
    case 0x20: { // JB bit addr, reladdr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        int8_t target = (int8_t)load_code(mcu, mcu->pc+2);
        uint8_t value = load_bit_addr(mcu, addr, false);
        mcu->pc += 3;

        if (value)
            jump(mcu, mcu->pc + target);
        
        return 2;
    }
        
    case 0x22: { // RET
        ret(mcu);
        return 2;
    }
        
    case 0x23: { // RL A
        mcu->a = ((mcu->a << 1) & 0xFF) | (mcu->a >> 7);
        mcu->pc += 1;
        return 1;
    }
    
    case 0x24: // ADD A, operand
    case 0x25: // ADDC A, operand
    case 0x26:
    case 0x27:
    case 0x28:
    case 0x29:
    case 0x2A:
    case 0x2B:
    case 0x2C:
    case 0x2D:
    case 0x2E:
    case 0x2F:
    case 0x34:
    case 0x35:
    case 0x36:
    case 0x37:
    case 0x38:
    case 0x39:
    case 0x3A:
    case 0x3B:
    case 0x3C:
    case 0x3D:
    case 0x3E:
    case 0x3F: {
        uint16_t value;
        mcu->pc += 1;

        if ((opcode & 0xF) == 0x4) {
            value = load_code(mcu, mcu->pc);
            mcu->pc += 1;

        } else if ((opcode & 0xF) == 0x5) { // iram addr
            value = load_iram(mcu, load_code(mcu, mcu->pc), false);
            mcu->pc += 1;
            
        } else {
            value = load_rn(mcu, opcode & 0xF);
        }

        // todo: check flags
        
        if (opcode & 0x10) {
            value += mcu->flags.c;
        }
        
        // C flag    
        int16_t temp = (int16_t)mcu->a + value;
        mcu->flags.c = (temp > 255) ? 1 : 0;
        
        // AC flag
        temp = ((mcu->a & 0xF) + (value & 0xF));
        mcu->flags.ac = (temp > 15) ? 1 : 0;
            
        // OV flag
        temp = (int8_t)(value) + (int8_t)(mcu->a);
        mcu->flags.ov = (temp < -128 || temp > 127) ? 1 : 0;
        
        // store A
        mcu->a = (mcu->a + value) & 0xFF;
        return 1;
    }

    case 0x30: { // JNB bit addr, reladdr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+2));
        uint8_t value = load_bit_addr(mcu, addr, false);
        
        mcu->pc += 3;
        if (!value)
            jump(mcu, mcu->pc + target);

        return 2;
    }
        
    case 0x32: { // RETI
        reti(mcu);
        return 2;
    }
        
    case 0x33: { // RLC A
        uint8_t tmp = (mcu->a >> 7) & 1;
        mcu->a = ((mcu->a << 1) & 0xFF) | mcu->flags.c;
        mcu->flags.c = tmp;
        mcu->pc += 1;
        return 1;
    }
        
    case 0x40: { // JC reladdr
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));

        mcu->pc += 2;
        if (mcu->flags.c)
            jump(mcu, mcu->pc + target);

        return 2;
    }

    case 0x42: // ORL iram addr, A
    case 0x52: // ANL iram addr, A
    case 0x62: { // XRL iram addr, A
        uint8_t op = (opcode >> 4) - 4;
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, true);
        value = op_util(op, value, mcu->a);
        store_iram(mcu, addr, value);
        mcu->pc += 2;
        return 1;
    }
        
    case 0x43:
    case 0x53:
    case 0x63: {
        // ORL iram addr, #data
        // ANL iram addr, #data
        // XRL iram addr, #data
        uint8_t op = (opcode >> 4) - 4;
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t data = load_code(mcu, mcu->pc+2);
        uint8_t value = load_iram(mcu, addr, true);
        value = op_util(op, value, data);
        store_iram(mcu, addr, value);
        mcu->pc += 3;
        return 2;
    }
        
    case 0x44: // ORL A, #data
    case 0x54: // ANL A, #data
    case 0x64: { // XRL A, #data
        uint8_t op = (opcode >> 4) - 4;
        uint8_t data = load_code(mcu, mcu->pc+1);
        mcu->a = op_util(op, mcu->a, data);
        mcu->pc += 2;
        return 1;
    }

    case 0x45: // ORL A, iram addr
    case 0x55: // ANL A, iram addr
    case 0x65: { // XRL A, iram addr
        uint8_t op = (opcode >> 4) - 4;
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, false);
        mcu->a = op_util(op, mcu->a, value);
        mcu->pc += 2;
        return 1;
    }
        
    case 0x82: { // ANL C, bit addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        mcu->flags.c &= load_bit_addr(mcu, addr, false);
        mcu->pc += 2;
        return 1;
    }
        
    case 0xB0: { // ANL C,/bit addr
        fprintf(stderr, "Not handled ANL C,/bit addr");
        return 2;
    }

    case 0x46: // ORL A, Rn
    case 0x47: // ORL A, @Rn
    case 0x48: // ANL A, Rn
    case 0x49: // ANL A, @Rn
    case 0x4A: // XRL A, Rn
    case 0x4B: // XRL A, @Rn
    case 0x4C:
    case 0x4D:
    case 0x4E:
    case 0x4F:
    case 0x56:
    case 0x57:
    case 0x58:
    case 0x59:
    case 0x5A:
    case 0x5B:
    case 0x5C:
    case 0x5D:
    case 0x5E:
    case 0x5F:
    case 0x66:
    case 0x67:
    case 0x68:
    case 0x69:
    case 0x6A:
    case 0x6B:
    case 0x6C:
    case 0x6D:
    case 0x6E:
    case 0x6F: {
        uint8_t op = (opcode >> 4) - 4;
        uint8_t value = load_rn(mcu, opcode & 0xF);
        mcu->a = op_util(op, mcu->a, value);
        mcu->pc += 1;
        return 1;
    }
        
    case 0x50: { // JNC reladdr
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));
        
        mcu->pc += 2;
        if (!mcu->flags.c)
            jump(mcu, mcu->pc + target);
        
        return 2;
    }
        
    case 0x60: { // JZ reladdr
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));
        
        mcu->pc += 2;
        if (mcu->a == 0)
            jump(mcu, mcu->pc + target);

        return 2;
    }
        
    case 0x70: { // JNZ reladdr
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));

        mcu->pc += 2;
        if (mcu->a != 0)
            jump(mcu, mcu->pc + target);
        
        return 2;
    }
        
    case 0x72: { // ORL C, bit addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        mcu->flags.c |= load_bit_addr(mcu, addr, false);
        mcu->pc += 2;
        return 2;
    }
        
    case 0x73: { // JMP @A+DPTR
        jump(mcu, mcu->a + mcu->dptr);
        return 2;
    }
        
    case 0x80: { // SJMP reladdr
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));
        mcu->pc += 2;
        jump(mcu, mcu->pc + target);
        return 2;
    }
        
    case 0x84: { // DIV AB
        mcu->flags.c = 0;

        if (mcu->b == 0) {
            mcu->flags.ov = 1;
        } else {
            uint8_t quotient = mcu->a / mcu->b;
            uint8_t remainder = mcu->a % mcu->b;
            mcu->a = quotient & 0xFF;
            mcu->b = remainder & 0xFF;
            mcu->flags.ov = 0;
        }
            
        mcu->pc += 1;
        return 4;
    }

    case 0x94: // SUBB A, operand
    case 0x95:
    case 0x96:
    case 0x97:
    case 0x98:
    case 0x99:
    case 0x9A:
    case 0x9B:
    case 0x9C:
    case 0x9D:
    case 0x9E:
    case 0x9F: {
        uint16_t value;
        mcu->pc += 1;
            
        if ((opcode & 0xF) == 0x4) { // #data
            value = load_code(mcu, mcu->pc);
            mcu->pc += 1;

        } else if ((opcode & 0xF) == 0x5) { // iram addr
            value = load_iram(mcu, load_code(mcu, mcu->pc), false);
            mcu->pc += 1;
            
        } else {
            value = load_rn(mcu, opcode & 0xF);
        }
            
        // this can't be right TODO (also check for ADD/ADC)
        value += mcu->flags.c;
        
        // C flag
        int16_t temp = (int16_t)mcu->a - (int16_t)value;
        mcu->flags.c = (temp < 0) ? 1 : 0;
        
        // AC flag
        temp = ((mcu->a & 0xF) - (value & 0xF));
        mcu->flags.ac = (temp > 15) ? 1 : 0;
            
        // OV flag
        temp = (int8_t)(value) + (int8_t)(mcu->a);
        mcu->flags.ov = (temp < -128 || temp > 127) ? 1 : 0;
        
        // store A
        mcu->a = (mcu->a - value) & 0xFF;
        return 1;
    }
        
    case 0xA0: { // ORL C, /bit addr
        fprintf(stderr, "is this bitwise not?\n");
        return 2;
    }
        
    case 0xA3: { // INC DPTR
        mcu->dptr = (mcu->dptr + 1) & 0xFFFF;
        mcu->pc += 1;
        return 2;
    }
        
    case 0xA4: { // MUL AB
        uint16_t res = (uint16_t)mcu->a * (uint16_t)mcu->b;
        mcu->flags.c = 0;
        mcu->flags.ov = (res > 255) ? 1 : 0;
        
        mcu->a = res & 0xFF;
        mcu->b = (res >> 8) & 0xFF;
        mcu->pc += 1;

        return 4;
    }
        
    case 0xA5: {
        fprintf(stderr, "undefined instruction A5\n");
        abort();
    }
        
    case 0xB2: { // CPL bit addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_bit_addr(mcu, addr, 1 - load_bit_addr(mcu, addr, true));
        mcu->pc += 2;
        return 1;
    }
        
    case 0xB3: { // CPL C
        mcu->flags.c ^= 1;
        mcu->pc += 1;
        return 1;
    }

    case 0xB4: // CJNE operand1, operand2, reladdr
    case 0xB5:
    case 0xB6:
    case 0xB7:
    case 0xB8:
    case 0xB9:
    case 0xBA:
    case 0xBB:
    case 0xBC:
    case 0xBD:
    case 0xBE:
    case 0xBF: {
        uint8_t operand1, operand2;
        if (opcode == 0xB4) {
            operand1 = mcu->a;
            operand2 = load_code(mcu, mcu->pc+1);
        
        } else if (opcode == 0xB5) {
            operand1 = mcu->a;
            operand2 = load_iram(mcu, load_code(mcu, mcu->pc+1), false);
            
        } else {
            operand1 = load_rn(mcu, opcode & 0xF);
            operand2 = load_code(mcu, mcu->pc+1);
        }
        
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+2));
        mcu->flags.c = (operand1 < operand2) ? 1 : 0;
        
        mcu->pc += 3;
        if (operand1 != operand2)
            jump(mcu, mcu->pc + target);

        return 2;
    }
            
    case 0xC0: { // PUSH iram addr 
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, false);
        push_stack(mcu, value);
        mcu->pc += 2;
        return 2;
    }
    
    case 0xC2: // CLR bit addr
    case 0xD2: { // SETB bit addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_bit_addr(mcu, addr, (opcode >> 4) & 1);
        mcu->pc += 2;
        return 1;
    }
    
    case 0xC3: // CLR C
    case 0xD3: { // SETB C
        mcu->flags.c = (opcode >> 4) & 1;
        mcu->pc += 1;
        return 1;
    }

    case 0xC4: { // SWAP A
        mcu->a = ((mcu->a << 4) | (mcu->a >> 4)) & 0xFF;
        mcu->pc += 1;
        return 1;
    }
        
    case 0xC5: { // XCHG A, iram addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t tmp = mcu->a;
        mcu->a = load_iram(mcu, addr, false);
        store_iram(mcu, addr, tmp);
        mcu->pc += 2;
        return 1;
    }
        
    case 0xC6: // XCHG A, Rn
    case 0xC7: // XCHG A, @Rn
    case 0xC8:
    case 0xC9:
    case 0xCA:
    case 0xCB:
    case 0xCC:
    case 0xCD:
    case 0xCE:
    case 0xCF: {
        uint8_t tmp = mcu->a;
        mcu->a = load_rn(mcu, opcode & 0xF);
        store_rn(mcu, opcode & 0xF, tmp);
        mcu->pc += 1;
        return 1;
    }
        
    case 0xD0: { // POP iram addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = pop_stack(mcu);
        store_iram(mcu, addr, value);
        mcu->pc += 2;
        return 2;
    }
        
    case 0xD4: { // DA
        fprintf(stderr, "Instruction DA is not implemented\n");
        return 1;
    }
        
    case 0xD5: {
        // DJNZ iram addr, reladdr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+2));
        uint8_t value = load_iram(mcu, addr, false); // FIXME TODO XXX 
        value = (value - 1) & 0xFF;
        store_iram(mcu, addr, value);
        
        mcu->pc += 3;
        if (value != 0)
            jump(mcu, mcu->pc + target);
        
        return 2;
    }
            
    case 0xD6: // XCHD A, @Rn
    case 0xD7: {
        uint8_t value = load_rn(mcu, opcode & 0xF);
        uint8_t tmp = mcu->a & 0xF;
        mcu->a = (mcu->a & 0xF0) | (value & 0xF);
        store_rn(mcu, opcode & 0xF, (value & 0xF0) | tmp);
        mcu->pc += 1;
        return 1;
    }
    
    case 0xD8: // DJNZ Rn, reladdr
    case 0xD9:
    case 0xDA:
    case 0xDB:
    case 0xDC:
    case 0xDD:
    case 0xDE:
    case 0xDF: {
        int8_t target = (int8_t)(load_code(mcu, mcu->pc+1));
        uint8_t value = load_rn(mcu, opcode & 0xF);
        value = (value - 1) & 0xFF;
        store_rn(mcu, opcode & 0xF, value);

        mcu->pc += 2;
        if (value != 0)
            jump(mcu, mcu->pc + target);
        
        return 2;
    }
        
    case 0xE4: { // CLR A
        mcu->a = 0;
        mcu->pc += 1;
        return 1;
    }
        
    case 0xF4: { // CPL A
        mcu->a = mcu->a ^ 0xFF;
        mcu->pc += 1;
        return 1;
    }
        
    case 0x74: { // MOV A,#data
        uint8_t data = load_code(mcu, mcu->pc+1);
        mcu->a = data;
        mcu->pc += 2;
        return 1;
    }
        
    case 0x75: { // MOV iram addr, #data
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t data = load_code(mcu, mcu->pc+2);
        store_iram(mcu, addr, data);
        mcu->pc += 3;
        return 2;
    }
        
    case 0x76: // MOV @Rn, #data
    case 0x77: // MOV Rn, #data
    case 0x78:
    case 0x79:
    case 0x7A:
    case 0x7B:
    case 0x7C:
    case 0x7D:
    case 0x7E:
    case 0x7F: {
        uint8_t data = load_code(mcu, mcu->pc+1);
        store_rn(mcu, opcode & 0xF, data);
        mcu->pc += 2;
        return 1;
    }
        
    case 0x83: { // MOVC A, @A+PC
        mcu->pc += 1;
        mcu->a = load_code(mcu, mcu->a + mcu->pc); // TODO: check which pc it's supposed to be!
        return 2;
    }
        
    case 0x85: { // MOV iram addr, iram addr
        uint8_t src = load_code(mcu, mcu->pc+1);
        uint8_t dst = load_code(mcu, mcu->pc+2);
        store_iram(mcu, dst, load_iram(mcu, src, false));
        mcu->pc += 3;
        return 2;
    }

    case 0x86: // MOV iram addr, @Rn
    case 0x87: // MOV iram addr, Rn
    case 0x88:
    case 0x89:
    case 0x8A:
    case 0x8B:
    case 0x8C:
    case 0x8D:
    case 0x8E:
    case 0x8F: {
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_iram(mcu, addr, load_rn(mcu, opcode & 0xF));
        mcu->pc += 2;
        return 2;
    }
        
    case 0x90: { // MOV DPTR, #data16
        // check endianness!
        uint16_t data16 = 0;
        data16 |= (uint16_t)load_code(mcu, mcu->pc+1) << 8;
        data16 |= load_code(mcu, mcu->pc+2);
        mcu->dptr = data16;
        mcu->pc += 3;
        return 2;
    }
        
    case 0x92: { // MOV bit addr, C
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_bit_addr(mcu, addr, mcu->flags.c);
        mcu->pc += 2;
        return 2;
    }
        
    case 0xA2: { // MOV C, bit addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        mcu->flags.c = load_bit_addr(mcu, addr, false);
        mcu->pc += 2;
        return 1;
    }

    case 0xA6: // MOV Rn, iram addr
    case 0xA7: // MOV @Rn, iram addr
    case 0xA8:
    case 0xA9:
    case 0xAA:
    case 0xAB:
    case 0xAC:
    case 0xAD:
    case 0xAE:
    case 0xAF: {
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_rn(mcu, opcode & 0xF, load_iram(mcu, addr, false));
        mcu->pc += 2;
        return 2;
    }
        
    case 0xE5: { // MOV A, iram addr
        uint8_t addr = load_code(mcu, mcu->pc+1);
        uint8_t value = load_iram(mcu, addr, false);
        mcu->a = value;
        mcu->pc += 2;
        return 1;
    }
        
    case 0xE6: // MOV A, Rn
    case 0xE7: // MOV A, @Rn
    case 0xE8:
    case 0xE9:
    case 0xEA:
    case 0xEB:
    case 0xEC:
    case 0xED:
    case 0xEE:
    case 0xEF: {
        uint8_t value = load_rn(mcu, opcode & 0xF);
        mcu->a = value;
        mcu->pc += 1;
        return 1;
    }
        
    case 0xF5: { // MOV iram addr, A
        uint8_t addr = load_code(mcu, mcu->pc+1);
        store_iram(mcu, addr, mcu->a);
        mcu->pc += 2;
        return 1;
    }
    
    case 0xF6: // MOV Rn, A
    case 0xF7: // MOV @Rn, A
    case 0xF8:
    case 0xF9:
    case 0xFA:
    case 0xFB:
    case 0xFC:
    case 0xFD:
    case 0xFE:
    case 0xFF: {
        store_rn(mcu, opcode & 0xF, mcu->a);
        mcu->pc += 1;
        return 1;
    }
        
    case 0x93: { // MOVC A, @A+DPTR
        mcu->a = load_code(mcu, mcu->a + mcu->dptr);
        mcu->pc += 1;
        return 2;
    }
            
    case 0xE0: { // MOVX A, @DPTR
        uint8_t value = mcu->load_xram16_cb(mcu->cb_arg, mcu->dptr);
        mcu->a = value;
        mcu->pc += 1;
        return 2;
    }
        
    case 0xE2: // MOVX A, @Rn
    case 0xE3: {
        uint8_t value = mcu->load_xram8_cb(mcu->cb_arg, load_reg(mcu, opcode - 0xE2));
        mcu->a = value;
        mcu->pc += 1;
        return 2;
    }
        
    case 0xF0: { // MOVX @DPTR, A
        mcu->store_xram16_cb(mcu->cb_arg, mcu->dptr, mcu->a);
        mcu->pc += 1;
        return 2;
    }
        
    case 0xF2: // MOVX @Rn, A
    case 0xF3: {
        mcu->store_xram8_cb(mcu->cb_arg, load_reg(mcu, opcode - 0xF2), mcu->a);
        mcu->pc += 1;
        return 2;
    }
    }

    fprintf(stderr, "Unhandled instruction %02X\n", opcode);
    abort();
}

static void update_timer01(mcu_8051_t *mcu, int timer, int cycles) {
    uint8_t mode;
    uint8_t run;
    uint32_t value;

    if (timer == 0) {
        mode = mcu->tmod & 15;
        run = (mcu->tcon >> 4) & 1;
        value = mcu->timer0;

    } else {
        mode = (mcu->tmod >> 4) & 15;
        run = (mcu->tcon >> 6) & 1;
        value = mcu->timer1;
    }

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
        bool overflow = false;

        switch (mode) {
        case 0:
            // 8-bit counter with 5-bit prescaler
            value &= 0x1FFF;
            value += cycles;

            if (value > 0x1FFF) {
                value &= 0x1FFF;
                overflow = true;
            }
            
            break;

        case 1:
            // 16-bit timer/counter
            value &= 0xFFFF;
            value += cycles;

            if (value > 0xFFFF) {
                value &= 0xFFFF;
                overflow = true;
            }

            break;

        case 2: {
            uint8_t reload = (value >> 8) & 0xFF;
            value &= 0xFF;
            value += cycles;

            if (value > 0xFF) {
                value &= 0xFF;
                value += reload;
                overflow = true;
            }

            value |= (reload << 8);
            break;
        }

        case 3:
            fprintf(stderr, "Timer mode 3 not implemented\n");
            abort();
            break;
        }

        if (overflow) {
            if (timer == 0)
                mcu->tcon |= (1<<5);
            else
                mcu->tcon |= (1<<7);
        }
    }

    if (timer == 0)
        mcu->timer0 = (uint16_t)value;
    else
        mcu->timer1 = (uint16_t)value;
}

static void update_timer2(mcu_8051_t *mcu, int cycles) {
    int down_counter = mcu->t2mod & 1; // ???
    // # output_enable is not relevant yet TODO what is this

    if (mcu->t2con & 4) {
        if (mcu->t2con & 0x7B) {
            fprintf(stderr, "t2con not handled\n");
            abort();
        }

        uint32_t timer2 = mcu->timer2 + cycles;

        if (timer2 > 0xFFFF) {
            timer2 += mcu->rcap2;
            mcu->t2con |= (1<<7); // overflow
        }

        mcu->timer2 = timer2 & 0xFFFF;
    }
}

static void update_timers(mcu_8051_t *mcu, int cycles) {
    update_timer01(mcu, 0, cycles);
    update_timer01(mcu, 1, cycles);
    update_timer2(mcu, cycles);

    if (mcu->tcon & (1<<5)) {
        on_irq(mcu, 0x0B, IRQ_TIMER_0);
    }

    if (mcu->tcon & (1<<7)) {
        on_irq(mcu, 0x1B, IRQ_TIMER_1);
    }

    if (mcu->t2con & (1<<7)) {
        on_irq(mcu, 0x2B, IRQ_TIMER_2);
    }
}

static void update_serial(mcu_8051_t *mcu, int cycles) {
    uint8_t mode = (mcu->scon >> 6) & 3;
    (void)mode;
    (void)cycles;

    // TODO: sbuf_out is not None
    if (mcu->sbuf_set) {
        mcu->sbuf_set = false;
        mcu->scon |= (1<<1);
    }

    if (mcu->scon & 3) {
        on_irq(mcu, 0x23, IRQ_SERIAL);
    }
}

static void update_exti(mcu_8051_t *mcu) {
    if (mcu->get_port_cb)
        mcu->p3_input = mcu->get_port_cb(mcu->cb_arg, 3);
        
    // INT0 (P3.2) and INT1 (P3.3)
    for (int n = 0; n < 2; n++) {
        if (mcu->tcon & (1<<(n*2))) {
            // Interrupt on falling edge
            if ((mcu->p3_prev_input & (1<<(n+2))) && !(mcu->p3_input & (1<<(n+2)))) {
                mcu->tcon |= (1<<(1+n*2));
            }

        } else {
            // Interrupt on low level
            if (!(mcu->p3_input & (1<<(n+2)))) {
                mcu->tcon |= (1<<(1+n*2));
            }
        }
    }

    mcu->p3_prev_input = mcu->p3_input;

    // Check IRQ
    if (mcu->tcon & (1<<1)) {
        if (on_irq(mcu, 0x03, IRQ_EXTI_0)) {
            mcu->tcon &= ~(1 << 0);
        }
    }

    if (mcu->tcon & (1<<3)) {
        if (on_irq(mcu, 0x13, IRQ_EXTI_1)) {
            mcu->tcon &= ~(1 << 2);
        }
    }
}

int mcu_8051_run_instr(mcu_8051_t *mcu) {
    // Update EXTI before
    update_exti(mcu);

    int cycles = exec_instr(mcu);
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

mcu_8051_t *mcu_8051_init(const mcu_8051_config_t *config) {
    mcu_8051_t *mcu = calloc(1, sizeof(mcu_8051_t));
    if (!mcu)
        return NULL;

    mcu->p0 = mcu->p1 = mcu->p2 = mcu->p3 = 0xFF;
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
    return mcu;
}

char *mcu_8051_dump_state(mcu_8051_t *mcu) {
    static char buf[4096];
    char *ptr = buf;

    ptr+=sprintf(ptr, "PC: %04X\n", mcu->pc);
    ptr+=sprintf(ptr, "A: %02X\n", mcu->a);
    ptr+=sprintf(ptr, "B: %02X\n", mcu->b);
    ptr+=sprintf(ptr, "DPTR: %04X\n", mcu->dptr);
    ptr+=sprintf(ptr, "SP: %04X\n", mcu->sp);
    for (int i = 0; i < 8; i++) {
        ptr+=sprintf(ptr, "R%d: %02X\n", i, load_reg(mcu, i));
    }
    ptr+=sprintf(ptr, "TIMER0: %04X\n", mcu->timer0);
    ptr+=sprintf(ptr, "TIMER1: %04X\n", mcu->timer1);
    ptr+=sprintf(ptr, "TIMER2: %04X\n", mcu->timer2);
    ptr+=sprintf(ptr, "P0: %02X\n", mcu->p0_input);
    ptr+=sprintf(ptr, "P1: %02X\n", mcu->p1_input);
    ptr+=sprintf(ptr, "P2: %02X\n", mcu->p2_input);
    ptr+=sprintf(ptr, "P3: %02X\n", mcu->p3_input);

    return buf;
}