#include <stdio.h>
#include <stdlib.h>

#include "mcu_private.h"
#include "mcu_instr.h"

#define ACC (mcu->sfr[SFR_ACC])
#define PSW (mcu->sfr[SFR_PSW])
#define DPTR ((mcu->sfr[SFR_DPH] << 8) | (mcu->sfr[SFR_DPL]))

static int get_bit(uint8_t *addr) {
    int bit = *addr & 7;
    
    if (*addr & 0x80) {
        *addr &= ~7;

    } else {
        *addr = 0x20 + (*addr >> 3);
    }

    return bit;
}

static inline void handle_add_flags(mcu_8051_t *mcu, uint8_t left, uint8_t right, int carry) {
    int c_flag = ((left & 255) + (right & 255) + carry) >> 8;
    int ac_flag = ((left & 7) + (right & 7) + carry) >> 3;
    int ov_flag = (((left & 127) + (right & 127) + carry) >> 7) ^ c_flag;

    PSW = (PSW & ~(SFR_PSW_AC | SFR_PSW_OV | SFR_PSW_C))
        | (c_flag << SFR_PSW_C_BIT)
        | (ac_flag << SFR_PSW_AC_BIT)
        | (ov_flag << SFR_PSW_OV_BIT);
}

static inline void handle_sub_flags(mcu_8051_t *mcu, uint8_t left, uint8_t right, int carry) {
    int c_flag = (((left & 255) - (right & 255) - carry) >> 8) & 1;
    int ac_flag = (((left & 7) - (right & 7) - carry) >> 3) & 1;
    int ov_flag = ((((left & 127) - (right & 127) - carry) >> 7) & 1) ^ c_flag;

    PSW = (PSW & ~(SFR_PSW_AC | SFR_PSW_OV | SFR_PSW_C))
        | (c_flag << SFR_PSW_C_BIT)
        | (ac_flag << SFR_PSW_AC_BIT)
        | (ov_flag << SFR_PSW_OV_BIT);
}

// Please keep these in the same order as in exec_instr

static inline int exec_instr_acall(mcu_8051_t *mcu, int page) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint16_t target = (mcu->pc & ~0x7FF) | (page << 8) | addr;
    call(mcu, target);
    return 2;
}

static inline int exec_instr_add_a_data(mcu_8051_t *mcu) {
    uint8_t value = load_code(mcu, mcu->pc++);
    handle_add_flags(mcu, ACC, value, 0);
    ACC += value;
    return 1;
}

static inline int exec_instr_add_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    handle_add_flags(mcu, ACC, value, 0);
    ACC += value;
    return 1;
}

static inline int exec_instr_add_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->ram[mcu->reg[reg]];
    handle_add_flags(mcu, ACC, value, 0);
    ACC += value;
    return 1;
}

static inline int exec_instr_add_a_reg(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->reg[reg];
    handle_add_flags(mcu, ACC, value, 0);
    ACC += value;
    return 1;
}

static inline int exec_instr_addc_a_data(mcu_8051_t *mcu) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = load_code(mcu, mcu->pc++);
    handle_add_flags(mcu, ACC, value, carry);
    ACC += value + carry;
    return 1;
}

static inline int exec_instr_addc_a_iram(mcu_8051_t *mcu) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    handle_add_flags(mcu, ACC, value, carry);
    ACC += value + carry;
    return 1;
}

static inline int exec_instr_addc_a_ind(mcu_8051_t *mcu, int reg) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = mcu->ram[mcu->reg[reg]];
    handle_add_flags(mcu, ACC, value, carry);
    ACC += value + carry;
    return 1;
}

static inline int exec_instr_addc_a_reg(mcu_8051_t *mcu, int reg) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = mcu->reg[reg];
    handle_add_flags(mcu, ACC, value, carry);
    ACC += value + carry;
    return 1;
}

static inline int exec_instr_ajmp(mcu_8051_t *mcu, int page) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint16_t target = (mcu->pc & ~0x7FF) | (page << 8) | addr;
    jump(mcu, target);
    return 2;
}

static inline int exec_instr_anl_iram_a(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) & ACC;
    store_iram(mcu, addr, value);
    return 1;
}

static inline int exec_instr_anl_iram_data(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t data = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) & data;
    store_iram(mcu, addr, value);
    return 2;
}

static inline int exec_instr_anl_a_data(mcu_8051_t *mcu) {
    uint8_t data = load_code(mcu, mcu->pc++);
    ACC &= data;
    return 1;
}

static inline int exec_instr_anl_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    ACC &= value;
    return 1;
}

static inline int exec_instr_anl_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->ram[mcu->reg[reg]];
    ACC &= value;
    return 1;
}

static inline int exec_instr_anl_a_reg(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->reg[reg];
    ACC &= value;
    return 1;
}

static inline int exec_instr_anl_c_bit(mcu_8051_t *mcu, int negate) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = ((load_iram(mcu, addr) >> bit) & 1) ^ negate;
    PSW &= (~SFR_PSW_C) | (value << SFR_PSW_C_BIT);
    return 1;
}

static inline int exec_instr_cjne_a_data(mcu_8051_t *mcu) {
    uint8_t value = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int c_flag = (ACC < value) ? SFR_PSW_C : 0;
    PSW = (PSW & ~SFR_PSW_C) | c_flag;
    
    if (ACC != value)
        jump(mcu, mcu->pc + target);

    return 2;
}

static inline int exec_instr_cjne_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    int c_flag = (ACC < value) ? SFR_PSW_C : 0;
    PSW = (PSW & ~SFR_PSW_C) | c_flag;
    
    if (ACC != value)
        jump(mcu, mcu->pc + target);

    return 2;
}

static inline int exec_instr_cjne_ind_data(mcu_8051_t *mcu, int reg) {
    uint8_t addr = mcu->reg[reg];
    uint8_t value = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int c_flag = (mcu->ram[addr] < value) ? SFR_PSW_C : 0;
    PSW = (PSW & ~SFR_PSW_C) | c_flag;
    
    if (mcu->ram[addr] != value)
        jump(mcu, mcu->pc + target);

    return 2;
}

static inline int exec_instr_cjne_reg_data(mcu_8051_t *mcu, int reg) {
    uint8_t value = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int c_flag = (mcu->reg[reg] < value) ? SFR_PSW_C : 0;
    PSW = (PSW & ~SFR_PSW_C) | c_flag;
    
    if (mcu->reg[reg] != value)
        jump(mcu, mcu->pc + target);

    return 2;
}

static inline int exec_instr_clr_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    store_iram(mcu, addr, get_iram(mcu, addr) & ~(1 << bit));
    return 1;
}

static inline int exec_instr_clr_c(mcu_8051_t *mcu) {
    PSW &= ~SFR_PSW_C;
    return 1;
}

static inline int exec_instr_clr_a(mcu_8051_t *mcu) {
    ACC = 0;
    return 1;
}

static inline int exec_instr_cpl_a(mcu_8051_t *mcu) {
    ACC ^= 0xFF;
    return 1;
}

static inline int exec_instr_cpl_c(mcu_8051_t *mcu) {
    PSW ^= SFR_PSW_C;
    return 1;
}

static inline int exec_instr_cpl_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    store_iram(mcu, addr, get_iram(mcu, addr) ^ (1 << bit));
    return 1;
}

static inline int exec_instr_da(mcu_8051_t *mcu) {
    return 1;
}

static inline int exec_instr_dec_a(mcu_8051_t *mcu) {
    ACC--;
    return 1;
}

static inline int exec_instr_dec_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr);
    store_iram(mcu, addr, value-1);
    return 1;
}

static inline int exec_instr_dec_ind(mcu_8051_t *mcu, int reg) {    
    mcu->ram[mcu->reg[reg]]--;
    return 1;
}

static inline int exec_instr_dec_reg(mcu_8051_t *mcu, int reg) {    
    mcu->reg[reg]--;
    return 1;
}

static inline int exec_instr_div_ab(mcu_8051_t *mcu) {
    if (mcu->sfr[SFR_B] != 0) {
        uint8_t quotient = ACC / mcu->sfr[SFR_B];
        uint8_t remainder = ACC % mcu->sfr[SFR_B];
        
        ACC = quotient;
        mcu->sfr[SFR_B] = remainder;

        PSW &= ~(SFR_PSW_OV | SFR_PSW_C);

    } else {
        PSW = (PSW & ~SFR_PSW_C) | SFR_PSW_OV;
    }
    
    return 4;
}

static inline int exec_instr_djnz_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) - 1;
    store_iram(mcu, addr, value);
    
    if (value != 0)
        jump(mcu, mcu->pc + target);
    
    return 2;
}

static inline int exec_instr_djnz_reg(mcu_8051_t *mcu, int reg) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    if ((--mcu->reg[reg]) != 0)
        jump(mcu, mcu->pc + target);
    
    return 2;
}

static inline int exec_instr_inc_a(mcu_8051_t *mcu) {
    ACC++;
    return 1;
}

static inline int exec_instr_inc_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr);
    store_iram(mcu, addr, value+1);
    return 1;
}

static inline int exec_instr_inc_ind(mcu_8051_t *mcu, int reg) {    
    mcu->ram[mcu->reg[reg]]++;
    return 1;
}

static inline int exec_instr_inc_reg(mcu_8051_t *mcu, int reg) {    
    mcu->reg[reg]++;
    return 1;
}

static inline int exec_instr_inc_dptr(mcu_8051_t *mcu) {
    uint16_t tmp = mcu->sfr[SFR_DPL] + 1;
    mcu->sfr[SFR_DPH] += (tmp >> 8);
    mcu->sfr[SFR_DPL] = tmp;
    return 2;
}

static inline int exec_instr_jb_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = load_iram(mcu, addr);

    if (value & (1<<bit)) {
        jump(mcu, mcu->pc + target);
    }
    
    return 2;
}

static inline int exec_instr_jbc_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = get_iram(mcu, addr);
    
    if (value & (1<<bit)) {
        value &= ~(1 << bit);
        store_iram(mcu, addr, value);
        jump(mcu, mcu->pc + target);
    }

    return 2;
}

static inline int exec_instr_jc(mcu_8051_t *mcu) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);

    if (PSW & SFR_PSW_C)
        jump(mcu, mcu->pc + target);

    return 2;
}

static inline int exec_instr_jmp_a_dptr(mcu_8051_t *mcu) {
    jump(mcu, ACC + DPTR);
    return 2;
}

static inline int exec_instr_jnb_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = load_iram(mcu, addr);
    
    if (!(value & (1<<bit))) {
        jump(mcu, mcu->pc + target);
    }

    return 2;
}

static inline int exec_instr_jnc(mcu_8051_t *mcu) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);

    if (!(PSW & SFR_PSW_C))
        jump(mcu, mcu->pc + target);
    
    return 2;
}

static inline int exec_instr_jnz(mcu_8051_t *mcu) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);

    if (ACC != 0)
        jump(mcu, mcu->pc + target);
    
    return 2;
}

static inline int exec_instr_jz(mcu_8051_t *mcu) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);

    if (ACC == 0)
        jump(mcu, mcu->pc + target);
    
    return 2;
}

static inline int exec_instr_lcall(mcu_8051_t *mcu) {
    uint16_t target = load_code(mcu, mcu->pc++) << 8;
    target |= load_code(mcu, mcu->pc++);
    call(mcu, target);
    return 2;
}

static inline int exec_instr_ljmp(mcu_8051_t *mcu) {
    uint16_t target = load_code(mcu, mcu->pc++) << 8;
    target |= load_code(mcu, mcu->pc++);
    jump(mcu, target);
    return 2;
}

static inline int exec_instr_mov_ind_data(mcu_8051_t *mcu, int reg) {
    uint8_t data = load_code(mcu, mcu->pc++);
    mcu->ram[mcu->reg[reg]] = data;
    return 1;
}

static inline int exec_instr_mov_ind_a(mcu_8051_t *mcu, int reg) {
    mcu->ram[mcu->reg[reg]] = ACC;
    return 1;
}

static inline int exec_instr_mov_ind_iram(mcu_8051_t *mcu, int reg) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    mcu->ram[mcu->reg[reg]] = load_iram(mcu, addr);
    return 2;
}

static inline int exec_instr_mov_a_data(mcu_8051_t *mcu) {
    uint8_t data = load_code(mcu, mcu->pc++);
    ACC = data;
    return 1;
}

static inline int exec_instr_mov_a_ind(mcu_8051_t *mcu, int reg) {
    ACC = mcu->ram[mcu->reg[reg]];
    return 1;
}

static inline int exec_instr_mov_a_reg(mcu_8051_t *mcu, int reg) {
    ACC = mcu->reg[reg];
    return 1;
}

static inline int exec_instr_mov_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    ACC = load_iram(mcu, addr);
    return 1;
}

static inline int exec_instr_mov_c_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = load_iram(mcu, addr);
    PSW = (PSW & ~SFR_PSW_C) | (((value >> bit) & 1) << SFR_PSW_C_BIT);
    return 1;
}

static inline int exec_instr_mov_dptr_data(mcu_8051_t *mcu) {
    mcu->sfr[SFR_DPH] = load_code(mcu, mcu->pc++);
    mcu->sfr[SFR_DPL] = load_code(mcu, mcu->pc++);
    return 2;
}

static inline int exec_instr_mov_reg_data(mcu_8051_t *mcu, int reg) {
    uint8_t data = load_code(mcu, mcu->pc++);
    mcu->reg[reg] = data;
    return 1;
}

static inline int exec_instr_mov_reg_a(mcu_8051_t *mcu, int reg) {
    mcu->reg[reg] = ACC;
    return 1;
}

static inline int exec_instr_mov_reg_iram(mcu_8051_t *mcu, int reg) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    mcu->reg[reg] = load_iram(mcu, addr);
    return 2;
}

static inline int exec_instr_mov_bit_c(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = get_iram(mcu, addr);
    store_iram(mcu, addr, (value & ~(1<<bit)) | (((PSW >> SFR_PSW_C_BIT) & 1) << bit));
    return 2;
}

static inline int exec_instr_mov_iram_data(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t data = load_code(mcu, mcu->pc++);
    store_iram(mcu, addr, data);
    return 2;
}

static inline int exec_instr_mov_iram_ind(mcu_8051_t *mcu, int reg) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    store_iram(mcu, addr, mcu->ram[mcu->reg[reg]]);
    return 2;
}

static inline int exec_instr_mov_iram_reg(mcu_8051_t *mcu, int reg) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    store_iram(mcu, addr, mcu->reg[reg]);
    return 2;
}

static inline int exec_instr_mov_iram_a(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    store_iram(mcu, addr, ACC);
    return 1;
}

static inline int exec_instr_mov_iram_iram(mcu_8051_t *mcu) {
    uint8_t src = load_code(mcu, mcu->pc++);
    uint8_t dst = load_code(mcu, mcu->pc++);
    store_iram(mcu, dst, load_iram(mcu, src));
    return 2;
}

static inline int exec_instr_movc_a_dptr(mcu_8051_t *mcu) {
    ACC = load_code(mcu, ACC + DPTR);
    return 2;
}

static inline int exec_instr_movc_a_pc(mcu_8051_t *mcu) {
    ACC = load_code(mcu, ACC + mcu->pc);
    return 2;
}

static inline int exec_instr_movx_dptr_a(mcu_8051_t *mcu) {
    mcu->store_xram16_cb(mcu->cb_arg, DPTR, ACC);
    return 2;
}

static inline int exec_instr_movx_reg_a(mcu_8051_t *mcu, int reg) {
    mcu->store_xram8_cb(mcu->cb_arg, mcu->reg[reg], ACC);
    return 2;
}

static inline int exec_instr_movx_a_dptr(mcu_8051_t *mcu) {
    ACC = mcu->load_xram16_cb(mcu->cb_arg, DPTR);
    return 2;
}

static inline int exec_instr_movx_a_reg(mcu_8051_t *mcu, int reg) {
    ACC = mcu->load_xram8_cb(mcu->cb_arg, mcu->reg[reg]);
    return 2;
}

static inline int exec_instr_mul_ab(mcu_8051_t *mcu) {
    uint16_t res = (uint16_t)ACC * (uint16_t)mcu->sfr[SFR_B];
    int ov_flag = (res & ~0xFF) ? SFR_PSW_OV : 0;
    
    ACC = res & 0xFF;
    mcu->sfr[SFR_B] = (res >> 8) & 0xFF;

    PSW = (PSW & ~(SFR_PSW_C | SFR_PSW_OV)) | ov_flag;
    return 4;
}

static inline int exec_instr_nop(mcu_8051_t *mcu) {
    return 1;
}

static inline int exec_instr_orl_iram_a(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) | ACC;
    store_iram(mcu, addr, value);
    return 1;
}

static inline int exec_instr_orl_iram_data(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t data = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) | data;
    store_iram(mcu, addr, value);
    return 2;
}

static inline int exec_instr_orl_a_data(mcu_8051_t *mcu) {
    uint8_t data = load_code(mcu, mcu->pc++);
    ACC |= data;
    return 1;
}

static inline int exec_instr_orl_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    ACC |= value;
    return 1;
}

static inline int exec_instr_orl_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->ram[mcu->reg[reg]];
    ACC |= value;
    return 1;
}

static inline int exec_instr_orl_a_reg(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->reg[reg];
    ACC |= value;
    return 1;
}

static inline int exec_instr_orl_c_bit(mcu_8051_t *mcu, int negate) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    uint8_t value = ((load_iram(mcu, addr) >> bit) & 1) ^ negate;
    PSW |= (value << SFR_PSW_C_BIT);
    return 2;
}

static inline int exec_instr_pop_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = pop_stack(mcu);
    store_iram(mcu, addr, value);
    return 2;
}

static inline int exec_instr_push_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    push_stack(mcu, value);
    return 2;
}

static inline int exec_instr_ret(mcu_8051_t *mcu) {
    ret(mcu);
    return 2;
}

static inline int exec_instr_reti(mcu_8051_t *mcu) {
    reti(mcu);
    return 2;
}

static inline int exec_instr_rl_a(mcu_8051_t *mcu) {
    ACC = (ACC << 1) | (ACC >> 7);
    return 1;
}

static inline int exec_instr_rlc_a(mcu_8051_t *mcu) {
    int prev_c = (PSW >> SFR_PSW_C_BIT) & 1;
    int new_c = (ACC >> 7) & 1;
    ACC = (ACC << 1) | prev_c;
    PSW = (PSW & ~SFR_PSW_C) | (new_c << SFR_PSW_C_BIT);
    return 1;
}

static inline int exec_instr_rr_a(mcu_8051_t *mcu) {
    ACC = (ACC >> 1) | (ACC << 7);
    return 1;
}

static inline int exec_instr_rrc_a(mcu_8051_t *mcu) {
    int prev_c = (PSW >> SFR_PSW_C_BIT) & 1;
    int new_c = ACC & 1;
    ACC = (ACC >> 1) | (prev_c << 7);
    PSW = (PSW & ~SFR_PSW_C) | (new_c << SFR_PSW_C_BIT);
    return 1;
}

static inline int exec_instr_setb_c(mcu_8051_t *mcu) {
    PSW |= SFR_PSW_C;
    return 1;
}

static inline int exec_instr_setb_bit(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    int bit = get_bit(&addr);
    store_iram(mcu, addr, get_iram(mcu, addr) | (1 << bit));
    return 1;
}

static inline int exec_instr_sjmp(mcu_8051_t *mcu) {
    int8_t target = (int8_t)load_code(mcu, mcu->pc++);
    jump(mcu, mcu->pc + target);
    return 2;
}

static inline int exec_instr_subb_a_data(mcu_8051_t *mcu) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = load_code(mcu, mcu->pc++);
    handle_sub_flags(mcu, ACC, value, carry);
    ACC -= value + carry;
    return 1;
}

static inline int exec_instr_subb_a_iram(mcu_8051_t *mcu) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    handle_sub_flags(mcu, ACC, value, carry);
    ACC -= value + carry;
    return 1;
}

static inline int exec_instr_subb_a_ind(mcu_8051_t *mcu, int reg) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = mcu->ram[mcu->reg[reg]];
    handle_sub_flags(mcu, ACC, value, carry);
    ACC -= value + carry;
    return 1;
}

static inline int exec_instr_subb_a_reg(mcu_8051_t *mcu, int reg) {
    int carry = (PSW >> SFR_PSW_C_BIT) & 1;
    uint8_t value = mcu->reg[reg];
    handle_sub_flags(mcu, ACC, value, carry);
    ACC -= value + carry;
    return 1;
}

static inline int exec_instr_swap_a(mcu_8051_t *mcu) {
    ACC = (ACC << 4) | (ACC >> 4);
    return 1;
}

static inline int exec_instr_xch_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t addr = mcu->reg[reg];
    uint8_t tmp = ACC;
    ACC = mcu->ram[addr];
    mcu->ram[addr] = tmp;
    return 1;
}

static inline int exec_instr_xch_a_reg(mcu_8051_t *mcu, int reg) {
    uint8_t tmp = ACC;
    ACC = mcu->reg[reg];
    mcu->reg[reg] = tmp;
    return 1;
}

static inline int exec_instr_xch_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t tmp = ACC;
    ACC = load_iram(mcu, addr);
    store_iram(mcu, addr, tmp);
    return 1;
}

static inline int exec_instr_xchd_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t addr = mcu->reg[reg];
    uint8_t value = mcu->ram[addr];
    uint8_t tmp = ACC;
    ACC = (ACC & 0xF0) | (value & 0x0F);
    mcu->ram[addr] = (value & 0xF0) | (tmp & 0x0F);
    return 1;
}


static inline int exec_instr_xrl_iram_a(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) ^ ACC;
    store_iram(mcu, addr, value);
    return 1;
}

static inline int exec_instr_xrl_iram_data(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t data = load_code(mcu, mcu->pc++);
    uint8_t value = get_iram(mcu, addr) ^ data;
    store_iram(mcu, addr, value);
    return 2;
}

static inline int exec_instr_xrl_a_data(mcu_8051_t *mcu) {
    uint8_t data = load_code(mcu, mcu->pc++);
    ACC ^= data;
    return 1;
}

static inline int exec_instr_xrl_a_iram(mcu_8051_t *mcu) {
    uint8_t addr = load_code(mcu, mcu->pc++);
    uint8_t value = load_iram(mcu, addr);
    ACC ^= value;
    return 1;
}

static inline int exec_instr_xrl_a_ind(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->ram[mcu->reg[reg]];
    ACC ^= value;
    return 1;
}

static inline int exec_instr_xrl_a_reg(mcu_8051_t *mcu, int reg) {
    uint8_t value = mcu->reg[reg];
    ACC ^= value;
    return 1;
}

// returns cycle count
int mcu_8051_exec_instr_internal(mcu_8051_t *mcu) {
    uint8_t opcode = load_code(mcu, mcu->pc++);

    switch (opcode) {
        // To simplify maintenance, the instructions are sorted as in
        // https://aeb.win.tue.nl/comp/8051/set8051.html

    case 0x11: return exec_instr_acall(mcu, 0);
    case 0x31: return exec_instr_acall(mcu, 1);
    case 0x51: return exec_instr_acall(mcu, 2);
    case 0x71: return exec_instr_acall(mcu, 3);
    case 0x91: return exec_instr_acall(mcu, 4);
    case 0xB1: return exec_instr_acall(mcu, 5);
    case 0xD1: return exec_instr_acall(mcu, 6);
    case 0xF1: return exec_instr_acall(mcu, 7);

    case 0x24: return exec_instr_add_a_data(mcu);
    case 0x25: return exec_instr_add_a_iram(mcu);
    case 0x26: return exec_instr_add_a_ind(mcu, 0);
    case 0x27: return exec_instr_add_a_ind(mcu, 1);
    case 0x28: return exec_instr_add_a_reg(mcu, 0);
    case 0x29: return exec_instr_add_a_reg(mcu, 1);
    case 0x2A: return exec_instr_add_a_reg(mcu, 2);
    case 0x2B: return exec_instr_add_a_reg(mcu, 3);
    case 0x2C: return exec_instr_add_a_reg(mcu, 4);
    case 0x2D: return exec_instr_add_a_reg(mcu, 5);
    case 0x2E: return exec_instr_add_a_reg(mcu, 6);
    case 0x2F: return exec_instr_add_a_reg(mcu, 7);
    case 0x34: return exec_instr_addc_a_data(mcu);
    case 0x35: return exec_instr_addc_a_iram(mcu);
    case 0x36: return exec_instr_addc_a_ind(mcu, 0);
    case 0x37: return exec_instr_addc_a_ind(mcu, 1);
    case 0x38: return exec_instr_addc_a_reg(mcu, 0);
    case 0x39: return exec_instr_addc_a_reg(mcu, 1);
    case 0x3A: return exec_instr_addc_a_reg(mcu, 2);
    case 0x3B: return exec_instr_addc_a_reg(mcu, 3);
    case 0x3C: return exec_instr_addc_a_reg(mcu, 4);
    case 0x3D: return exec_instr_addc_a_reg(mcu, 5);
    case 0x3E: return exec_instr_addc_a_reg(mcu, 6);
    case 0x3F: return exec_instr_addc_a_reg(mcu, 7);

    case 0x01: return exec_instr_ajmp(mcu, 0);
    case 0x21: return exec_instr_ajmp(mcu, 1);
    case 0x41: return exec_instr_ajmp(mcu, 2);
    case 0x61: return exec_instr_ajmp(mcu, 3);
    case 0x81: return exec_instr_ajmp(mcu, 4);
    case 0xA1: return exec_instr_ajmp(mcu, 5);
    case 0xC1: return exec_instr_ajmp(mcu, 6);
    case 0xE1: return exec_instr_ajmp(mcu, 7);

    case 0x52: return exec_instr_anl_iram_a(mcu);
    case 0x53: return exec_instr_anl_iram_data(mcu);
    case 0x54: return exec_instr_anl_a_data(mcu);
    case 0x55: return exec_instr_anl_a_iram(mcu);
    case 0x56: return exec_instr_anl_a_ind(mcu, 0);
    case 0x57: return exec_instr_anl_a_ind(mcu, 1);
    case 0x58: return exec_instr_anl_a_reg(mcu, 0);
    case 0x59: return exec_instr_anl_a_reg(mcu, 1);
    case 0x5A: return exec_instr_anl_a_reg(mcu, 2);
    case 0x5B: return exec_instr_anl_a_reg(mcu, 3);
    case 0x5C: return exec_instr_anl_a_reg(mcu, 4);
    case 0x5D: return exec_instr_anl_a_reg(mcu, 5);
    case 0x5E: return exec_instr_anl_a_reg(mcu, 6);
    case 0x5F: return exec_instr_anl_a_reg(mcu, 7);
    case 0x82: return exec_instr_anl_c_bit(mcu, 0);
    case 0xB0: return exec_instr_anl_c_bit(mcu, 1);

    case 0xB4: return exec_instr_cjne_a_data(mcu);
    case 0xB5: return exec_instr_cjne_a_iram(mcu);
    case 0xB6: return exec_instr_cjne_ind_data(mcu, 0);
    case 0xB7: return exec_instr_cjne_ind_data(mcu, 1);
    case 0xB8: return exec_instr_cjne_reg_data(mcu, 0);
    case 0xB9: return exec_instr_cjne_reg_data(mcu, 1);
    case 0xBA: return exec_instr_cjne_reg_data(mcu, 2);
    case 0xBB: return exec_instr_cjne_reg_data(mcu, 3);
    case 0xBC: return exec_instr_cjne_reg_data(mcu, 4);
    case 0xBD: return exec_instr_cjne_reg_data(mcu, 5);
    case 0xBE: return exec_instr_cjne_reg_data(mcu, 6);
    case 0xBF: return exec_instr_cjne_reg_data(mcu, 7);

    case 0xC2: return exec_instr_clr_bit(mcu);
    case 0xC3: return exec_instr_clr_c(mcu);
    case 0xE4: return exec_instr_clr_a(mcu);

    case 0xF4: return exec_instr_cpl_a(mcu);
    case 0xB3: return exec_instr_cpl_c(mcu);
    case 0xB2: return exec_instr_cpl_bit(mcu);

    case 0xD4: return exec_instr_da(mcu);

    case 0x14: return exec_instr_dec_a(mcu);
    case 0x15: return exec_instr_dec_iram(mcu);
    case 0x16: return exec_instr_dec_ind(mcu, 0);
    case 0x17: return exec_instr_dec_ind(mcu, 1);
    case 0x18: return exec_instr_dec_reg(mcu, 0);
    case 0x19: return exec_instr_dec_reg(mcu, 1);
    case 0x1A: return exec_instr_dec_reg(mcu, 2);
    case 0x1B: return exec_instr_dec_reg(mcu, 3);
    case 0x1C: return exec_instr_dec_reg(mcu, 4);
    case 0x1D: return exec_instr_dec_reg(mcu, 5);
    case 0x1E: return exec_instr_dec_reg(mcu, 6);
    case 0x1F: return exec_instr_dec_reg(mcu, 7);

    case 0x84: return exec_instr_div_ab(mcu);

    case 0xD5: return exec_instr_djnz_iram(mcu);
    case 0xD8: return exec_instr_djnz_reg(mcu, 0);
    case 0xD9: return exec_instr_djnz_reg(mcu, 1);
    case 0xDA: return exec_instr_djnz_reg(mcu, 2);
    case 0xDB: return exec_instr_djnz_reg(mcu, 3);
    case 0xDC: return exec_instr_djnz_reg(mcu, 4);
    case 0xDD: return exec_instr_djnz_reg(mcu, 5);
    case 0xDE: return exec_instr_djnz_reg(mcu, 6);
    case 0xDF: return exec_instr_djnz_reg(mcu, 7);

    case 0x04: return exec_instr_inc_a(mcu);
    case 0x05: return exec_instr_inc_iram(mcu);
    case 0x06: return exec_instr_inc_ind(mcu, 0);
    case 0x07: return exec_instr_inc_ind(mcu, 1);
    case 0x08: return exec_instr_inc_reg(mcu, 0);
    case 0x09: return exec_instr_inc_reg(mcu, 1);
    case 0x0A: return exec_instr_inc_reg(mcu, 2);
    case 0x0B: return exec_instr_inc_reg(mcu, 3);
    case 0x0C: return exec_instr_inc_reg(mcu, 4);
    case 0x0D: return exec_instr_inc_reg(mcu, 5);
    case 0x0E: return exec_instr_inc_reg(mcu, 6);
    case 0x0F: return exec_instr_inc_reg(mcu, 7);
    case 0xA3: return exec_instr_inc_dptr(mcu);

    case 0x20: return exec_instr_jb_bit(mcu);
    case 0x10: return exec_instr_jbc_bit(mcu);
    case 0x40: return exec_instr_jc(mcu);
    case 0x73: return exec_instr_jmp_a_dptr(mcu);
    case 0x30: return exec_instr_jnb_bit(mcu);
    case 0x50: return exec_instr_jnc(mcu);
    case 0x70: return exec_instr_jnz(mcu);
    case 0x60: return exec_instr_jz(mcu);
    
    case 0x12: return exec_instr_lcall(mcu);
    case 0x02: return exec_instr_ljmp(mcu);

    case 0x76: return exec_instr_mov_ind_data(mcu, 0);
    case 0x77: return exec_instr_mov_ind_data(mcu, 1);
    case 0xF6: return exec_instr_mov_ind_a(mcu, 0);
    case 0xF7: return exec_instr_mov_ind_a(mcu, 1);
    case 0xA6: return exec_instr_mov_ind_iram(mcu, 0);
    case 0xA7: return exec_instr_mov_ind_iram(mcu, 1);
    case 0x74: return exec_instr_mov_a_data(mcu);
    case 0xE6: return exec_instr_mov_a_ind(mcu, 0);
    case 0xE7: return exec_instr_mov_a_ind(mcu, 1);
    case 0xE8: return exec_instr_mov_a_reg(mcu, 0);
    case 0xE9: return exec_instr_mov_a_reg(mcu, 1);
    case 0xEA: return exec_instr_mov_a_reg(mcu, 2);
    case 0xEB: return exec_instr_mov_a_reg(mcu, 3);
    case 0xEC: return exec_instr_mov_a_reg(mcu, 4);
    case 0xED: return exec_instr_mov_a_reg(mcu, 5);
    case 0xEE: return exec_instr_mov_a_reg(mcu, 6);
    case 0xEF: return exec_instr_mov_a_reg(mcu, 7);
    case 0xE5: return exec_instr_mov_a_iram(mcu);
    case 0xA2: return exec_instr_mov_c_bit(mcu);
    case 0x90: return exec_instr_mov_dptr_data(mcu);
    case 0x78: return exec_instr_mov_reg_data(mcu, 0);
    case 0x79: return exec_instr_mov_reg_data(mcu, 1);
    case 0x7A: return exec_instr_mov_reg_data(mcu, 2);
    case 0x7B: return exec_instr_mov_reg_data(mcu, 3);
    case 0x7C: return exec_instr_mov_reg_data(mcu, 4);
    case 0x7D: return exec_instr_mov_reg_data(mcu, 5);
    case 0x7E: return exec_instr_mov_reg_data(mcu, 6);
    case 0x7F: return exec_instr_mov_reg_data(mcu, 7);
    case 0xF8: return exec_instr_mov_reg_a(mcu, 0);
    case 0xF9: return exec_instr_mov_reg_a(mcu, 1);
    case 0xFA: return exec_instr_mov_reg_a(mcu, 2);
    case 0xFB: return exec_instr_mov_reg_a(mcu, 3);
    case 0xFC: return exec_instr_mov_reg_a(mcu, 4);
    case 0xFD: return exec_instr_mov_reg_a(mcu, 5);
    case 0xFE: return exec_instr_mov_reg_a(mcu, 6);
    case 0xFF: return exec_instr_mov_reg_a(mcu, 7);
    case 0xA8: return exec_instr_mov_reg_iram(mcu, 0);
    case 0xA9: return exec_instr_mov_reg_iram(mcu, 1);
    case 0xAA: return exec_instr_mov_reg_iram(mcu, 2);
    case 0xAB: return exec_instr_mov_reg_iram(mcu, 3);
    case 0xAC: return exec_instr_mov_reg_iram(mcu, 4);
    case 0xAD: return exec_instr_mov_reg_iram(mcu, 5);
    case 0xAE: return exec_instr_mov_reg_iram(mcu, 6);
    case 0xAF: return exec_instr_mov_reg_iram(mcu, 7);
    case 0x92: return exec_instr_mov_bit_c(mcu);
    case 0x75: return exec_instr_mov_iram_data(mcu);
    case 0x86: return exec_instr_mov_iram_ind(mcu, 0);
    case 0x87: return exec_instr_mov_iram_ind(mcu, 1);
    case 0x88: return exec_instr_mov_iram_reg(mcu, 0);
    case 0x89: return exec_instr_mov_iram_reg(mcu, 1);
    case 0x8A: return exec_instr_mov_iram_reg(mcu, 2);
    case 0x8B: return exec_instr_mov_iram_reg(mcu, 3);
    case 0x8C: return exec_instr_mov_iram_reg(mcu, 4);
    case 0x8D: return exec_instr_mov_iram_reg(mcu, 5);
    case 0x8E: return exec_instr_mov_iram_reg(mcu, 6);
    case 0x8F: return exec_instr_mov_iram_reg(mcu, 7);
    case 0xF5: return exec_instr_mov_iram_a(mcu);
    case 0x85: return exec_instr_mov_iram_iram(mcu);

    case 0x93: return exec_instr_movc_a_dptr(mcu); // MOVC A,@A+DPTR
    case 0x83: return exec_instr_movc_a_pc(mcu); // MOVC A,@A+PC

    case 0xF0: return exec_instr_movx_dptr_a(mcu);   // MOVX @DPTR,A
    case 0xF2: return exec_instr_movx_reg_a(mcu, 0); // MOVX @R0,A
    case 0xF3: return exec_instr_movx_reg_a(mcu, 1); // MOVX @R1,A
    case 0xE0: return exec_instr_movx_a_dptr(mcu);   // MOVX A, @DPTR
    case 0xE2: return exec_instr_movx_a_reg(mcu, 0); // MOVX A,@R0
    case 0xE3: return exec_instr_movx_a_reg(mcu, 1); // MOVX A,@R1

    case 0xA4: return exec_instr_mul_ab(mcu);
    case 0x00: return exec_instr_nop(mcu);

    case 0x42: return exec_instr_orl_iram_a(mcu);
    case 0x43: return exec_instr_orl_iram_data(mcu);
    case 0x44: return exec_instr_orl_a_data(mcu);
    case 0x45: return exec_instr_orl_a_iram(mcu);
    case 0x46: return exec_instr_orl_a_ind(mcu, 0);
    case 0x47: return exec_instr_orl_a_ind(mcu, 1);
    case 0x48: return exec_instr_orl_a_reg(mcu, 0);
    case 0x49: return exec_instr_orl_a_reg(mcu, 1);
    case 0x4A: return exec_instr_orl_a_reg(mcu, 2);
    case 0x4B: return exec_instr_orl_a_reg(mcu, 3);
    case 0x4C: return exec_instr_orl_a_reg(mcu, 4);
    case 0x4D: return exec_instr_orl_a_reg(mcu, 5);
    case 0x4E: return exec_instr_orl_a_reg(mcu, 6);
    case 0x4F: return exec_instr_orl_a_reg(mcu, 7);
    case 0x72: return exec_instr_orl_c_bit(mcu, 0);
    case 0xA0: return exec_instr_orl_c_bit(mcu, 1);

    case 0xD0: return exec_instr_pop_iram(mcu);
    case 0xC0: return exec_instr_push_iram(mcu);
    case 0x22: return exec_instr_ret(mcu);
    case 0x32: return exec_instr_reti(mcu);
    case 0x23: return exec_instr_rl_a(mcu);
    case 0x33: return exec_instr_rlc_a(mcu);
    case 0x03: return exec_instr_rr_a(mcu);
    case 0x13: return exec_instr_rrc_a(mcu);

    case 0xD3: return exec_instr_setb_c(mcu);
    case 0xD2: return exec_instr_setb_bit(mcu);

    case 0x80: return exec_instr_sjmp(mcu);

    case 0x94: return exec_instr_subb_a_data(mcu);
    case 0x95: return exec_instr_subb_a_iram(mcu);
    case 0x96: return exec_instr_subb_a_ind(mcu, 0);
    case 0x97: return exec_instr_subb_a_ind(mcu, 1);
    case 0x98: return exec_instr_subb_a_reg(mcu, 0);
    case 0x99: return exec_instr_subb_a_reg(mcu, 1);
    case 0x9A: return exec_instr_subb_a_reg(mcu, 2);
    case 0x9B: return exec_instr_subb_a_reg(mcu, 3);
    case 0x9C: return exec_instr_subb_a_reg(mcu, 4);
    case 0x9D: return exec_instr_subb_a_reg(mcu, 5);
    case 0x9E: return exec_instr_subb_a_reg(mcu, 6);
    case 0x9F: return exec_instr_subb_a_reg(mcu, 7);

    case 0xC4: return exec_instr_swap_a(mcu);

    case 0xC6: return exec_instr_xch_a_ind(mcu, 0);
    case 0xC7: return exec_instr_xch_a_ind(mcu, 1);
    case 0xC8: return exec_instr_xch_a_reg(mcu, 0);
    case 0xC9: return exec_instr_xch_a_reg(mcu, 1);
    case 0xCA: return exec_instr_xch_a_reg(mcu, 2);
    case 0xCB: return exec_instr_xch_a_reg(mcu, 3);
    case 0xCC: return exec_instr_xch_a_reg(mcu, 4);
    case 0xCD: return exec_instr_xch_a_reg(mcu, 5);
    case 0xCE: return exec_instr_xch_a_reg(mcu, 6);
    case 0xCF: return exec_instr_xch_a_reg(mcu, 7);
    case 0xC5: return exec_instr_xch_a_iram(mcu);

    case 0xD6: return exec_instr_xchd_a_ind(mcu, 0);
    case 0xD7: return exec_instr_xchd_a_ind(mcu, 1);

    case 0x62: return exec_instr_xrl_iram_a(mcu);
    case 0x63: return exec_instr_xrl_iram_data(mcu);
    case 0x64: return exec_instr_xrl_a_data(mcu);
    case 0x65: return exec_instr_xrl_a_iram(mcu);
    case 0x66: return exec_instr_xrl_a_ind(mcu, 0);
    case 0x67: return exec_instr_xrl_a_ind(mcu, 1);
    case 0x68: return exec_instr_xrl_a_reg(mcu, 0);
    case 0x69: return exec_instr_xrl_a_reg(mcu, 1);
    case 0x6A: return exec_instr_xrl_a_reg(mcu, 2);
    case 0x6B: return exec_instr_xrl_a_reg(mcu, 3);
    case 0x6C: return exec_instr_xrl_a_reg(mcu, 4);
    case 0x6D: return exec_instr_xrl_a_reg(mcu, 5);
    case 0x6E: return exec_instr_xrl_a_reg(mcu, 6);
    case 0x6F: return exec_instr_xrl_a_reg(mcu, 7);
    }

    fprintf(stderr, "Unhandled instruction %02X\n", opcode);
    abort();
}