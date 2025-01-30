#include "graphics.h"

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>

struct ef9345_state_t {
    uint8_t *ram;
    uint16_t ram_mask;
    const uint8_t *charset;

    uint8_t reg[8];

    // Busy time in ticks
    uint32_t busy;

    // This is a timer with a low frequency (1Hz or 0.5Hz)
    uint32_t flash_ticks;
    uint8_t flash_state;

    // These are the internal registers
    uint8_t tgs;
    uint8_t mat;
    uint8_t pat;
    uint8_t dor;
    uint8_t ror;

    int screen_y;
    double prev_double_height[80];
    uint32_t latch_code;
};

// EF9345 pointers formats are:
//   [ / , / ,d'0,y'4,y'3,y'2,y'1,y'0] r4
//   [b'0,b'1,x'5,x'4,x'3,x'2,x'1,x'0] r5
//   [ d1,d'1, d0, y4, y3, y2, y1, y0] r6
//   [ b0, b1, x5, x4, x3, x2, x1, x0] r7


// Read from the RAM
static uint8_t ram_read(ef9345_state_t *gfx, uint16_t addr) {
    return gfx->ram[addr & gfx->ram_mask];
}

// Write into the RAM
static void ram_write(ef9345_state_t *gfx, uint16_t addr, uint8_t val) {
    gfx->ram[addr & gfx->ram_mask] = val;
}

// Resolves an address from a pointer
static uint16_t get_address(int x, int y, int block, int district) {
    uint16_t address = ((district & 3) << 12) | ((block & 3) << 10) | (x & 7);

    if (y & ~7) {
        address |= (y & 7) << 5;

        if (x & (1<<5)) {
            address |= (y & 0x18);

        } else {
            address |= (y & 0x18) << 5;
            address |= (x & 0x18);
        }

    } else {
        if (y & 1) {
            address |= (1 << 7);
            address |= (~x & 0x18);

            if (!(block & 1))
                address |= ((x & 8) << 7);

        } else {
            address |= (x & 0x38) << 2;
        }
    }
    
    return address;
}

// This resolves an address from a pointer
static uint16_t get_pointer_address(ef9345_state_t *gfx, bool aux_flag) {
    int x, y, district, block;

    if (aux_flag) {
        district = (gfx->reg[6] >> 5) & 2;
        district |= (gfx->reg[4] >> 5) & 1;
        block = (gfx->reg[5] >> 5) & 2;
        block |= (gfx->reg[5] >> 7) & 1;
        y = gfx->reg[4] & 31;
        x = gfx->reg[5] & 63;

    } else {
        district = (gfx->reg[6] >> 6) & 2;
        district |= (gfx->reg[6] >> 5) & 1;
        block = (gfx->reg[7] >> 5) & 2;
        block |= (gfx->reg[7] >> 7) & 1;
        y = gfx->reg[6] & 31;
        x = gfx->reg[7] & 63;
    }

    return get_address(x, y, block, district);
}

static void increment_pointer_z(ef9345_state_t *gfx, bool aux_flag) {
    int reg = aux_flag ? 4 : 6;

    gfx->reg[reg+1] ^= ((gfx->reg[reg+1] >> 7) ^ 1) << 7;
}

static void increment_pointer_y(ef9345_state_t *gfx, bool aux_flag, bool incr_z) {
    int reg = aux_flag ? 4 : 6;

    // Attempt to increment Y then
    if ((gfx->reg[reg] & 31) >= 31) {
        gfx->reg[reg] -= (24 - 1);

        // Attempt to increment Z then
        if (!incr_z)
            return;

        // That's annoying because block numbers are bit swapped
        increment_pointer_z(gfx, aux_flag);
    }
}

// Returns true if overflow occured
static bool increment_pointer(ef9345_state_t *gfx, bool aux_flag, bool incr_y, bool incr_z) {
    int reg = aux_flag ? 4 : 6;

    // Attempt to increment X
    if ((gfx->reg[reg+1] & 63) >= 39) {
        gfx->reg[reg+1] -= (40 - 1);

        if (incr_y) {
            increment_pointer_y(gfx, aux_flag, incr_z);
        } else if (incr_z) {
            increment_pointer_z(gfx, aux_flag);
        }

        return true;

    } else {
        gfx->reg[reg+1] += 1;
        return false;
    }
}

static void execute_ind(ef9345_state_t *gfx, int reg, bool read_flag) {
    if (read_flag) {
        gfx->busy = 3.5 * 12;

        switch (reg) {
            case 0: gfx->reg[1] = 0; break; // TODO
            case 1: gfx->reg[1] = gfx->tgs; break;
            case 2: gfx->reg[1] = gfx->mat; break;
            case 3: gfx->reg[1] = gfx->pat; break;
            case 4: gfx->reg[1] = gfx->dor; break;
            case 7: gfx->reg[1] = gfx->ror; break;
            default:
                fprintf(stderr, "Invalid reg %d\n", reg);
                abort();
        }

    } else {
        gfx->busy = 2 * 12;

        switch (reg) {
            case 0: fprintf(stderr, "Cannot write to ROM!\n"); break;
            case 1: gfx->tgs = gfx->reg[1]; break;
            case 2: gfx->mat = gfx->reg[1]; break;
            case 3: gfx->pat = gfx->reg[1]; break;
            case 4: gfx->dor = gfx->reg[1]; break;
            case 7: gfx->ror = gfx->reg[1]; break;
            default:
                fprintf(stderr, "Invalid reg %d\n", reg);
                abort();
        }
    }
}

static void execute_krg(ef9345_state_t *gfx, bool read_flag, bool incr_flag) {
    uint16_t address = get_pointer_address(gfx, false);
    if (incr_flag) 
        increment_pointer(gfx, false, false, false);
    
    if (address & 0x400) {
        fprintf(stderr, "Block is not even!\n");
        abort();
    }
    
    if (read_flag) {
        gfx->reg[1] = ram_read(gfx, address);
        gfx->reg[2] = ram_read(gfx, address + 0x400);
        gfx->busy = 7.5 * 12;

    } else {
        ram_write(gfx, address, gfx->reg[1]);
        ram_write(gfx, address + 0x400, gfx->reg[2]);
        gfx->busy = 5.5 * 12;
    }

}

static void execute_krl(ef9345_state_t *gfx, bool read_flag, bool incr_flag) {
    uint16_t address = get_pointer_address(gfx, false);
    if (incr_flag) {
        if (gfx->reg[7] & (1<<7)) {
            // Block is odd
            gfx->reg[7] &= ~(1<<7);
            increment_pointer(gfx, false, false, false);

        } else {
            gfx->reg[7] |= (1<<7);
        }
    }

    // KRL transfers 12 bits
    // If block address is even, it transfers A (R3) to MSB of B+2
    // If block address is odd,  it transfers A (R3) to LSB of B+1

    uint16_t address2 = address + 0x400 + ((address & 0x400) ^ 0x400);
    uint8_t tmp = ram_read(gfx, address2);
    
    if (read_flag) {
        if (address & 0x400) {
            // Odd block
            tmp = tmp & 0xF;

        } else {
            // Even block
            tmp = (tmp >> 4) & 0xF;
        }

        gfx->reg[1] = ram_read(gfx, address + 0x400);
        //gfx->reg[2] = (tmp >> (((address & 0x400) >> 8) ^ 4)) & 0xF;
        gfx->reg[2] = tmp;
        gfx->busy = 11.5 * 12;

    } else {
        if (address & 0x400) {
            // Odd block
            tmp = (tmp & ~0xF) | (gfx->reg[3] & 0xF);

        } else {
            // Even block
            tmp = (tmp & ~0xF0) | ((gfx->reg[3] << 4) & 0xF0);
        }

        ram_write(gfx, address, gfx->reg[1]);
        ram_write(gfx, address2, tmp);
        gfx->busy = 12.5 * 12;
    }
}

static void execute_nop(ef9345_state_t *gfx) {
    gfx->busy = 1 * 12;
}

static void execute_move_buffer(ef9345_state_t *gfx, bool ap_to_mp_flag, bool no_stop_flag, int blocks) {
    // It's the slowest command because it keeps converting from AP/MP to address
    // This should be optimized?

    // TODO: busy should be set to the correct value
    gfx->busy = (2 + 12) * 12;

    if (no_stop_flag) {
        fprintf(stderr, "no_stop_flag is not implemented\n");
        abort();
    }

    bool loop = true;
    while (loop) {
        uint16_t src_addr = get_pointer_address(gfx, ap_to_mp_flag);
        uint16_t dst_addr = get_pointer_address(gfx, !ap_to_mp_flag);

        for (int i = 0; i < blocks; i++) {
            ram_write(gfx, dst_addr + 0x400*i, ram_read(gfx, src_addr + 0x400*i));
        }

        loop = !increment_pointer(gfx, false, false, false);
        loop = !increment_pointer(gfx, true, false, false) && loop;
    }
}

static void execute_mvt(ef9345_state_t *gfx, bool ap_to_mp_flag, bool no_stop_flag) {
    execute_move_buffer(gfx, ap_to_mp_flag, no_stop_flag, 3);
}

static void execute_oct(ef9345_state_t *gfx, bool read_flag, bool aux_flag, bool incr_flag) {
    uint16_t address = get_pointer_address(gfx, aux_flag);
    if (incr_flag)
        increment_pointer(gfx, aux_flag, !aux_flag, false); // TODO - check if !aux_flag is valid
    
    if (read_flag) {
        gfx->reg[1] = ram_read(gfx, address);
        gfx->busy = 4.5 * 12;

    } else {
        ram_write(gfx, address, gfx->reg[1]);
        gfx->busy = 4 * 12;
    }
}

static void execute_command(ef9345_state_t *gfx) {
    uint8_t cmd = gfx->reg[0];

    if ((cmd & 0xF0) == 0x80) {
        int reg = cmd & 7;
        bool read_flag = !!(cmd & (1<<3));
        execute_ind(gfx, reg, read_flag);

    } else if ((cmd & 0xF6) == 0x2) {
        bool read_flag = !!(cmd & (1<<3));
        bool incr_flag = !!(cmd & (1<<0));
        execute_krg(gfx, read_flag, incr_flag);

    } else if ((cmd & 0xF6) == 0x50) {
        bool read_flag = !!(cmd & (1<<3));
        bool incr_flag = !!(cmd & (1<<0));
        execute_krl(gfx, read_flag, incr_flag);
        
    } else if (cmd == 0x91) {
        execute_nop(gfx);

    } else if ((cmd & 0xF0) == 0xF0) {
        if ((cmd & 5) & ((cmd >> 1) & 5)) {
            fprintf(stderr, "Invalid command! %02X\n", cmd);
            abort();
        }

        bool ap_to_mp_flag = !!(cmd & (1<<3));
        bool no_stop_flag = !!(cmd & (1<<1));
        execute_mvt(gfx, ap_to_mp_flag, no_stop_flag);

    } else if ((cmd & 0xF2) == 0x30) {
        bool read_flag = !!(cmd & (1<<3));
        bool aux_flag = !!(cmd & (1<<2));
        bool incr_flag = !!(cmd & (1<<0));
        execute_oct(gfx, read_flag, aux_flag, incr_flag);

    } else {
        fprintf(stderr, "Unknown or invalid command! %02X\n", cmd);
    }
}


ef9345_state_t *ef9345_init(uint8_t *ram, uint16_t ram_mask, const uint8_t *charset) {
    ef9345_state_t *gfx = (ef9345_state_t *)calloc(1, sizeof(ef9345_state_t));
    if (!gfx)
        return NULL;
    
    gfx->ram = ram;
    gfx->ram_mask = ram_mask;
    gfx->charset = charset;
    return gfx;
}

uint8_t ef9345_read(ef9345_state_t *gfx, uint8_t addr) {
    int reg = addr & 7;

    if (reg == 0) {
        return gfx->busy ? 0x80 : 0;
    } else {
        return gfx->reg[reg];
    }
}

void ef9345_write(ef9345_state_t *gfx, uint8_t addr, uint8_t value) {
    gfx->reg[addr & 7] = value;

    if (addr & 8) {
        execute_command(gfx);
    }
}


void ef9345_update(ef9345_state_t *gfx, uint32_t ticks) {
    // Check busy
    gfx->busy = (ticks >= gfx->busy) ? 0 : (gfx->busy - ticks);

    // Check flash timer
    if (ticks < gfx->flash_ticks) {
        gfx->flash_ticks -= ticks;

    } else {
        gfx->flash_ticks = (12000000 / 2) - (ticks - gfx->flash_ticks);
        gfx->flash_state = (gfx->flash_state + 1) & 3;
    }
}

static uint8_t get_block_origin(ef9345_state_t *gfx) {
    uint8_t bor = 0;
    bor |= (gfx->ror >> 4) & 2;
    bor |= (gfx->ror >> 3) & 4;
    bor |= (gfx->ror >> 5) & 8;
    return bor;
}

static uint32_t handle_40_short(ef9345_state_t *gfx, uint16_t address) {
    // Check Figure 20 from the datasheet
    // This function does exactly that
    uint8_t short_a = ram_read(gfx, address);
    uint8_t short_b = ram_read(gfx, address + 0x400);

    uint32_t code = gfx->latch_code;

    if ((short_b & 0xE0) == 0x80) {
        // DELimiter
        code = 0; // There is nothing to keep

        code |= (1<<13);
        code |= ((short_b >> 2) & 1) << 12; // U
        code |= ((short_b >> 0) & 1) << 10; // m
        code |= ((short_b >> 1) & 1) << 8; // i
        code |= (1<<7); // N
        code |= (short_a & 7) << 4; // C1
        code |= ((short_a >> 4) & 7) << 0; // C0

    } else if (short_a & (1<<7)) {
        // semi-graphic
        code &= 0x000500; // Only keep m i
        code |= (short_b & 0x7F) << 16; // Char code
        code |= ((short_b >> 7) & 1) << 15; // Bit set for custom character
        code |= (1<<13); // Bit set for semi-graphic
        code |= (short_a & 7) << 4; // C1
        code |= ((short_a >> 3) & 1) << 3; // F
        code |= ((short_a >> 4) & 7) << 0; // C0

    } else {
        // alpha(numeric)
        code &= 0x001507; // Only keep U m i C0

        code |= (short_b & 0x7F) << 16; // Char code
        code |= ((short_b >> 7) & 1) << 15; // Bit set for custom character
        code |= ((short_a >> 5) & 1) << 11; // L
        code |= ((short_a >> 4) & 1) << 9; // H
        code |= ((short_a >> 6) & 1) << 7; // N
        code |= (short_a & 7) << 4; // C1
        code |= ((short_a >> 3) & 1) << 3; // F
    }

    // Store the previous code
    gfx->latch_code = code;
    return code;
}

static uint32_t handle_40_var(ef9345_state_t *gfx, uint16_t address) {
    fprintf(stderr, "40 CHAR VAR is TODO\n");
    //abort();
    return 0x4141;
}

static uint32_t handle_40_long(ef9345_state_t *gfx, uint16_t address) {
    uint8_t code_c = ram_read(gfx, address);
    uint8_t code_b = ram_read(gfx, address + 0x400);
    uint8_t code_a = ram_read(gfx, address + 0x800);

    uint32_t code = ((uint32_t)code_c << 16) | ((uint32_t)code_b << 8) | ((uint32_t)code_a);
    return code;
}

static void render_40(ef9345_state_t *gfx, uint8_t *dst, int y, int screen_y) {
    // This is not accurate to the real chip as we're drawing character lines
    // and not pixel lines (I guess?)
    bool prev_double_width = false;
    uint8_t prev_character;

    // cursor address
    uint16_t cursor = get_pointer_address(gfx, false);

    // reset latch
    gfx->latch_code = 0;

    uint16_t bor = get_block_origin(gfx);

    for (int x = 0; x < 40; x++) {
        uint16_t address = get_address(x, y, bor & 3, (bor >> 2) & 3);
        uint32_t code;

        if (gfx->pat & (1<<7)) {
            if (gfx->tgs & (1<<6)) {
                fprintf(stderr, "TGS6 cannot be set if PAT7 is set\n");
                abort();
            }

            // 40 CHAR SHORT
            code = handle_40_short(gfx, address);

        } else {
            if (gfx->tgs & (1<<6)) {
                code = handle_40_long(gfx, address);
            } else {
                code = handle_40_var(gfx, address);
            }
        }

        // Parse attributes from code
        uint16_t character = ((code >> 16) & 0x7F) | (((code >> 12) & 15) << 7);

        if (((character >> 9) & 3) == 3) {
            fprintf(stderr, "Quadrichrome not implemented!\n");
            abort();
        }

        bool double_width = (code & (1<<11));
        bool conceal = (code & (1<<10));
        bool double_height = (code & (1<<9));
        bool insert = (code & (1<<8));
        bool negative = (code & (1<<7));
        uint8_t fgcolor = (code >> 4) & 7;
        bool flash = (code & (1<<3));
        uint8_t bgcolor = code & 7;

        // Underline attribute is only valid for some characters
        bool underline = false;

        if (((character >> 8) & 7) != 1) {
            underline = (character & (1<<7));
        }

        if (character & (1<<10)) {
            fprintf(stderr, "Custom character not implemented!");
            //abort();
        }

        if (conceal || insert) {
            fprintf(stderr, "Conceal or insert not implemented!\n");
            //abort();
        }

        if (negative) {
            // Swap the two colors if flash
            uint8_t tmp = bgcolor;
            bgcolor = fgcolor;
            fgcolor = tmp;
        }

        if (flash && (gfx->flash_state & 2) ^ (negative ? 2 : 0)) {
            fgcolor = bgcolor;
        }

        if ((gfx->mat & (1<<6)) && address == cursor && (!(gfx->mat & (1<<5)) || (gfx->flash_state & 1))) {
            if (gfx->mat & (1<<4)) {
                underline = !underline;

            } else {
                fgcolor ^= 7;
                bgcolor ^= 7;
            }
        }

        if (prev_double_width)
            character = prev_character;

        for (int draw_y = 0; draw_y < 10; draw_y++) {
            uint8_t graphic;

            int char_y = draw_y;
            if (double_height) {
                char_y /= 2;

                if (screen_y != 0 && gfx->prev_double_height[x]) {
                    char_y += 5;
                }
            }

            graphic = gfx->charset[character * 10 + char_y];
            if (prev_double_width)
                graphic >>= 4;

            for (int i = 0; i < 8; i++) {
                uint8_t color;

                if (underline && draw_y == 9 && char_y == 9) {
                    color = fgcolor;

                } else if (double_width) {
                    color = (graphic & (1 << (i/2))) ? fgcolor : bgcolor;

                } else {
                    color = (graphic & (1 << i)) ? fgcolor : bgcolor;
                }

                dst[draw_y * 320 + x * 8 + i] = (color) | (color << 4);
            }
        }

        // For double width and height
        gfx->prev_double_height[x] = double_height;
        prev_double_width = double_width && !prev_double_width;
        prev_character = character;
    }
}

static void render_80(ef9345_state_t *gfx, uint8_t *dst, int y, int screen_y) {
    // cursor address
    uint16_t cursor = get_pointer_address(gfx, false);

    // reset latch
    gfx->latch_code = 0;

    uint16_t bor = get_block_origin(gfx);

    for (int x = 0; x < 80; x += 2) {
        uint16_t address = get_address(x/2, y, bor & 3, (bor >> 2) & 3);
        uint32_t code;

        if (gfx->pat & (1<<7)) {
            fprintf(stderr, "PAT7 cannot be set if 80 Char mode\n");
            abort();
        }

        // C0 is left, C1 is right
        uint16_t code_c;
        code_c = ram_read(gfx, address);
        code_c |= (ram_read(gfx, address + 0x400) << 8);

        // MSB A is left, LSB A is right
        uint8_t code_a = 0;
        if (gfx->tgs & (1<<6)) {
            code_a = ram_read(gfx, address + 0x800);
        }

        // Parse attributes
        for (int xd = 0; xd < 2; xd++) {
            uint8_t character = code_c & 0xFF;

            bool negative = code_a & (1<<7);
            bool flash = code_a & (1<<6);
            bool underline = code_a & (1<<5);
            bool color_set = code_a & (1<<4);

            // I have no idea what color_set does
            if (color_set) {
                fprintf(stderr, "Color set not implemented!\n");
                //abort();
            }
            
            uint8_t fgcolor = 7;
            uint8_t bgcolor = 0;

            if (flash && (gfx->flash_state & 2) ^ (negative ? 2 : 0)) {
                fgcolor = bgcolor;
            }

            if ((gfx->mat & (1<<6)) && (address + 0x400*xd) == cursor && (!(gfx->mat & (1<<5)) || (gfx->flash_state & 1))) {
                if (gfx->mat & (1<<4)) {
                    underline = !underline;

                } else {
                    fgcolor ^= 7;
                    bgcolor ^= 7;
                }
            }

            for (int draw_y = 0; draw_y < 10; draw_y++) {
                uint8_t graphic;

                if (character & 0x80) {
                    fprintf(stderr, "Mosaic is not implemented!\n");
                    graphic = gfx->charset[draw_y]; // Dummy value

                } else {
                    graphic = gfx->charset[character * 10 + draw_y];
                }

                // Left pixel is LSB, right pixel is MSB
                for (int i = 0; i < 8; i += 2) {
                    uint8_t color1;
                    uint8_t color2;

                    if (underline && draw_y == 9 && draw_y == 9) {
                        color1 = color2 = fgcolor;

                    } else {
                        color1 = (graphic & (1 << i)) ? fgcolor : bgcolor;
                        color2 = (graphic & (2 << i)) ? fgcolor : bgcolor;
                    }

                    dst[draw_y * 320 + (x + xd) * 4 + i/2] = (color1) | (color2 << 4);
                }
            }
            
            // For next loop
            code_c >>= 8;
            code_a <<= 4; 
        }
        
    }
}

int ef9345_render(ef9345_state_t *gfx, uint8_t *dst) {
    int screen_y = gfx->screen_y;
    gfx->screen_y++;
    if (gfx->screen_y >= 25)
        gfx->screen_y = 0;

    // Calculate actual y position
    int y;
    if (screen_y == 0) {
        // Service row
        y = (gfx->tgs >> 5) & 1;

    } else {
        y = (gfx->ror & 31) + (screen_y - 1);
        if (y >= 32)
            y -= 24;
    }

    if (gfx->tgs & (1 << 7)) {
        render_80(gfx, dst, y, screen_y);

    } else {
        render_40(gfx, dst, y, screen_y);
    }

    return screen_y;
}

void ef9345_destroy(ef9345_state_t *gfx) {
    free(gfx);
}
