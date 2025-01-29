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

static void increment_pointer_y(ef9345_state_t *gfx, bool aux_flag, bool incr_z) {
    int reg = aux_flag ? 4 : 6;

    // Attempt to increment Y then
    if ((gfx->reg[reg] & 31) >= 31) {
        gfx->reg[reg] -= (24 - 1);

        // Attempt to increment Z then
        if (!incr_z)
            return;

        // That's annoying because block numbers are bit swapped
        gfx->reg[reg+1] ^= ((gfx->reg[reg+1] >> 7) ^ 1) << 7;
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
    (void)incr_flag;
    printf("KRL command is not implemented\n");

    if (read_flag) {
        gfx->busy = 11.5 * 12;
    } else {
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

int ef9345_render(ef9345_state_t *gfx, uint8_t *dst) {
    // This is not accurate to the real chip
    // as we're recalculating the latched attributes every time we go through a line.
    // The real chip most likely had a cache of the current line it's drawing
    bool prev_double_width = false;
    uint8_t prev_character;

    bool latch_underline = false;
    bool latch_insert = false;
    bool latch_conceal = false;
    uint8_t latch_bgcolor = 0;

    int screen_y = gfx->screen_y;
    gfx->screen_y = (gfx->screen_y + 1);
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

    // cursor address
    uint16_t cursor = get_pointer_address(gfx, false);

    for (int x = 0; x < 40; x++) {
        uint16_t address = get_address(x, y, 0, 0);
        uint8_t short_a = ram_read(gfx, address);
        uint8_t short_b = ram_read(gfx, address + 0x400);

        // all attributes
        uint16_t character; // character code from charset
        uint8_t bgcolor = latch_bgcolor;
        uint8_t fgcolor;
        bool underline = latch_underline;
        bool insert = latch_insert;
        bool conceal = latch_conceal;
        bool flash;
        bool negative;
        bool double_width;
        bool double_height;

        if ((short_b & 0xE0) == 0x80) {
            // DELimiter
            character = 0x100;

            latch_underline = underline = !!(short_b & (1<<2));
            latch_insert = insert = !!(short_b & (1<<1));
            latch_conceal = conceal = !!(short_b & (1<<0));
            latch_bgcolor = bgcolor = (short_a >> 4) & 7;
            fgcolor = (short_a >> 0) & 7;

            double_width = false;
            double_height = false;
            negative = true;
            flash = false;

        } else if (short_a & (1<<7)) {
            // semi-graphic
            character = 0x100 | (short_b & 0x7F);

            latch_bgcolor = bgcolor = (short_a >> 4) & 7; // latched?
            flash = !!(short_a & (1<<3));
            fgcolor = short_a & 7;

            double_width = false;
            double_height = false;
            negative = false;
            latch_underline = underline = false; // TODO: check if this is latched

        } else {
            // alpha(numeric)
            character = short_b & 0x7F;

            negative = !!(short_a & (1<<6));
            double_width = !!(short_a & (1<<5));
            double_height = !!(short_a & (1<<4));
            flash = !!(short_a & (1<<3));
            fgcolor = short_a & 7;
        }

        if ((short_b & 0x80) && (short_b & 0x60)) {
            fprintf(stderr, "Custom character not implemented!");
            abort();
        }

        if (conceal || insert) {
            fprintf(stderr, "Conceal or insert not implemented!\n");
            abort();
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

            // Left pixel is LSB, right pixel is MSB
            for (int i = 0; i < 8; i += 2) {
                uint8_t color1;
                uint8_t color2;

                if (underline && draw_y == 9 && char_y == 9) {
                    color1 = color2 = fgcolor;

                } else if (double_width) {
                    color1 = color2 = (graphic & (1 << (i/2))) ? fgcolor : bgcolor;

                } else {
                    color1 = (graphic & (1 << i)) ? fgcolor : bgcolor;
                    color2 = (graphic & (2 << i)) ? fgcolor : bgcolor;
                }

                dst[draw_y * 160 + x * 4 + i/2] = (color1) | (color2 << 4);
            }
        }

        // For double width and height
        gfx->prev_double_height[x] = double_height;
        prev_double_width = double_width && !prev_double_width;
        prev_character = character;
    }

    return screen_y;
}

void ef9345_destroy(ef9345_state_t *gfx) {
    free(gfx);
}
