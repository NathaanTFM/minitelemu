#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <SDL2/SDL.h>
#include <stdint.h>
#include <stdio.h>

// https://entropie.org/3615/wp-content/uploads/2020/08/DWOEDv0.jpg
static uint8_t get_code_from_key(SDL_Keycode sym) {
    switch (sym) {
    case SDLK_a: return 0x39;
    case SDLK_z: return 0x29;
    case SDLK_e: return 0x25;
    case SDLK_r: return 0x35;
    case SDLK_t: return 0x15;
    case SDLK_y: return 0x45;
    case SDLK_u: return 0xB9;
    case SDLK_i: return 0xC9;
    case SDLK_o: return 0xD9;
    case SDLK_p: return 0xE9;
    case SDLK_q: return 0x3A;
    case SDLK_s: return 0x2A;
    case SDLK_d: return 0x26;
    case SDLK_f: return 0x36;
    case SDLK_g: return 0x16;
    case SDLK_h: return 0x46;
    case SDLK_j: return 0xBA;
    case SDLK_k: return 0xCA;
    case SDLK_l: return 0xDA;
    case SDLK_m: return 0xEA;
    case SDLK_w: return 0x3F;
    case SDLK_x: return 0x2F;
    case SDLK_c: return 0x28;
    case SDLK_v: return 0x38;
    case SDLK_b: return 0x18;
    case SDLK_n: return 0x48;
    case SDLK_SPACE: return 0x4F;
    case SDLK_TAB: return 0x1A;
    case SDLK_RALT: return 0x1F;
    case SDLK_LCTRL: return 0x4A;
    case SDLK_LSHIFT: return 0xB0;
    case SDLK_RSHIFT: return 0xB0;
    case SDLK_ESCAPE: return 0x27;

    case SDLK_0: return 0xB8;
    case SDLK_1: return 0xE6;
    case SDLK_2: return 0xE8;
    case SDLK_3: return 0xEF;
    case SDLK_4: return 0xD6;
    case SDLK_5: return 0xD8;
    case SDLK_6: return 0xDF;
    case SDLK_7: return 0xC6;
    case SDLK_8: return 0xC8;
    case SDLK_9: return 0xCF;

    case SDLK_LEFT: return 0xC0;
    case SDLK_RIGHT: return 0xD0;
    case SDLK_UP: return 0x10;
    case SDLK_DOWN: return 0x40;

    case SDLK_F1: return 0x49;
    case SDLK_F2: return 0x19;
    case SDLK_F3: return 0x03;
    case SDLK_F4: return 0x02;
    case SDLK_F5: return 0x7C;
    case SDLK_F6: return 0x7B;
    case SDLK_F7: return 0x7E;
    case SDLK_F8: return 0x7D;
    
    case SDLK_RETURN: return 0x0E;

    default: {
        printf("Pressed key %d\n", sym);
        return 0;
    }
    }
}

#endif