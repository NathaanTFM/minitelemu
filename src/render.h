#ifndef RENDER_H
#define RENDER_H

#include <SDL2/SDL.h>
#include "minitel.h"

static void render_screen(minitel_t *minitel, SDL_Texture *texture) {
    uint8_t *pixels;
    int pitch;
    SDL_LockTexture(texture, NULL, (void **)&pixels, &pitch);

    for (int i = 0; i < 25; i++) {
        static uint8_t line[10*80*8/2]; // 10 pixels height, 80 characters long, 8 pixels per character, 4/8 bits per color
        //static uint8_t linergb[20][80*8*2]; // 10 pixels height, 80 characters long, 8 pixels per character, 2 bytes per color
        int y = minitel_render(minitel, line);

        for (int r = 0; r < 10; r++) {
            // loop for each draw y
            int dy = y * 10 + r;

            for (int x = 0; x < 80 * 4; x++) {
                // loop for each pixel
                uint8_t color = line[r * 320 + x];

                for (int j = 0; j < 2; j++) {
                    // GB,AR
                    int dx = x * 2 + j;

                    uint8_t red = ((color >> 0) & 1) * 0xFF;
                    uint8_t green = ((color >> 1) & 1) * 0xFF;
                    uint8_t blue = ((color >> 2) & 1) * 0xFF;

                    //uint16_t abgr = 0xF000 | (((color >> 0) & 1) * 0xF00) | (((color >> 1) & 1) * 0xF0) | (((color >> 2) & 1) * 0xF);
                    //linergb[dy*2+0][dx*2+0] = linergb[dy*2+1][dx*2+0] = abgr;
                    //linergb[dy*2+0][dx*2+1] = linergb[dy*2+1][dx*2+1] = abgr >> 8;

                    pixels[(dy * pitch) + (dx * 4) + 0] = 0xFF;
                    pixels[(dy * pitch) + (dx * 4) + 1] = blue;
                    pixels[(dy * pitch) + (dx * 4) + 2] = green;
                    pixels[(dy * pitch) + (dx * 4) + 3] = red;
                    color >>= 4;
                }
                
            }
        }
    }

    SDL_UnlockTexture(texture);
}

#endif