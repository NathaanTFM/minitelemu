#define _GNU_SOURCE
#include "mcu.h"
#include "graphics.h"
#include "modem.h"

#include <stdio.h>
#include <time.h>

#include <SDL2/SDL.h>

static SDL_Window *window;
static SDL_Texture *texture;
static SDL_Renderer *render;

static uint8_t rom[0x2000];
static uint8_t charset[0x2800];
static uint8_t vram[0x2000];

static uint8_t *page;
static size_t pagesize;

static mcu_8051_t *mcu;
static ef9345_state_t *gfx;

static uint8_t load_xram8(void *arg, uint8_t addr) {
    (void)arg;
    return ef9345_read(gfx, addr);
}

static uint8_t load_xram16(void *arg, uint16_t addr) {
    return load_xram8(arg, addr & 0xFF);
}

static void store_xram8(void *arg, uint8_t addr, uint8_t value) {
    (void)arg;
    return ef9345_write(gfx, addr, value);
}

static void store_xram16(void *arg, uint16_t addr, uint8_t value) {
    store_xram8(arg, addr & 0xFF, value);
}

static void set_port(void *arg, uint8_t port, uint8_t value) {
    (void)arg;
    (void)port;
    (void)value;
}

static mcu_8051_config_t config = {
    .rom = rom,
    .rom_mask = 0x1FFF,

    .load_xram8_cb = load_xram8,
    .load_xram16_cb = load_xram16,
    .store_xram8_cb = store_xram8,
    .store_xram16_cb = store_xram16,

    .set_port_cb = set_port,
};

static double diff_timespec(const struct timespec *time1, const struct timespec *time0) {
    return (time1->tv_sec - time0->tv_sec) + (time1->tv_nsec - time0->tv_nsec) / 1000000000.0;
}

int main(void) {
    modem_init();

    SDL_Init(SDL_INIT_VIDEO);

    window = SDL_CreateWindow("EF9345", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 8 * 40 * 3, 10 * 25 * 3, SDL_WINDOW_RESIZABLE);
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
    SDL_RenderSetLogicalSize(render, 8 * 40, 10 * 25);

    texture = SDL_CreateTexture(render, SDL_PIXELFORMAT_ARGB4444, SDL_TEXTUREACCESS_STREAMING, 8 * 40, 10 * 25);
    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);

    FILE *f;
    
    f = fopen("data/rom.bin", "rb");
    fread(rom, 1, sizeof(rom), f);
    fclose(f);

    f = fopen("data/charset.bin", "rb");
    fread(charset, 1, sizeof(charset), f);
    fclose(f);

    f = fopen("data/teletel2.vdt", "rb");
    fseek(f, 0, SEEK_END);
    pagesize = ftell(f);
    fseek(f, 0, SEEK_SET);

    page = malloc(pagesize + 0x40);
    memset(page, 0, 0x40);
    pagesize = fread(page + 0x40, 1, pagesize, f) + 0x40;

    fclose(f);

    modem_set_page(page, pagesize);

    mcu = mcu_8051_init(&config);
    gfx = ef9345_init(vram, 0x1FFF, charset);

    struct timespec t1, t2;

    double freq_avg = 0;

    #define DIVIDER (60)

    for (int gi = 0; gi < 2000; gi++) {
        clock_gettime(CLOCK_MONOTONIC, &t1);

        uint32_t cycles = 0;
        while (cycles < (11059200 / 12 / DIVIDER)) {
            uint32_t instr_cycles = mcu_8051_run_instr(mcu);
            cycles += instr_cycles;

            ef9345_update(gfx, instr_cycles * 12);
            modem_update(mcu, instr_cycles * 12);
        }

        //SDL_Delay(1000/DIVIDER);
        clock_gettime(CLOCK_MONOTONIC, &t2);

        double freq_mhz = (11.059200 * (1./DIVIDER)) / diff_timespec(&t2, &t1);
        freq_avg += freq_mhz;
        //printf("Freq: %.3f MHz   ", freq_mhz);
        //printf("Factor: %.1f%%\n", (100 * freq_mhz) / 11.059200);

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                return 0;
            }
        }

        for (int i = 0; i < 25*10; i++) {
            static uint8_t line[10][40*8/2];
            static uint8_t linergb[10][40*8*2];
            int y = ef9345_render(gfx, line);

            for (int dy = 0; dy < 10; dy++) {
                for (int x = 0; x < 40 * 8; x += 2) {
                    uint8_t color = line[dy][x/2];

                    for (int j = 0; j < 2; j++) {
                        // GB,AR
                        linergb[dy][(x+j) * 2 + 0] = (((color >> 1) & 1) * 0xF0) | (((color >> 2) & 1) * 0xF);
                        linergb[dy][(x+j) * 2 + 1] =  0xF0 | (((color >> 0) & 1) * 0xF);
                        color >>= 4;
                    }
                    
                }
            }

            SDL_Rect rect;
            rect.x = 0;
            rect.y = y * 10;
            rect.w = 40*8;
            rect.h = 10;
            int ret = SDL_UpdateTexture(texture, &rect, linergb, sizeof(*linergb));
            if (ret != 0) {
                puts(SDL_GetError());
            }
        }

        SDL_Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.w = 40*8;
        rect.h = 25*10;
        SDL_RenderClear(render);
        SDL_RenderCopy(render, texture, NULL, &rect);
        SDL_RenderPresent(render);

    }

    f = fopen("vram_dump.bin", "wb");
    fwrite(vram, 1, sizeof(vram), f);
    fclose(f);

    printf("Freq avg: %.3f MHz\n", freq_avg / 2000.0);
}