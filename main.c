#define _GNU_SOURCE
#include "mcu.h"
#include "graphics.h"
#include "modem.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>

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

static void render_screen() {
    for (int i = 0; i < 25; i++) {
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

static double diff_timespec(const struct timespec *time1, const struct timespec *time0) {
    return (time1->tv_sec - time0->tv_sec) * 1000000.0 + (time1->tv_nsec - time0->tv_nsec) / 1000.0;
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

    #define DIVIDER (60)

    struct timespec t1, t2, t3;

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &t1);

        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                return 0;
            }
        }


        uint32_t cycles = 0;
        
        while (cycles < (11059200 / 12 / DIVIDER)) {
            uint32_t instr_cycles = mcu_8051_run_instr(mcu);
            cycles += instr_cycles;

            ef9345_update(gfx, instr_cycles * 12);
            modem_update(mcu, instr_cycles * 12);
        }

        render_screen();

        clock_gettime(CLOCK_MONOTONIC, &t2);

        uint64_t delay = ((double)cycles / (11.059200 / 12));
        double diff = diff_timespec(&t2, &t1);

        if (delay >= diff) {
            usleep(delay - diff);
        }

        clock_gettime(CLOCK_MONOTONIC, &t3);
        double diff2 = diff_timespec(&t3, &t1);
        
        double freq1_mhz = (cycles * 12) / diff;
        double freq2_mhz = (cycles * 12) / diff2;
        printf("Freq: %.3f MHz   ", freq1_mhz);
        printf("Factor: %.1f%%   ", (100 * freq1_mhz) / (cycles * 12./1000000.));
        printf("Capped: %.3f MHz\n", freq2_mhz);
    }
}