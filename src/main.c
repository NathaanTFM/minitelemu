#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#define _GNU_SOURCE
#include "mcu.h"
#include "graphics.h"
#include "modem.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

#include <SDL2/SDL.h>

#define DIVIDER (60)

static const uint8_t KB_MAP_1[] = {4, 2, 3, 1, 14, 12, 13, 11};
static const uint8_t KB_MAP_2[] = {15, 10, 9, 8, 7, 6, 5, 0};

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

    default: return 0;
    }
}

static bool keydown[16][16];

#define NELEMS(tab) ((sizeof(*tab) / sizeof(tab)))

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

    if (port == 1) {
        int left = KB_MAP_1[value & 7];
        uint8_t mask = 0;

        for (int i = 0; i < 8; i++) {
            int right = KB_MAP_2[i];

            if (keydown[left][right]) {
                mask |= (1 << i);
            }
        }

        mcu_8051_set_port(mcu, 2, 0xFF & ~mask);
    }
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
        static uint8_t line[10*80*8/2]; // 10 pixels height, 80 characters long, 8 pixels per character, 4/8 bits per color
        static uint8_t linergb[20][80*8*2]; // 10 pixels height, 80 characters long, 8 pixels per character, 2 bytes per color
        int y = ef9345_render(gfx, line);

        for (int dy = 0; dy < 10; dy++) {
            // loop for each draw y

            for (int x = 0; x < 80 * 4; x++) {
                // loop for each pixel
                uint8_t color = line[dy * 320 + x];

                for (int j = 0; j < 2; j++) {
                    // GB,AR
                    int dx = x * 2 + j;

                    uint16_t abgr = 0xF000 | (((color >> 0) & 1) * 0xF00) | (((color >> 1) & 1) * 0xF0) | (((color >> 2) & 1) * 0xF);
                    linergb[dy*2+0][dx*2+0] = linergb[dy*2+1][dx*2+0] = abgr;
                    linergb[dy*2+0][dx*2+1] = linergb[dy*2+1][dx*2+1] = abgr >> 8;
                    color >>= 4;
                }
                
            }
        }

        SDL_Rect rect;
        rect.x = 0;
        rect.y = y * 20;
        rect.w = 80*8;
        rect.h = 20;
        int ret = SDL_UpdateTexture(texture, &rect, linergb, sizeof(*linergb));
        if (ret != 0) {
            puts(SDL_GetError());
        }
    }

    SDL_Rect rect;
    rect.x = 0;
    rect.y = 0;
    rect.w = 80*8;
    rect.h = 25*20;
    SDL_RenderClear(render);
    SDL_RenderCopy(render, texture, NULL, &rect);
    SDL_RenderPresent(render);
}

static void data_init() {
    FILE *f;

    f = fopen("data/rom.bin", "rb");
    if (f == NULL) { perror("fopen rom"); abort(); }
    if (fread(rom, 1, sizeof(rom), f) != sizeof(rom)) { perror("fread rom"); abort(); }
    fclose(f);

    f = fopen("data/charset.bin", "rb");
    if (f == NULL) { perror("fopen charset"); abort(); }
    if (fread(charset, 1, sizeof(charset), f) != sizeof(charset)) { perror("fread charset"); abort(); }
    fclose(f);

    f = fopen("data/teletel2.vdt", "rb");
    if (f == NULL) { perror("fopen page"); abort(); }
    fseek(f, 0, SEEK_END);
    pagesize = ftell(f);
    fseek(f, 0, SEEK_SET);

    page = malloc(pagesize + 0x40);
    if (page == NULL) { perror("malloc page"); abort(); }
    memset(page, 0, 0x40);
    pagesize = fread(page + 0x40, 1, pagesize, f) + 0x40;

    fclose(f);
}

static double diff_timespec(const struct timespec *time1, const struct timespec *time0) {
    return (time1->tv_sec - time0->tv_sec) * 1000000.0 + (time1->tv_nsec - time0->tv_nsec) / 1000.0;
}

static uint32_t loop_func() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            exit(0);
        } else if ((event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) && !event.key.repeat) {
            SDL_Keycode sym = event.key.keysym.sym;
            uint8_t code = get_code_from_key(sym);

            if (code != 0) {
                keydown[((code >> 4) & 0xF)][(code & 0xF)] = (event.type == SDL_KEYDOWN);
            }
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
    return cycles;
}

#ifdef __EMSCRIPTEN__
static void em_loop_func() {
    struct timespec t1, t2;
    clock_gettime(CLOCK_MONOTONIC, &t1);
    double cycles = loop_func();
    clock_gettime(CLOCK_MONOTONIC, &t2);

    double diff = diff_timespec(&t2, &t1);
    double freq1_mhz = (cycles * 12) / diff;
    printf("Freq: %.3f MHz   ", freq1_mhz);
    printf("Factor: %.1f%%\n", (100 * freq1_mhz) / 11.059200);
}

#include "rom_bin.h"
#include "charset_bin.h"
#include "page_vdt.h"

#define MIN(a, b) ((a) <= (b) ? (a) : (b))

static void em_data_init() {
    memcpy(rom, rom_bin, MIN(rom_bin_len, sizeof(rom)));
    memcpy(charset, charset_bin, MIN(charset_bin_len, sizeof(charset)));
    page = (uint8_t *)page_vdt;
    pagesize = page_vdt_len;
}
#endif

int main(void) {
#ifdef __EMSCRIPTEN__
    em_data_init();
#else
    data_init();
#endif
    modem_init();

    SDL_Init(SDL_INIT_VIDEO);
    SDL_StopTextInput();

    window = SDL_CreateWindow("EF9345", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 80 * 8, 10 * 25 * 2, SDL_WINDOW_RESIZABLE);
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
    SDL_RenderSetLogicalSize(render, 80 * 8, 10 * 25 * 2);

    texture = SDL_CreateTexture(render, SDL_PIXELFORMAT_ARGB4444, SDL_TEXTUREACCESS_STREAMING, 80 * 8, 10 * 25 * 2);
    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);

    mcu = mcu_8051_init(&config);
    gfx = ef9345_init(vram, 0x1FFF, charset);
    modem_set_page(page, pagesize);

#ifdef __EMSCRIPTEN__
    emscripten_set_main_loop(em_loop_func, DIVIDER, true);
#else
    struct timespec t1, t2, t3;

    double freq1_avg = 0, freq2_avg = 0;
    int iterations = 0;

    for (;;) {
        t3 = t1;
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double cycles = loop_func();
        clock_gettime(CLOCK_MONOTONIC, &t2);

        uint64_t delay = ((double)cycles / (11.059200 / 12));
        double diff = diff_timespec(&t2, &t1);

        if (delay >= diff) {
            usleep(delay - diff);
        }

        clock_gettime(CLOCK_MONOTONIC, &t3);
        double diff2 = diff_timespec(&t3, &t1);
        
        double freq1_mhz = (cycles * 12) / diff;

        if (iterations != 0) {
            double freq2_mhz = (cycles * 12) / diff2;
            freq2_avg += freq2_mhz;
        }

        freq1_avg += freq1_mhz;
        iterations++;

        if ((iterations & 63) == 0) {
            printf("Freq avg: %.3f MHz (capped: %.3f MHz)\n", freq1_avg / iterations, freq2_avg / iterations);
        }

        /*
        printf("Freq: %.3f MHz   ", freq1_mhz);
        printf("Factor: %.1f%%   ", (100 * freq1_mhz) / 11.059200);
        printf("Capped: %.3f MHz\n", freq2_mhz);
        */
    }

    printf("Freq avg: %.3f MHz\n", freq1_avg / iterations);

#endif
}