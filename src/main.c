#define _GNU_SOURCE

#include "minitel.h"
#include "keyboard.h"
#include "render.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

#include <SDL2/SDL.h>

#define DIVIDER (60)

static SDL_Window *window;
static SDL_Texture *texture;
static SDL_Renderer *render;

static uint8_t rom[0x2000];
static uint8_t charset[0x2800];

static uint8_t *page;
static size_t pagesize;

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

static void check_events(minitel_t *minitel) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            exit(0);
        } else if ((event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) && !event.key.repeat) {
            SDL_Keycode sym = event.key.keysym.sym;
            uint8_t code = get_code_from_key(sym);

            if (code != 0) {
                minitel_keyboard_set(minitel, code, event.type == SDL_KEYDOWN);
            }
        }
    }
}

static void on_modem_event(void *arg, modem_event_t event, uint8_t byte) {
    printf("byte %02X\n", byte);
}

int main(void) {
    data_init();

    SDL_Init(SDL_INIT_VIDEO);
    SDL_StopTextInput();

    window = SDL_CreateWindow("EF9345", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 80 * 8, 10 * 25 * 2, SDL_WINDOW_RESIZABLE);
    render = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
    SDL_RenderSetLogicalSize(render, 80 * 8, 10 * 25 * 2);

    texture = SDL_CreateTexture(render, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, 80 * 8, 10 * 25);
    SDL_SetRenderDrawColor(render, 0, 0, 0, 255);
    SDL_RenderClear(render);
    SDL_RenderPresent(render);

    minitel_t *minitel = minitel_init(rom, charset, on_modem_event, NULL);

    struct timespec t1, t2;

    double freq1_avg = 0, freq2_avg = 0;
    int iterations = 0;

    minitel_modem_tx(minitel, page, pagesize);

    for (;;) {
        clock_gettime(CLOCK_MONOTONIC, &t1);
        check_events(minitel);
        minitel_run(minitel, 1000000. / DIVIDER);
        render_screen(minitel, texture);
        SDL_RenderClear(render);
        SDL_RenderCopy(render, texture, NULL, NULL);
        SDL_RenderPresent(render);
        clock_gettime(CLOCK_MONOTONIC, &t2);

        double diff = diff_timespec(&t2, &t1);
        double delay = 1000000. / DIVIDER;
        if (delay >= diff) {
            usleep(delay - diff);
        }
    }
}