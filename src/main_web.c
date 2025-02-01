#define _GNU_SOURCE

#include <emscripten.h>

#include "minitel.h"
#include "keyboard.h"
#include "render.h"

#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>

#include <SDL2/SDL.h>
#include "rom_bin.h"
#include "charset_bin.h"

EM_JS(void, ws_init, (), {
    var ws = null;

    Module.ws_connect = () => {
        if (ws != null) {
            ws.close();
        }

        ws = new WebSocket("wss://go.minipavi.fr:8181");

        ws.onmessage = (event) => {
            console.log(event);
            console.log(event.data);
            Module._wasm_tx(stringToNewUTF8(event.data), lengthBytesUTF8(event.data));
        };
    };

    Module.ws_disconnect = () => {
        if (ws) {
            ws.close();
            ws = null;
        }
    };

    Module.ws_send_byte = (byte) => {
        if (ws) {
            ws.send(String.fromCharCode(byte));
        }
    };
});

EM_JS(void, ws_connect, (), {
    Module.ws_connect();
})

EM_JS(void, ws_disconnect, (), {
    Module.ws_disconnect();
})

EM_JS(void, ws_send_byte, (uint8_t byte), {
    Module.ws_send_byte(byte);
})

#define MIN(a, b) ((a) <= (b) ? (a) : (b))

static SDL_Window *window;
static SDL_Texture *texture;
static SDL_Renderer *render;

static minitel_t *minitel;

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

static struct timespec em_time;

static void em_loop_func() {
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);

    // calc time diff
    double timediff = diff_timespec(&time, &em_time);

    // 0.1 sec limit per frame
    if (timediff > 100000) {
        timediff = 100000;
    }

    check_events(minitel);
    minitel_run(minitel, timediff);
    render_screen(minitel, texture);
    SDL_RenderClear(render);
    SDL_RenderCopy(render, texture, NULL, NULL);
    SDL_RenderPresent(render);

    em_time = time;
}

static void on_modem_cb(void *arg, modem_event_t event, uint8_t byte) {
    printf("event %d\n", event);
    switch (event) {
        case MODEM_CONNECTED:
            printf("connected...\n");
            ws_connect();
            break;

        case MODEM_DISCONNECTED:
            printf("disconnected...\n");
            ws_disconnect();
            break;
        
        case MODEM_DATA:
            printf("data %02X...\n", byte);
            ws_send_byte(byte);
            break;
    }

}

EMSCRIPTEN_KEEPALIVE void wasm_tx(char *arg, unsigned int length) {
    minitel_modem_tx(minitel, (uint8_t *)arg, length);
}

int main(void) {
    clock_gettime(CLOCK_MONOTONIC, &em_time);
    ws_init();

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

    minitel = minitel_init((const uint8_t *)rom_bin, (const uint8_t *)charset_bin, on_modem_cb, NULL);
    emscripten_set_main_loop(em_loop_func, 0, true);
}