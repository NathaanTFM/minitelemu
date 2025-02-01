#ifndef MINITEL_H
#define MINITEL_H

#include <stdint.h>
#include <stdbool.h>

#include "modem.h"

typedef struct minitel_t minitel_t;

minitel_t *minitel_init(const uint8_t *rom, const uint8_t *charset, modem_callback_t modem_cb, void *cb_arg);
void minitel_run(minitel_t *minitel, uint32_t usec);
int minitel_render(minitel_t *minitel, uint8_t *buf);
void minitel_keyboard_set(minitel_t *minitel, uint8_t keycode, bool keydown);
bool minitel_modem_tx(minitel_t *minitel, uint8_t *buf, unsigned len);
void minitel_free(minitel_t *minitel);

#endif