#ifndef GRAPHICS_H
#define GRAPHICS_H

#include <stdint.h>

typedef struct ef9345_state_t ef9345_state_t;

ef9345_state_t *ef9345_init(uint8_t *ram, uint16_t ram_mask, const uint8_t *charset);

// Write and read actions (such as movx of 8051)
uint8_t ef9345_read(ef9345_state_t *gfx, uint8_t addr);
void ef9345_write(ef9345_state_t *gfx, uint8_t addr, uint8_t value);

// The EF9345 should be clocked at 12 MHz.
// Therefore, a tick = 1/12'000'000 seconds
void ef9345_update(ef9345_state_t *gfx, uint32_t ticks);

// Returns Y of rendered line
int ef9345_render(ef9345_state_t *gfx, uint8_t *dst);

void ef9345_destroy(ef9345_state_t *gfx);

#endif