#ifndef MODEM_H
#define MODEM_H

#include "mcu.h"
#include <stdint.h>
#include <stddef.h>

void modem_update(mcu_8051_t *mcu, uint32_t cycles);
void modem_set_page(const uint8_t *page, size_t pagesize);
void modem_init();

#endif