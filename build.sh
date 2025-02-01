#!/bin/bash
gcc -g src/main.c src/mcu.c src/mcu_instr.c src/graphics.c src/modem.c src/minitel.c -o main -Wall -Wextra -lSDL2 -O3 -Wno-unused
exit $?
