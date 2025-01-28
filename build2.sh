#!/bin/sh
gcc -g main.c mcu.c graphics.c modem.c -Ofast -Wall -Wextra -march=native -mtune=native -lSDL2 && ./a.out
