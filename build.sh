#!/bin/sh
gcc mcu.c -shared -O3 -fPIC -Wall -Wextra -o mcu_native.so

