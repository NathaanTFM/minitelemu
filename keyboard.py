# wtf should i put here
from graphics import Graphics
import pygame


KEYBOARD_MAP_1 = [12, 14, 13, 15, 2, 4, 3, 5] # P2.0-P2.2: pins 3, 13, 15 -> B, A, 2~{C}
KEYBOARD_MAP_2 = [1, 6, 7, 8, 9, 10, 11, 16] # P1.0 to P1.7


PYGAME_SCAN_TO_KEY = {
    pygame.KSCAN_Q: 0x39,
    pygame.KSCAN_W: 0x29,
    pygame.KSCAN_E: 0x25,
    pygame.KSCAN_R: 0x35,
    pygame.KSCAN_T: 0x15,
    pygame.KSCAN_Y: 0x45,
    pygame.KSCAN_U: 0xB9,
    pygame.KSCAN_I: 0xC9,
    pygame.KSCAN_O: 0xD9,
    pygame.KSCAN_P: 0xE9,
    pygame.KSCAN_A: 0x3A,
    pygame.KSCAN_S: 0x2A,
    pygame.KSCAN_D: 0x26,
    pygame.KSCAN_F: 0x36,
    pygame.KSCAN_G: 0x16,
    pygame.KSCAN_H: 0x46,
    pygame.KSCAN_J: 0xBA,
    pygame.KSCAN_K: 0xCA,
    pygame.KSCAN_L: 0xDA,
    pygame.KSCAN_SEMICOLON: 0xEA, # that's "M"
    pygame.KSCAN_Z: 0x3F,
    pygame.KSCAN_X: 0x2F,
    pygame.KSCAN_C: 0x28,
    pygame.KSCAN_V: 0x38,
    pygame.KSCAN_B: 0x18,
    pygame.KSCAN_N: 0x48,
    pygame.KSCAN_SPACE: 0x4F,
    pygame.KSCAN_TAB: 0x1A,
    pygame.KSCAN_RALT: 0x1F,
    pygame.KSCAN_LCTRL: 0x4A,
    pygame.KSCAN_LSHIFT: 0xB0,
    pygame.KSCAN_RSHIFT: 0xB0,
    pygame.KSCAN_ESCAPE: 0x27,

    pygame.KSCAN_0: 0xB8,
    pygame.KSCAN_1: 0xE6,
    pygame.KSCAN_2: 0xE8,
    pygame.KSCAN_3: 0xEF,
    pygame.KSCAN_4: 0xD6,
    pygame.KSCAN_5: 0xD8,
    pygame.KSCAN_6: 0xDF,
    pygame.KSCAN_7: 0xC6,
    pygame.KSCAN_8: 0xC8,
    pygame.KSCAN_9: 0xCF,
}

class Keyboard:
    def __init__(self, gfx: Graphics):
        self.__gfx = gfx
        self.__xcl = 0


    def set_port1(self, value: int) -> None:
        # excl bits
        self.__xcl = value


    def get_port2(self) -> int:
        # Attempt to write "Guide", instead
        conn_fin = [1, 0xF]
        character = [3, 9] # [1, 0xF] # "Guide", per https://entropie.org/3615/wp-content/uploads/2020/08/DWOEDv0.jpg
        character = [16 - character[0], 16 - character[1]]

        # Pinouts are: 9 10 11 12 7 6 5 4 (from table)
        # Matching:   12 13 14 15 2 3 4 5 (kbmap2)

        # However, we're not using the normal order
        # Thus, our orders from 0 to 7: CAB -> CBA
        # 000 -> 000 -> 12
        # 001 -> 010 -> 14
        # 010 -> 001 -> 13
        # 011 -> 011 -> 15
        # 100 -> 100 -> 2
        # 101 -> 110 -> 4
        # 110 -> 101 -> 3
        # 111 -> 111 -> 5

        keys = self.__gfx.get_pressed()

        mask = 0

        for key in keys:
            mapped = PYGAME_SCAN_TO_KEY.get(key, None)
            if mapped is not None:
                left = 16 - ((mapped >> 4) & 0xF)
                right = 16 - (mapped & 0xF)
                if KEYBOARD_MAP_1[self.__xcl] == left:
                    mask |= (1 << KEYBOARD_MAP_2.index(right))

            else:
                print("Unmatched key: %r" % key)
                print("Possible values: %r" % [x for x in dir(pygame) if getattr(pygame, x) == key])

        return (0xFF & ~mask)