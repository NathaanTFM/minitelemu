import ctypes
import pygame
import os
import sys

log = print
log = lambda *args, **kwargs: None

class Graphics:
    def __init__(self):
        self.__reg = [0] * 8
        self.__window = pygame.display.set_mode((8 * 40 * 2, 10 * 25 * 2))
        self.__surface = pygame.surface.Surface((8 * 40, 10 * 25))
        self.__busy = 0
        self.__refreshcnt = 0
        self.__pressed = set()
        self.__clock = pygame.time.Clock()
        self.__flashcnt = 0
        self.__flashval = 0

        # load charset
        tmp_surface = pygame.surface.Surface((8, 10))
        self.__charset_surface = pygame.image.load("data/charset.png").convert()
        self.__charset: list[pygame.mask.Mask] = []
        for i in range(1024):
            tmp_surface.blit(self.__charset_surface, (0, 0), ((i % 16) * 8, (i // 16) * 10, 8, 10))
            self.__charset.append(pygame.mask.from_threshold(tmp_surface, (255, 255, 255), threshold=(255, 255, 255, 255)))

        # registers
        self.__tgs = 0
        self.__mat = 0
        self.__pat = 0
        self.__dor = 0
        self.__ror = 0

        self.__memory = bytearray(0x2000)

        seen = [None for _ in range(0x2000)]
        for district in range(2):
            for block in range(4):
                for y in [0, 1, *range(8, 32)]:
                    for x in (range(32, 40) if y == 1 and block & 1 else range(40)):
                        addr = self.__get_address(district, block, x, y)
                        if seen[addr] is not None:
                            print("dup", (district, block, x, y), seen[addr], hex(addr))
                        seen[addr] = (district, block, x, y)

        assert all(x is not None for x in seen)


    def __write_memory(self, addr: int, val: int) -> None:
        self.__memory[addr & 0x1FFF] = val


    def __read_memory(self, addr: int) -> int:
        return self.__memory[addr & 0x1FFF]


    def __get_address(self, district: int, block: int, x: int, y: int) -> int:
        # Figure 11 of EF9345 datasheet
        address = ((district & 0b11) << 12) | ((block & 0b10) << 10) | (x & 0b111)
        if y >= 8:
            address |= (block & 0b1) << 10
            address |= (y & 0b111) << 5

            if ((x >> 5) & 1) == 0:
                address |= (y & 0b11000) << 5
                address |= (x & 0b11000)
            else:
                address |= (y & 0b11000)
        else:
            if ((y >> 0) & 1) == 0:
                address |= (block & 0b1) << 10
                address |= (x & 0b111000) << 2
            else:
                address |= (1 << 7) #  ??? "I" or "1"?
                address |= (~x & 0b110000) << 1
                if ((block >> 0) & 1) == 0:
                    address |= ((x & 0b1000) << 7) # ??? "I" or "1"?
                else:
                    address |= (1 << 10) # ??? "I" or "1"?
        
        return address


    def __get_address_homemade(self, district: int, block: int, x: int, y: int) -> int:
        # X is [0-40]
        # Y is [0-1, 8-31]
        # even block takes 1040 bytes
        # odd block takes 1008 bytes
        address = district * 4096
        address += (block >> 1) * 2048 # odd+even = 2048
        address += (block & 1) * 1040 # odd += 1040

        if y < 8:
            y &= 1
        if y >= 8:
            y -= 6
            
        if block & 1:
            if y == 0:
                address += x
            elif y == 1:
                address += 40
                address += x & 0b111
            else:
                address += 40 + 8
                address += (y - 2) * 40
                address += x

        else:
            address += y * 40
            address += x

        return address


    def __print_config(self) -> None:
        log("CONFIGURATION")
        log("  Origin row adress YOR = %d" % (self.__ror & 0b11111))

        # Z0 is implicit
        block_origin  = (self.__ror & 0b10000000) >> (5-1)
        block_origin |= (self.__ror & 0b01000000) >> (6-1)
        block_origin |= (self.__ror & 0b00100000) >> (4-1)
        log("  Block origin (even) = %d" % block_origin)
        log("  Service row select Y = %d" % ((self.__tgs >> 5) & 1))
        log("  Lines (525/625): %d" % (525 if self.__tgs & (1 << 0) else 625))
        log("  Interlaced: %s" % ("yes" if self.__tgs & (1 << 1) else "no"))
        log("  Horizontal resync: %s" % ("enabled" if self.__tgs & (1 << 2) else "disabled"))
        log("  Vertical resync: %s" % ("enabled" if self.__tgs & (1 << 3) else "disabled"))
        log("  Sync out pins: %s" % ("composite sync + phase comparator" if self.__tgs & (1 << 4) else "V sync + H sync"))
        log("  Char Code: %s" % {0b000: "40 CHAR LONG", 0b001: "40 CHAR VAR", 0b100: "40 CHAR SHORT", 0b011: "80 CHAR LONG", 0b010: "80 CHAR SHORT"}.get(((self.__tgs >> 6) & 0b11) | (self.__pat >> 5) & 0b100, "invalid"))
        log("  Service row: %s" % ("enabled" if self.__pat & (1 << 0) else "disabled"))
        log("  Upper bulk: %s" % ("enabled" if self.__pat & (1 << 1) else "disabled"))
        log("  Lower bulk: %s" % ("enabled" if self.__pat & (1 << 2) else "disabled"))
        log("  Conceal: %s" % ("enabled" if self.__pat & (1 << 3) else "disabled"))
        log("  Insert Mode: %s" % ["INLAY", "BOXING", "CHARACTER MARK", "ACTIVE AREA MARK"][(self.__pat >> 4) & 0b11])
        log("  Flash: %s" % ("enabled" if self.__pat & (1 << 6) else "disabled"))
        log("  Margin color: (R=%d, G=%d, B=%d)" % ((self.__mat >> 0) & 1, (self.__mat >> 1) & 1, (self.__mat >> 2) & 1))
        log("  Margin insert: %s" % ("yes" if self.__mat & (1 << 3) else "no"))
        log("  Cursor Display mode: %s %s" % (["FIXED", "FLASH"][(self.__mat >> 5) & 1], ["COMPLEMENTED", "UNDERLINED"][(self.__mat >> 4) & 1]))
        log("  Cursor display: %s" % ("enabled" if self.__mat & (1 << 6) else "disabled"))
        log("  Double height: %s" % ("yes" if self.__mat & (1 << 7) else "no"))
        log()


    def __get_pointer(self, aux_flag: bool) -> tuple[int, int, int, int]:
        if aux_flag:
            district = (self.__reg[6] >> 5) & 0b10
            district |= (self.__reg[4] >> 5) & 0b1
            block = (self.__reg[5] >> 5) & 0b10
            block |= (self.__reg[5] >> 7) & 0b1
            y = self.__reg[4] & 0b11111
            x = self.__reg[5] & 0b111111
            return (district, block, x, y)
        
        else:
            district = (self.__reg[6] >> 6) & 0b10
            district |= (self.__reg[6] >> 5) & 0b1
            block = (self.__reg[7] >> 5) & 0b10
            block |= (self.__reg[7] >> 7) & 0b1
            y = self.__reg[6] & 0b11111
            x = self.__reg[7] & 0b111111
            return (district, block, x, y)
        

    def __set_pointer(self, aux_flag: bool, district: int, block: int, x: int, y: int) -> None:
        if aux_flag:
            self.__reg[6] = (self.__reg[6] & ~0b01000000) | ((district & 0b10) << 5)
            self.__reg[5] = ((block & 0b1) << 7) | ((block & 0b10) << 5) | (x & 0b111111)
            self.__reg[4] = ((district & 0b1) << 5) | (y & 0b11111)
        
        else:
            self.__reg[6] = (self.__reg[6] & 0b01000000) | ((district & 0b10) << 6) | ((district & 0b1) << 5) | (y & 0b11111)
            self.__reg[7] = ((block & 0b1) << 7) | ((block & 0b10) << 5) | (x & 0b111111)


    def __increment_pointer(self, aux_flag: bool) -> tuple[int, int, int, int]:
        # return pointer before incrementation
        result = self.__get_pointer(aux_flag)
        district, block, x, y = result
        x += 1
        if x >= 40:
            y += 1

        x %= 40
        y %= 32

        if y == 1 and block & 1:
            x = 32 + (x % 8)

        self.__set_pointer(aux_flag, district, block, x, y)
        return result
    

    def __execute_ind(self, reg: int, read_flag: bool) -> None:
        log("[IND]", reg, read_flag, self.__reg)

        # Pointer in R6, R7
        # Data in R1

        if read_flag:
            self.__busy = round(3.5*12)
            if reg == 0:
                log("ROM access")
            elif reg == 1:
                self.__reg[1] = self.__tgs
            elif reg == 2:
                self.__reg[1] = self.__mat
            elif reg == 3:
                self.__reg[1] = self.__pat
            elif reg == 4:
                self.__reg[1] = self.__dor
            elif reg == 7:
                self.__reg[1] = self.__ror
            else:
                print("invalid reg")

        else:
            self.__busy = round(2*12)
            if reg == 0:
                log("ROM access")
            elif reg == 1:
                self.__tgs = self.__reg[1]

            elif reg == 2:
                self.__mat = self.__reg[1]

            elif reg == 3:
                self.__pat = self.__reg[1]

            elif reg == 4:
                self.__dor = self.__reg[1]

            elif reg == 7:
                self.__ror = self.__reg[1]

            else:
                print("invalid reg")

            self.__print_config()

    
    def __execute_krg(self, read_flag: bool, incr_flag: bool) -> None:
        log("[KRG]", read_flag, incr_flag, self.__reg)
        #self.emu.debuginfo()

        district, block, x, y = self.__get_pointer(False)
        if incr_flag:
            self.__increment_pointer(False)

        log("District: %d  Block: %d  X: %d  Y: %d" % (district, block, x, y))
        log("A*: 0x%02X  B*: 0x%02X" % (self.__reg[1], self.__reg[2]))

        if block & 1:
            print("--> WARNING: block is not even")
            block &= ~1

        log()

        if read_flag:
            self.__reg[1] = self.__read_memory(self.__get_address(district, block, x, y))
            self.__reg[2] = self.__read_memory(self.__get_address(district, block+1, x, y))
            self.__busy = round(7.5*12)
        else:
            self.__write_memory(self.__get_address(district, block, x, y), self.__reg[1])
            self.__write_memory(self.__get_address(district, block+1, x, y), self.__reg[2])
            self.__busy = round(5.5*12)
            
    
    def __execute_krl(self, read_flag: bool, incr_flag: bool) -> None:
        log("[KRL]", read_flag, incr_flag, self.__reg)
        # KRL is NOT implemented

        if read_flag:
            self.__busy = round(11.5*12)
        else:
            self.__busy = round(12.5*12)


    def __execute_mvt(self, ap_to_mp_flag: bool, no_stop_flag: bool) -> None:
        log("[MVT] %s %s" % ("AP->MP" if ap_to_mp_flag else "MP->AP", "no stop" if no_stop_flag else "stop at end of buffer"))
        log("MP: %r" % (self.__get_pointer(False),))
        log("AP: %r" % (self.__get_pointer(True),))
        log()
        self.__busy = round((2 + 12)*12) # TODO

        assert not no_stop_flag

        while 1:
            district_src, block_src, x_src, y_src = self.__increment_pointer(ap_to_mp_flag) # if True, ap is src
            district_dst, block_dst, x_dst, y_dst = self.__increment_pointer(not ap_to_mp_flag)

            for i in range(3):
                val = self.__read_memory(self.__get_address(district_src, block_src + i, x_src, y_src))
                self.__write_memory(self.__get_address(district_dst, block_dst + i, x_dst, y_dst), val)

            #log(src, dst)

            if x_src == 39 or x_dst == 39:
                break



    def __execute_oct(self, read_flag: bool, aux_flag: bool, incr_flag: bool) -> None:
        log("[OCT]", read_flag, aux_flag, incr_flag, self.__reg)
        district, block, x, y = self.__get_pointer(aux_flag)
        log("D:%02X B:%02X X:%02X Y:%02X" % (district, block, x, y))

        if incr_flag:
            # TODO:
            # "When MP is in use, an overflow yields to a Y incrementation"
            # implies that it's not the case with AP?
            self.__increment_pointer(aux_flag)

        if read_flag:
            self.__reg[1] = self.__read_memory(self.__get_address(district, block, x, y))
            self.__busy = round(4.5*12)
        else:
            self.__write_memory(self.__get_address(district, block, x, y), self.__reg[1])
            log("->", repr(chr(self.__reg[1])))
            self.__busy = round(4*12)

        log()


    def __execute(self):
        #log(format(self.__reg[0], "b").rjust(8, "0"))
        cmd = self.__reg[0]

        if   (cmd & 0b11110000) == 0b10000000:
            #raise NotImplementedError("IND")
            reg = cmd & 0b111
            read_flag = cmd & (1<<3)
            self.__execute_ind(reg, bool(read_flag))

        elif (cmd & 0b11110110) == 0b00000010:
            read_flag = cmd & (1<<3)
            incr_flag = cmd & (1<<0)
            self.__execute_krg(bool(read_flag), bool(incr_flag))

        elif (cmd & 0b11110110) == 0b01000000:
            raise NotImplementedError("KRC")
        
        elif (cmd & 0b11110110) == 0b01010000:
            read_flag = cmd & (1<<3)
            incr_flag = cmd & (1<<0)
            self.__execute_krl(bool(read_flag), bool(incr_flag))

        elif (cmd & 0b11110110) == 0b00100000:
            raise NotImplementedError("KRV")
        
        elif (cmd & 0b11111111) == 0b10010001:
            print("NOP")
            self.__busy = round(1*12)

        elif (cmd & 0b11110000) == 0b11110000:
            if (cmd & 0b101) & ((cmd >> 1) & 0b101):
                print("command is invalid")

            ap_to_mp_flag = cmd & (1 << 3)
            no_stop_flag = cmd & (1 << 1)
            self.__execute_mvt(bool(ap_to_mp_flag), bool(no_stop_flag))

        elif (cmd & 0b11110010) == 0b00110000:
            read_flag = cmd & (1<<3)
            aux_flag = cmd & (1<<2)
            incr_flag = cmd & (1<<0)
            self.__execute_oct(bool(read_flag), bool(aux_flag), bool(incr_flag))

        else:
            raise NotImplementedError(format(cmd, "b").rjust(8, "0"))

        
    def load_addr8(self, addr: int) -> int:
        reg = addr & 7
        if reg == 0:
            value = 0
            if self.__busy > 0:
                value |= 0x80

            return value
        else:
            return self.__reg[reg]
    
        
    def store_addr8(self, addr: int, value: int) -> None:
        self.__reg[addr & 7] = value
        if addr & 8:
            self.__execute()

        
    def load_addr16(self, addr: int) -> int:
        return self.load_addr8(addr & 0xFF)
        

    def store_addr16(self, addr: int, value: int) -> None:
        self.store_addr8(addr & 0xFF, value)

    
    def __draw_line(self, y: int, screenY: int) -> None:
        # so, if position is even and odd, the behaviour is different
        block = 0
        
        # Latched attributes
        underline = 0
        insert = 0
        conceal = 0
        bg_color = 0

        cursor = self.__get_pointer(False)

        for x in range(40):
            short_a = self.__read_memory(self.__get_address(0, 0, x, y))
            short_b = self.__read_memory(self.__get_address(0, 1, x, y))

            # short code conversion (manually)
            if short_b & 0b11100000 == 0b10000000:
                # DEL
                character = 0x100

                underline = (short_b >> 2) & 1
                insert = (short_b >> 1) & 1
                conceal = (short_b >> 0) & 1
                bg_color = (short_a >> 4) & 7
                fg_color = (short_a >> 0) & 7

                # now for the remaining attributes
                double_width = 0
                double_height = 0
                negative = 1
                flash = 0

            elif short_a & (1<<7):
                # semi-graphic
                character = 0x100 | (short_b & 0x7F)

                bg_color = (short_a >> 4) & 7
                flash = (short_a >> 3) & 1
                fg_color = (short_a >> 0) & 7

                # for the remaining attributes
                double_width = 0
                double_height = 0
                negative = 0
                underline = 0

                conceal = conceal
                insert = insert

            else:
                # alpha(numeric)
                character = (short_b & 0x7F)

                negative = (short_a >> 6) & 1
                double_width = (short_a >> 5) & 1
                double_height = (short_a >> 4) & 1
                flash = (short_a >> 3) & 1
                fg_color = (short_a >> 0) & 7

                # as for the remaining attributes
                underline = underline
                conceal = conceal
                insert = insert
                bg_color = bg_color

            if short_b & 0b10000000 and short_b & 0b01100000:
                # character is custom
                raise Exception("custom character not handled!")
            
            assert not (conceal or insert)
            
            # do the drawing
            color_to_rgb = lambda color: (((color >> 0) & 1) * 255, ((color >> 1) & 1) * 255, ((color >> 2) & 1) * 255)
            bg_rgb = color_to_rgb(fg_color if negative else bg_color)
            fg_rgb = color_to_rgb(bg_color if negative else fg_color)
            if flash and bool(self.__flashval & 2) ^ bool(negative):
                fg_rgb = bg_rgb

            is_cursor = self.__mat & (1<<6) and x == cursor[2] and y == cursor[3] and (not (self.__mat & (1<<5)) or (self.__flashval & 1))
            if is_cursor and not self.__mat & (1<<4):
                fg_rgb = tuple(c ^ 255 for c in fg_rgb)
                bg_rgb = tuple(c ^ 255 for c in bg_rgb)

            self.__charset[character].to_surface(self.__surface, dest=(x * 8, screenY * 10), setcolor=fg_rgb, unsetcolor=bg_rgb)
            if bool(underline) ^ (bool(is_cursor and (self.__mat & (1<<4)))):
                pygame.draw.rect(self.__surface, fg_rgb, (x * 8,  screenY * 10 + 9,  x * 8 + 7,  screenY * 10 + 9))
                
            #self.__surface.blit(, (x * 8, screenY * 10), ((character % 16) * 8, (character // 16) * 10, 8, 10))
            #self.__surface.set_at((x * 8, screenY * 10), (255, 0, 0))
            #if short_b & 0b11100000 == 0b10000000:
            #if y == 8 and x == 3:
            #    log(format(short_b, "b").rjust(8, "0") + " " + format(short_a, "b").rjust(8, "0"))
            #    self.__surface.set_at((x * 8, screenY * 10), (255, 0, 0))

        pygame.transform.scale(self.__surface, self.__window.get_size(), self.__window)
        #pygame.transform.scale2x(self.__surface, self.__window)

    def update(self, ticks: int) -> None:
        # NOTE: ticks is clocked at 11059200/12 Hz
        self.__refreshcnt -= ticks
        self.__flashcnt -= ticks
        while self.__flashcnt < 0:
            self.__flashval = (self.__flashval + 1) % 4
            self.__flashcnt += (11059200//24)

        self.__busy = max(self.__busy - ticks, 0)
    
        while self.__refreshcnt < 0:
            for evt in pygame.event.get():
                if evt.type == pygame.QUIT:
                    sys.exit(0)
                elif evt.type == pygame.KEYDOWN:
                    self.__pressed.add(evt.scancode)
                elif evt.type == pygame.KEYUP:
                    self.__pressed.discard(evt.scancode)

            # Service row
            self.__draw_line((self.__tgs >> 5) & 1, 0)

            for y in range(24):
                screen_y = y + 1
                origin_y = (self.__ror & 0b11111) + y
                if origin_y >= 32:
                    origin_y -= 24

                self.__draw_line(origin_y, screen_y)


            pygame.display.update()
            self.__refreshcnt += 30000
    
    def get_pressed(self) -> frozenset[int]:
        return frozenset(self.__pressed)