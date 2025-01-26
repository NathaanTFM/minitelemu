class Modem:
    def __init__(self):
        self.__counter = 0
        self.__rxcur = 0b1111111111
        self.__txcur = 0b1111111111

        # outputs
        self.__p17 = 0 # "distance prêt"
        self.__dp = 0 # détection porteuse
        self.__p33 = 1  # données en sotie (RD?)

        # inputs
        self.__p14 = 1 # MLM?
        self.__dpe = 1
        self.__ed = 1
        self.__p34 = 1 # Pas modem, je crois (PT prise)
        self.__p35 = 1 # REV?

        # Manually send page through modem (temporary)
        with open("data/teletel2.vdt", "rb") as f:
            self.__outbuf = bytearray(f.read())

        #self.__outbuf = bytearray(b"\x1B\x3A\x69\x43" + b"".join(b"test %d\r\n" % i for i in range(200)))
        #self.__outbuf = bytearray(b'\x12Ca\x7Fb' + b'\x00' * 100 + b'\x0C')#a\x12C')
        #self.__outbuf = bytearray(b"ceci est un test !")
        self.__delay = 1200*2


    def __tick_75(self):
        if not self.__p14: # Si on est connecté (MLM=0)
            if self.__p35: # Inversé (75 bauds VERS minitel)
                self.__p33 = self.__txbit()
            else:
                self.__rxbit(self.__ed)


    def __tick_1200(self):
        if not self.__p14: # Si on est connecté (MLM=0)
            self.__p17 = 1
            self.__dp = 1

            if self.__p35: # Inversé (1200 bauds depuis minitel)
                self.__rxbit(self.__ed)
            else:
                self.__p33 = self.__txbit()
        else:
            self.__p17 = 0
            self.__dp = 0

            byte = self.__rxbit(self.__ed)
            if byte is not None: 
                byte &= 0x7F

                self.__txcur = 0x200 | (byte << 1) | (self.__calc_parity(byte) << 8)
                print("txcur", hex(self.__txcur))

            self.__p33 = self.__txbit()

            if self.__delay > 0:
                self.__delay -= 1
            else:
                if self.__outbuf:
                    byte = self.__outbuf.pop(0)
                    #print("TXCUR is NOW", repr(self.__txcur))
                    #print()
                    #print()
                    self.__txcur = 0x200 | (byte << 1) | (self.__calc_parity(byte) << 8)
                    self.__delay = 10


    def __calc_parity(self, v: int) -> int:
        v ^= v >> 4
        v &= 0xF
        return (0x6996 >> v) & 1


    def __rxbit(self, bit: int) -> int | None:
        self.__rxcur = (self.__rxcur >> 1) | (bit << 9)
        if (self.__rxcur & 0x201) == 0x200:
            print("rxcur", hex(self.__rxcur))
            # start et stop bit ok
            byte = (self.__rxcur >> 1) & 0x7F
            if ((self.__rxcur >> 8) & 1) ^ self.__calc_parity(byte):
                # erreur de parité
                print("parity error!")
                byte = ord("Z")
            
            # reset
            print("got byte from modem %02X" % byte)
            self.__rxcur = 0b1111111111
            return byte
        
        return None


    def __txbit(self) -> int:
        ret = self.__txcur & 1
        self.__txcur = (self.__txcur >> 1) | (1 << 9)
        return ret


    def tick(self):
        self.__counter += 1
        while self.__counter >= 16:
            self.__tick_75()
            self.__counter -= 16

        self.__tick_1200()


    def get_p17(self) -> int:
        # unk name yet
        return self.__p17

    def get_dp(self) -> int:
        return self.__dp
    
    def get_p33(self) -> int:
        # réception données modem "en coupure"
        return self.__p33
    
    def set_p14(self, value: int) -> None:
        self.__p14 = value

    def set_dpe(self, value: int) -> None:
        self.__dpe = value
    
    def set_ed(self, value: int) -> None:
        self.__ed = value
    
    def set_p34(self, value: int) -> None:
        self.__p34 = value
    
    def set_p35(self, value: int) -> None:
        self.__p35 = value
    