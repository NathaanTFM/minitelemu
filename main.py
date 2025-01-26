from mcu import MCUPython, MCUNative, MCUExternalMemory, MCUInputOutput, MCUSerial
from graphics import Graphics
from modem import Modem
from keyboard import Keyboard
import time

class Minitel:
    def __init__(self, rom: bytes, native: bool):
        self.__modem = Modem()
        self.__graphics = Graphics()#
        self.__keyboard = Keyboard(self.__graphics)

        self.__io = MinitelIO(self.__modem, self.__keyboard)
        self.__xmem = MinitelXMem(self.__graphics)

        self.__mcu = (MCUNative if native else MCUPython)(rom, io=self.__io, xmem=self.__xmem)

        # we need to call update on modem every 768 cycles
        self.__cycles_modem = 0
        self.__cycles_overflow = 0


    def run(self, cycles: int) -> None:
        count = self.__cycles_overflow

        while count < cycles:
            instr_cycles = self.__mcu.run_instr()
            count += instr_cycles

            self.__cycles_modem += instr_cycles

            while self.__cycles_modem >= 768:
                self.__cycles_modem -= 768
                self.__modem.tick()

            self.__graphics.update(instr_cycles)

        self.__cycles_overflow = count - cycles


class MinitelIO(MCUInputOutput):
    def __init__(self, modem: Modem, keyboard: Keyboard):
        super().__init__()
        self.__modem = modem
        self.__keyboard = keyboard


    def get_port(self, port: int) -> int:
        if port == 0:
            return 0xFF # unused
        
        elif port == 1:
            value = 0b01110111
            value |= (self.__modem.get_p17() << 7)
            value |= (self.__modem.get_dp() << 3)
            return value

        elif port == 2:
            return self.__keyboard.get_port2()
        
        elif port == 3:
            value = 0b11110111
            value |= (self.__modem.get_p33() << 3)
            return value
        
        else:
            raise ValueError(port)
    
    
    def set_port(self, port: int, value: int) -> None:
        if port == 0:
            # inutilisé
            pass

        elif port == 1:
            self.__keyboard.set_port1(value & 0b111)
            self.__modem.set_p14((value >> 4) & 1)
            self.__modem.set_dpe((value >> 5) & 1)
            self.__modem.set_ed((value >> 6) & 1)

        elif port == 2:
            # inutilisé 
            pass

        elif port == 3:
            self.__modem.set_p34((value >> 4) & 1)
            self.__modem.set_p35((value >> 5) & 1)


class MinitelXMem(MCUExternalMemory):
    def __init__(self, graphics: Graphics):
        super().__init__()
        self.__gfx = graphics


    def load_addr8(self, addr: int) -> int:
        return self.__gfx.load_addr8(addr)
    
        
    def store_addr8(self, addr: int, value: int) -> None:
        self.__gfx.store_addr8(addr, value)

        
    def load_addr16(self, addr: int) -> int:
        return self.load_addr8(addr & 0xFF)
    
        
    def store_addr16(self, addr: int, value: int) -> None:
        return self.store_addr8(addr & 0xFF, value)
    

if __name__ == "__main__":
    ONE_SEC = 11059200 // 12

    with open("data/rom.bin", "rb") as f:
        rom = f.read()

    minitel = Minitel(rom, True)
    while 1:
        a = time.perf_counter_ns()
        minitel.run(ONE_SEC)
        b = time.perf_counter_ns()

        print("Freq: %.3f MHz" % ((11059200 / (b - a)) * 1000))


# This was used to test the native MCU
"""
import multiprocessing as mp
import multiprocessing.connection as mpconn

def procfunc(native: bool, pipe: mpconn.Connection):
    with open("data/rom.bin", "rb") as f:
        rom = f.read()

    minitel = Minitel(rom, native)
    while 1:
        msg = pipe.recv()
        minitel.run(msg)
        pipe.send(minitel._Minitel__mcu.dump_state())


if __name__ == "__main__":
    # this was used to
    ONE_SEC = 11059200 // 12

    pipe1 = mp.Pipe()
    pipe2 = mp.Pipe()

    proc1 = mp.Process(target=procfunc, args=(False, pipe1[0]))
    proc2 = mp.Process(target=procfunc, args=(True, pipe2[0]))

    proc1.start()
    proc2.start()

    cycles = ONE_SEC * 2

    pipe1[1].send(cycles)
    pipe2[1].send(cycles)

    state1 = pipe1[1].recv()
    state2 = pipe2[1].recv()
    
    if state1 != state2:
        raise Exception("already mismtach")

    while 1:
        count = ONE_SEC
        pipe1[1].send(count)#ONE_SEC * 0.2)
        pipe2[1].send(count)#ONE_SEC * 0.2)
        cycles += count

        state1 = pipe1[1].recv()
        state2 = pipe2[1].recv()

        print("Cycles", cycles)
        print("PYTHON", *state1.split("\n"))
        print("NATIVE", *state2.split("\n"))
        print()
        
        if state1 != state2:
            print("State mismatch!")
            break

    proc1.kill()
    proc2.kill()
"""