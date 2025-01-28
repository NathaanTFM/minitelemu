import operator
import ctypes
import os
import traceback

McuNativeGetPortCb = ctypes.CFUNCTYPE(ctypes.c_uint8, *(ctypes.c_void_p, ctypes.c_uint8))
McuNativeSetPortCb = ctypes.CFUNCTYPE(None, *(ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8))
McuNativeLoadXram8Cb = ctypes.CFUNCTYPE(ctypes.c_uint8, *(ctypes.c_void_p, ctypes.c_uint8))
McuNativeLoadXram16Cb = ctypes.CFUNCTYPE(ctypes.c_uint8, *(ctypes.c_void_p, ctypes.c_uint16))
McuNativeStoreXram8Cb = ctypes.CFUNCTYPE(None, *(ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8))
McuNativeStoreXram16Cb = ctypes.CFUNCTYPE(None, *(ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint8))

class McuNativeConfig(ctypes.Structure):
    _fields_ = [
        ("rom", ctypes.POINTER(ctypes.c_uint8)),
        ("rom_mask", ctypes.c_uint16),
        ("cb_arg", ctypes.c_void_p),
        ("get_port_cb", McuNativeGetPortCb),
        ("set_port_cb", McuNativeSetPortCb),
        ("load_xram8_cb", McuNativeLoadXram8Cb),
        ("load_xram16_cb", McuNativeLoadXram16Cb),
        ("store_xram8_cb", McuNativeStoreXram8Cb),
        ("store_xram16_cb", McuNativeStoreXram16Cb)
    ]

try:
    MCU_SO = ctypes.CDLL(os.path.abspath("mcu_native.so"))

except Exception:
    traceback.print_exc()
    MCU_SO = None

else:
    McuNativeInit = MCU_SO.mcu_8051_init
    McuNativeInit.argtypes = (ctypes.POINTER(McuNativeConfig),)
    McuNativeInit.restype = ctypes.c_void_p

    McuNativeSetPort = MCU_SO.mcu_8051_set_port
    McuNativeSetPort.argtypes = (ctypes.c_void_p, ctypes.c_uint8, ctypes.c_uint8)
    McuNativeSetPort.restype = None

    McuNativeRunInstr = MCU_SO.mcu_8051_run_instr
    McuNativeRunInstr.argtypes = (ctypes.c_void_p,)
    McuNativeRunInstr.restype = ctypes.c_uint32

    McuNativeDumpState = MCU_SO.mcu_8051_dump_state
    McuNativeDumpState.argtypes = (ctypes.c_void_p,)
    McuNativeDumpState.restype = ctypes.c_char_p


class MCUSerial:
    def __init__(self):
        pass


class MCUInputOutput:
    def __init__(self):
        pass


    def get_port(self, port: int) -> int:
        print("WARNING: get_port is NOT implemented")
        return 0
    

    def set_port(self, port: int, value: int) -> None:
        print("WARNING: set_port is NOT implemented")
        pass


class MCUExternalMemory:
    def __init__(self):
        pass


    def load_addr8(self, addr: int) -> int:
        print("WARNING: load_addr8 is NOT implemented")
        return 0
    
        
    def store_addr8(self, addr: int, value: int) -> None:
        print("WARNING: store_addr8 is NOT implemented")
        pass

        
    def load_addr16(self, addr: int) -> int:
        print("WARNING: load_addr16 is NOT implemented")
        return 0
    
        
    def store_addr16(self, addr: int, value: int) -> None:
        print("WARNING: store_addr16 is NOT implemented")
        pass


class MCU:
    def __init__(self, rom: bytes, serial: MCUSerial = None, io: MCUInputOutput = None, xmem: MCUExternalMemory = None):
        self.__cycles_overflow = 0
        
        
    def run(self, cycles: int) -> None:
        count = self.__cycles_overflow

        while count < cycles:
            count += self.run_instr()

        self.__cycles_overflow = cycles - count


    def run_instr(self) -> int:
        raise NotImplementedError("run_instr")
    

    def dump_state(self) -> None:
        raise NotImplementedError()
    

class MCUNative(MCU):
    def __init__(self, rom, serial = None, io = None, xmem = None):
        if MCU_SO is None:
            raise Exception("MCUNative is not available")
        
        super().__init__(rom, serial, io, xmem)
        self.__serial = serial
        self.__io = io
        self.__xmem = xmem

        self.__rombuf = ctypes.create_string_buffer(rom)

        self.__config = McuNativeConfig()
        self.__config.rom = ctypes.cast(self.__rombuf, ctypes.POINTER(ctypes.c_uint8))
        self.__config.rom_mask = 0x1FFF

        self.__config.get_port_cb = McuNativeGetPortCb(self.__get_port_cb)
        self.__config.set_port_cb = McuNativeSetPortCb(self.__set_port_cb)
        self.__config.load_xram8_cb = McuNativeLoadXram8Cb(self.__load_xram8_cb)
        self.__config.load_xram16_cb = McuNativeLoadXram16Cb(self.__load_xram16_cb)
        self.__config.store_xram8_cb = McuNativeStoreXram8Cb(self.__store_xram8_cb)
        self.__config.store_xram16_cb = McuNativeStoreXram16Cb(self.__store_xram16_cb)

        self.__handle = McuNativeInit(ctypes.byref(self.__config))

    def __get_port_cb(self, arg: ctypes.c_void_p, port: int) -> int:
        return self.__io.get_port(port)

    def __set_port_cb(self, arg: ctypes.c_void_p, port: int, value: int) -> None:
        self.__io.set_port(port, value)

    def __load_xram8_cb(self, arg: ctypes.c_void_p, addr: int) -> int:
        return self.__xmem.load_addr8(addr)

    def __load_xram16_cb(self, arg: ctypes.c_void_p, addr: int) -> int:
        return self.__xmem.load_addr16(addr)

    def __store_xram8_cb(self, arg: ctypes.c_void_p, addr: int, value: int) -> None:
        self.__xmem.store_addr8(addr, value)

    def __store_xram16_cb(self, arg: ctypes.c_void_p, addr: int, value: int) -> None:
        self.__xmem.store_addr16(addr, value)


    def run_instr(self) -> int:
        #McuNativeSetPort(self.__handle, 0, self.__io.get_port(0))
        #McuNativeSetPort(self.__handle, 1, self.__io.get_port(1))
        #McuNativeSetPort(self.__handle, 2, self.__io.get_port(2))
        #McuNativeSetPort(self.__handle, 3, self.__io.get_port(3))

        cycles = McuNativeRunInstr(self.__handle)
        return cycles
    

    def dump_state(self) -> None:
        result = McuNativeDumpState(self.__handle)
        return result.decode("ascii")


class MCUPython(MCU):
    def __init__(self, rom: bytes, serial: MCUSerial = None, io: MCUInputOutput = None, xmem: MCUExternalMemory = None) -> None:
        super().__init__(rom, serial, io, xmem) # hmm
        self.program = rom
        
        # 8 bit
        self.a = 0
        self.b = 0
        self.psw = 0
        self.ie = 0
        self.ip = 0
        self.scon = 0
        self.sbuf_in = 0
        self.sbuf_out = None # this one cannot be not read
        self.tcon = 0
        self.tmod = 0
        self.p0 = 0xFF # P0 output
        self.p1 = 0xFF # P1 output
        self.p2 = 0xFF # P2 output
        self.p3 = 0xFF # P3 output
        self.t2con = 0
        self.t2mod = 0
        
        # 16 bit
        self.pc = 0
        self.sp = 0
        self.dptr = 0
        self.rcap2 = 0
        self.timer0 = 0
        self.timer1 = 0
        self.timer2 = 0
        
        # 256 bytes ram
        self.ram = bytearray(256)
        
        # flags
        self.c_flag = 0
        self.ac_flag = 0
        self.ov_flag = 0
        
        # pending irq
        self.current_irq: list[int] = []

        # IO input
        self.p0_input = 0xFF
        self.p1_input = 0xFF
        self.p2_input = 0xFF
        self.p3_input = 0xFF
        self.p3_input_prev = self.p3_input

        # handlers
        self.serial = serial if serial is not None else MCUSerial()
        self.io = io if io is not None else MCUInputOutput()
        self.xmem = xmem if xmem is not None else MCUExternalMemory()


    def on_irq(self, dest: int, interrupt: int) -> bool:
        if interrupt in self.current_irq:
            return False
            
        if not (self.ie & (1 << 7)):
            # interrupts not enabled
            return False
        
        if self.ie & (1 << interrupt):
            # check irq priority TODO!
            self.current_irq.append(interrupt)
            self.call(dest)
            return True

        return False
    
        
    def reti(self) -> None:
        self.ret()
        irq = self.current_irq.pop()
        if irq == 1: # Timer 0
            self.tcon &= ~(1 << 5)

        elif irq == 3: # Timer 1
            self.tcon &= ~(1 << 7)
        

    def u8_to_s8(self, value: int) -> int:
        if value >= 128:
            return value - 256
        else:
            return value
        
        
    def load_code(self, adrs: int) -> int:
        return self.program[adrs % len(self.program)]
    
        
    def jump(self, target: int) -> None:
        self.pc = target

        
    def call(self, target: int) -> None:
        self.push_stack(self.pc & 0xFF)
        self.push_stack(self.pc >> 8)
        self.pc = target

        
    def ret(self) -> None:
        target = (self.pop_stack() << 8)
        target |= self.pop_stack()
        self.pc = target
        
        
    # load registers R0-R7
    def load_reg(self, index: int) -> int:
        bank = (self.psw >> 3) & 0x3
        return self.load_ram(bank * 0x8 + index)
    
        
    def store_reg(self, index: int, value: int) -> None:
        bank = (self.psw >> 3) & 0x3
        self.store_ram(bank * 0x8 + index, value)


    def calc_parity8(self, v: int) -> int:
        v ^= v >> 4
        v &= 0xF
        return (0x6996 >> v) & 1
        
        
    # load FSR
    def load_fsr(self, addr: int, to_write: bool) -> int:
        if addr == 0xE0: # ACC
            return self.a 
        
        elif addr == 0xF0: # B
            return self.b
        
        elif addr == 0x81: # SP
            return self.sp
        
        elif addr == 0xD0: # PSW
            value = self.psw & 0b00111000 # keep f0, rs1, rs0
            value |= self.c_flag << 7
            value |= self.ac_flag << 6
            value |= self.ov_flag << 2
            value |= self.calc_parity8(self.a)
            return value
        
        elif addr == 0x82: # DPL
            return self.dptr & 0xFF
        
        elif addr == 0x83: # DPH
            return self.dptr >> 8
        
        elif addr == 0xA8: # IE
            return self.ie
        
        elif addr == 0xB8: # IP
            return self.ip
        
        elif addr == 0x98: # SCON
            return self.scon
        
        elif addr == 0x99: # SBUF
            return self.sbuf_in
        
        elif addr == 0xCA: # RCAP2L
            return self.rcap2 & 0xFF
        
        elif addr == 0xCB: # RCAP2H
            return self.rcap2 >> 8
        
        elif addr == 0x88: # TCON
            return self.tcon
        
        elif addr == 0x89: # TMOD
            return self.tmod
        
        elif addr == 0x80: # P0
            if to_write: return self.p0
            self.p0_input = self.io.get_port(0)
            return self.p0_input & self.p0
        
        elif addr == 0x90: # P1
            if to_write: return self.p1
            self.p1_input = self.io.get_port(1)
            return self.p1_input & self.p1
        
        elif addr == 0xA0: # P2
            # P2: clavier entrée
            if to_write: return self.p2
            self.p2_input = self.io.get_port(2)
            return self.p2_input & self.p2
        
        elif addr == 0xB0: # P3
            if to_write: return self.p3
            #self.p3_input = self.io.get_port(3)
            return self.p3_input & self.p3
        
        elif addr == 0x8A: # TL0
            return self.timer0 & 0xFF
        
        elif addr == 0x8B: # TL1
            return self.timer1 & 0xFF
        
        elif addr == 0x8C: # TH0
            return self.timer0 >> 8
        
        elif addr == 0x8D: # TH1
            return self.timer1 >> 8
            
        elif addr == 0xCC: # TL2
            return self.timer2 & 0xFF
        
        elif addr == 0xCD: # TH2
            return self.timer2 >> 8
            
        elif addr == 0xC8: # T2CON
            return self.t2con
        
        elif addr == 0xC9: # T2MOD
            return self.t2mod

        else:
            raise NotImplementedError("load_fsr %02X (from %04X)" % (addr, self.pc))
        
        
    def store_fsr(self, addr: int, value: int) -> None:
        if addr == 0xE0: # ACC
            self.a = value

        elif addr == 0xF0: # B
            self.b = value

        elif addr == 0x81: # SP
            self.sp = value

        elif addr == 0xD0: # PSW
            self.c_flag = (value >> 7) & 1
            self.ac_flag = (value >> 6) & 1
            self.ov_flag = (value >> 2) & 1
            self.psw = value

        elif addr == 0x82: # DPL
            self.dptr = (self.dptr & 0xFF00) | value

        elif addr == 0x83: # DPH
            self.dptr = (self.dptr & 0xFF) | (value << 8)

        elif addr == 0xA8: # IE
            self.ie = value
            #print("IE", format(self.ie, "b").rjust(8, "0"))

        elif addr == 0xB8: # IP
            self.ip = value

        elif addr == 0x98: # SCON
            self.scon = value

        elif addr == 0x99: # SBUF
            self.sbuf_out = value

        elif addr == 0xCA: # RCAP2L
            self.rcap2 = (self.rcap2 & 0xFF00) | value

        elif addr == 0xCB: # RCAP2H
            self.rcap2 = (self.rcap2 & 0xFF) | (value << 8)

        elif addr == 0x88: # TCON
            self.tcon = value

        elif addr == 0x89: # TMOD
            self.tmod = value

        elif addr == 0x80: # P0
            self.p0 = value
            self.io.set_port(0, self.p0)

        elif addr == 0x90: # P1
            self.p1 = value
            self.io.set_port(1, self.p1)

        elif addr == 0xA0: # P2
            self.p2 = value
            self.io.set_port(2, self.p2)

        elif addr == 0xB0: # P3
            self.p3 = value
            self.io.set_port(3, self.p3)

        elif addr == 0x8A: # TL0
            self.timer0 = (self.timer0 & 0xFF00) | value

        elif addr == 0x8B: # TL1
            self.timer1 = (self.timer1 & 0xFF00) | value

        elif addr == 0x8C: # TH0
            self.timer0 = (self.timer0 & 0xFF) | (value << 8)

        elif addr == 0x8D: # TH1
            self.timer1 = (self.timer1 & 0xFF) | (value << 8)
            
        elif addr == 0xCC: # TL2
            self.timer2 = (self.timer2 & 0xFF00) | value

        elif addr == 0xCD: # TH2
            self.timer2 = (self.timer2 & 0xFF) | (value << 8)
            
        elif addr == 0xC8: # T2CON
            self.t2con = value

        elif addr == 0xC9: # T2MOD
            self.t2mod = value

        else:
            raise Exception("store_fsr %02X = %02X (from %04X)" % (addr, value, self.pc))
        
        
    # load ram
    def load_ram(self, addr: int) -> int:
        return self.ram[addr]
        

    def store_ram(self, addr: int, value: int) -> None:
        self.ram[addr] = value
        
        
    # load iram (RAM 00-7Fh and FSR 80h-FFh)
    def load_iram(self, addr: int, to_write: bool) -> int:
        if addr >= 128:
            return self.load_fsr(addr, to_write)
        else:
            return self.load_ram(addr)
        

    def store_iram(self, addr: int, value: int) -> None:
        if addr >= 128:
            self.store_fsr(addr, value)
        else:
            self.store_ram(addr, value)
            
            
    # bit addressables
    def parse_bit_addr(self, addr):
        if addr >= 0x00 and addr <= 0x7F:
            byte_addr = 0x20 + (addr >> 3)
            bit_addr = addr & 7
        
        elif addr >= 0x80 and addr <= 0xFF:
            byte_addr = addr & ~7
            bit_addr = addr & 7
            
        else:
            raise NotImplementedError(hex(addr))
            
        return byte_addr, bit_addr
    
        
    def load_bit_addr(self, addr, to_write: bool):
        byte_addr, bit_addr = self.parse_bit_addr(addr)
        value = self.load_iram(byte_addr, to_write)
        value >>= bit_addr
        return value & 1
    
        
    def store_bit_addr(self, addr, value):
        byte_addr, bit_addr = self.parse_bit_addr(addr)
        tmp = self.load_iram(byte_addr, True)
        if value:
            tmp |= (1 << bit_addr)
        else:
            tmp &= ~(1 << bit_addr)
        
        self.store_iram(byte_addr, tmp)
        
        
    # load @Rn (6-7) Rn (8-F)
    def load_rn(self, off):
        if off >= 6 and off <= 7:
            return self.load_ram(self.load_reg(off - 6))
        elif off >= 8 and off <= 0xF:
            return self.load_reg(off - 8)
        else:
            raise NotImplementedError(hex(off))
            

    def store_rn(self, off, value):
        if off >= 6 and off <= 7:
            return self.store_ram(self.load_reg(off - 6), value)
        elif off >= 8 and off <= 0xF:
            return self.store_reg(off - 8, value)
        else:
            raise NotImplementedError(hex(off))
            
    
    # stack magic
    def push_stack(self, value: int) -> None:
        self.sp = (self.sp + 1) & 0xFF
        self.store_ram(self.sp, value)
        

    def pop_stack(self) -> int:
        value = self.load_ram(self.sp)
        self.sp = (self.sp - 1) & 0xFF
        return value
        
        
    # external
    def load_xmem_addr8(self, addr: int) -> int:
        #self.xmem2.load_addr8(addr)
        return self.xmem.load_addr8(addr)
    
        
    def store_xmem_addr8(self, addr: int, value: int) -> None:
        self.xmem.store_addr8(addr, value)
        #self.xmem2.store_addr8(addr, value)

        
    def load_xmem_addr16(self, addr: int) -> int:
        #self.xmem2.load_addr16(addr)
        return self.xmem.load_addr16(addr)
    
        
    def store_xmem_addr16(self, addr: int, value: int) -> None:
        #self.xmem2.store_addr16(addr, value)
        self.xmem.store_addr16(addr, value)
        
        
    # instruction run
    def exec_instr(self) -> int:
        opcode = self.load_code(self.pc)
        
        if opcode == 0x00:
            # NOP
            self.pc += 1
            return 1
            
        elif (opcode & 0x0F) == 0x01:
            # AJMP
            # ACALL
            page = opcode >> 5
            addr = self.load_code(self.pc + 1)
            target = (self.pc & ~0x7FF) | (page << 8) | addr
            self.pc += 2
            
            if opcode & 0x10:
                self.call(target)
            else:
                self.jump(target)

            return 2
        
        elif opcode in (0x02, 0x12):
            # LJMP
            # LCALL
            target = (self.load_code(self.pc+1) << 8) | (self.load_code(self.pc+2))
            self.pc += 3
            
            if opcode & 0x10:
                self.call(target)
            else:
                self.jump(target)

            return 2
            
        elif opcode == 0x03:
            # RR A
            self.a = (self.a >> 1) | ((self.a & 0x1) << 7)
            self.pc += 1
            return 1
            
        elif opcode == 0x04:
            # INC A
            self.a = (self.a + 1) & 0xFF
            self.pc += 1

            return 1
            
        elif opcode == 0x05:
            # INC iram addr
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, True)
            self.store_iram(addr, (value + 1) & 0xFF)
            self.pc += 2

            return 1
            
        elif opcode >= 0x06 and opcode <= 0x0F:
            # INC @Rn, INC Rn
            value = self.load_rn(opcode & 0xF)
            self.store_rn(opcode & 0xF, (value + 1) & 0xFF)
            self.pc += 1

            return 1
            
        elif opcode == 0x10:
            # JBC bit addr, reladdr
            addr = self.load_code(self.pc+1)
            target = self.u8_to_s8(self.load_code(self.pc+2))
            value = self.load_bit_addr(addr, False) # XXX TODO FIXME
            self.pc += 3
            
            if value == 1:
                self.store_bit_addr(addr, 0)
                self.jump(self.pc + target)

            return 2
            
        elif opcode == 0x13:
            # RRC A
            tmp = self.a & 1
            self.a = (self.a >> 1) | (self.c_flag << 7)
            self.c_flag = tmp
            self.pc += 1
            return 1
            
        elif opcode == 0x14:
            # DEC A
            self.a = (self.a - 1) & 0xFF
            self.pc += 1

            return 1
            
        elif opcode == 0x15:
            # DEC iram addr
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, True)
            self.store_iram(addr, (value - 1) & 0xFF)
            self.pc += 2

            return 1
            
        elif opcode >= 0x16 and opcode <= 0x1F:
            # DEC @Rn, DEC Rn
            value = self.load_rn(opcode & 0xF)
            self.store_rn(opcode & 0xF, (value - 1) & 0xFF)
            self.pc += 1

            return 1
            
        elif opcode == 0x20:
            # JB bit addr, reladdr
            addr = self.load_code(self.pc+1)
            target = self.u8_to_s8(self.load_code(self.pc+2))
            value = self.load_bit_addr(addr, False)
            self.pc += 3
            
            if value == 1:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode == 0x22:
            # RET
            self.ret()
            return 2
            
        elif opcode == 0x23:
            # RL A
            self.a = ((self.a << 1) & 0xFF) | (self.a >> 7)
            self.pc += 1
            return 1
            
        elif (opcode >= 0x24 and opcode <= 0x2F) or (opcode >= 0x34 and opcode <= 0x3F):
            # ADD A, operand
            # ADDC A, operand
            self.pc += 1
                
            if (opcode & 0xF) == 0x4: # #data
                value = self.load_code(self.pc)
                self.pc += 1
                
            elif (opcode & 0xF) == 0x5: # iram addr
                value = self.load_iram(self.load_code(self.pc), False)
                self.pc += 1
                
            else:
                value = self.load_rn(opcode & 0xF)
                
            # todo: check flags
            
            if opcode & 0x10:
                value += self.c_flag
            
            # C flag    
            temp = self.a + value
            self.c_flag = 1 if (temp > 255) else 0
            
            # AC flag
            temp = ((self.a & 0xF) + (value & 0xF))
            self.ac_flag = 1 if (temp > 15) else 0
                
            # OV flag
            temp = self.u8_to_s8(value) + self.u8_to_s8(self.a)
            self.ov_flag = 1 if (temp < -128 or temp > 127) else 0
            
            # store A
            self.a = (self.a + value) & 0xFF
            return 1
            
        elif opcode == 0x30:
            # JNB bit addr, reladdr
            addr = self.load_code(self.pc+1)
            target = self.u8_to_s8(self.load_code(self.pc+2))
            value = self.load_bit_addr(addr, False)
            self.pc += 3
            
            if value == 0:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode == 0x32:
            # RETI
            self.reti()
            return 2
            
        elif opcode == 0x33:
            # RLC A
            tmp = (self.a >> 7) & 0x1
            self.a = ((self.a << 1) & 0xFF) | self.c_flag
            self.c_flag = tmp
            self.pc += 1
            return 1
            
        elif opcode == 0x40:
            # JC reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            self.pc += 2
            
            if self.c_flag == 1:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode in (0x42, 0x52, 0x62):
            # ORL iram addr, A
            # ANL iram addr, A
            # XRL iram addr, A
            op = [operator.or_, operator.and_, operator.xor][(opcode >> 4) - 4]
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, True)
            value = op(value, self.a)
            self.store_iram(addr, value)
            self.pc += 2
            return 1
            
        elif opcode in (0x43, 0x53, 0x63):
            # ORL iram addr, #data
            # ANL iram addr, #data
            # XRL iram addr, #data
            op = [operator.or_, operator.and_, operator.xor][(opcode >> 4) - 4]
            addr = self.load_code(self.pc+1)
            data = self.load_code(self.pc+2)
            value = self.load_iram(addr, True)
            value = op(value, data)
            self.store_iram(addr, value)
            self.pc += 3
            return 2
            
        elif opcode in (0x44, 0x54, 0x64):
            # ORL A, #data
            # ANL A, #data
            # XRL A, #data
            op = [operator.or_, operator.and_, operator.xor][(opcode >> 4) - 4]
            data = self.load_code(self.pc+1)
            self.a = op(self.a, data)
            self.pc += 2
            return 1
        
        elif opcode in (0x45, 0x55, 0x65):
            # ORL A, iram addr
            # ANL A, iram addr
            # XRL A, iram addr
            op = [operator.or_, operator.and_, operator.xor][(opcode >> 4) - 4]
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, False)
            self.a = op(self.a, value)
            self.pc += 2
            return 1
            
        elif opcode == 0x82:
            # ANL C, bit addr
            addr = self.load_code(self.pc+1)
            self.c_flag &= self.load_bit_addr(addr, False)
            self.pc += 2
            return 1
            
        elif opcode == 0xB0:
            # ANL C,/bit addr
            raise NotImplementedError("is this bitwise not?")
            return 2
            
        elif (opcode >= 0x46 and opcode <= 0x4F) or (opcode >= 0x56 and opcode <= 0x5F) or (opcode >= 0x66 and opcode <= 0x6F):
            # ORL A, Rn
            # ORL A, @Rn
            op = [operator.or_, operator.and_, operator.xor][(opcode >> 4) - 4]
            value = self.load_rn(opcode & 0xF)
            self.a = op(self.a, value)
            self.pc += 1
            return 1
            
        elif opcode == 0x50:
            # JNC reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            self.pc += 2
            
            if self.c_flag == 0:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode == 0x60:
            # JZ reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            self.pc += 2
            
            if self.a == 0:
                self.jump(self.pc + target)

            return 2
            
        elif opcode == 0x70:
            # JNZ reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            self.pc += 2
            
            if self.a != 0:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode == 0x72:
            # ORL C, bit addr
            addr = self.load_code(self.pc+1)
            self.c_flag |= self.load_bit_addr(addr, False)
            self.pc += 2
            return 2
            
        elif opcode == 0x73:
            # JMP @A+DPTR
            self.jump(self.a + self.dptr)
            return 2
            
        elif opcode == 0x80:
            # SJMP reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            self.pc += 2
            self.jump(self.pc + target)

            return 2
            
        elif opcode == 0x84:
            # DIV AB
            self.c_flag = 0
            if self.b == 0:
                self.ov_flag = 1
            else:
                quotient = self.a // self.b
                remainder = self.a % self.b
                self.a = quotient & 0xFF
                self.b = remainder & 0xFF
                self.ov_flag = 0
                
            self.pc += 1

            return 4
            
        elif opcode >= 0x94 and opcode <= 0x9F:
            # SUBB A, operand
            self.pc += 1
                
            if (opcode & 0xF) == 0x4: # #data
                value = self.load_code(self.pc)
                self.pc += 1
                
            elif (opcode & 0xF) == 0x5: # iram addr
                value = self.load_iram(self.load_code(self.pc), False)
                self.pc += 1
                
            else:
                value = self.load_rn(opcode & 0xF)
                
            # this can't be right TODO (also check for ADD/ADC)
            value += self.c_flag
            
            # C flag
            temp = self.a - value
            self.c_flag = 1 if (temp < 0) else 0
            
            # AC flag
            temp = ((self.a & 0xF) - (value & 0xF))
            self.ac_flag = 1 if (temp > 15) else 0
                
            # OV flag
            temp = self.u8_to_s8(value) + self.u8_to_s8(self.a)
            self.ov_flag = 1 if (temp < -128 or temp > 127) else 0
            
            # store A
            self.a = (self.a - value) & 0xFF
            return 1
            
        elif opcode == 0xA0:
            # ORL C, /bit addr
            raise NotImplementedError("is this bitwise not?")
            return 2
            
        elif opcode == 0xA3:
            # INC DPTR
            self.dptr = (self.dptr + 1) & 0xFFFF
            self.pc += 1

            return 2
            
        elif opcode == 0xA4:
            # MUL AB
            res = self.a * self.b
            self.c_flag = 0
            self.ov_flag = 1 if res > 255 else 0
            
            self.a = res & 0xFF
            self.b = (res >> 8) & 0xFF
            self.pc += 1

            return 4
            
        elif opcode == 0xA5:
            raise Exception("undefined instruction")
            
        elif opcode == 0xB2:
            # CPL bit addr
            addr = self.load_code(self.pc+1)
            self.store_bit_addr(addr, 1 - self.load_bit_addr(addr, True))
            self.pc += 2

            return 1
            
        elif opcode == 0xB3:
            # CPL C
            self.c_flag ^= 1
            self.pc += 1

            return 1
            
        elif opcode >= 0xB4 and opcode <= 0xBF:
            # CJNE operand1, operand2, reladdr
            if opcode == 0xB4:
                operand1 = self.a
                operand2 = self.load_code(self.pc+1)
            
            elif opcode == 0xB5:
                operand1 = self.a
                operand2 = self.load_iram(self.load_code(self.pc+1), False)
                
            else:
                operand1 = self.load_rn(opcode & 0xF)
                operand2 = self.load_code(self.pc+1)
            
            target = self.u8_to_s8(self.load_code(self.pc+2))
            self.c_flag = 1 if operand1 < operand2 else 0
            
            self.pc += 3
            if operand1 != operand2:
                self.jump(self.pc + target)

            return 2
                
        elif opcode == 0xC0:
            # PUSH iram addr 
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, False)
            self.push_stack(value)
            self.pc += 2
            return 2
            
        elif opcode in (0xC2, 0xD2):
            # CLR bit addr
            # SETB bit addr
            addr = self.load_code(self.pc+1)
            self.store_bit_addr(addr, 0 if opcode == 0xC2 else 1)
            self.pc += 2
            return 1
            
        elif opcode in (0xC3, 0xD3):
            # CLR C
            # SETB C
            self.c_flag = (0 if opcode == 0xC3 else 1)
            self.pc += 1
            return 1
            
        elif opcode == 0xC4:
            # SWAP A
            self.a = ((self.a << 4) | (self.a >> 4)) & 0xFF
            self.pc += 1
            return 1
            
        elif opcode == 0xC5:
            # XCHG A, iram addr
            addr = self.load_code(self.pc+1)
            
            tmp = self.a
            self.a = self.load_iram(addr, False)
            self.store_iram(addr, tmp)
            
            self.pc += 2
            return 1
            
        elif opcode >= 0xC6 and opcode <= 0xCF:
            # XCHG A, Rn
            # XCHG A, @Rn
            tmp = self.a
            self.a = self.load_rn(opcode & 0xF)
            self.store_rn(opcode & 0xF, tmp)
            self.pc += 1
            return 1
            
        elif opcode == 0xD0:
            # POP iram addr
            addr = self.load_code(self.pc+1)
            value = self.pop_stack()
            self.store_iram(addr, value)
            self.pc += 2
            return 2
            
        elif opcode == 0xD4:
            # DA
            raise NotImplementedError("DA")
        
            return 1
            
        elif opcode == 0xD5:
            # DJNZ iram addr, reladdr
            addr = self.load_code(self.pc+1)
            target = self.u8_to_s8(self.load_code(self.pc+2))
            
            value = self.load_iram(addr, False) # FIXME TODO XXX 
            value = (value - 1) & 0xFF
            self.store_iram(addr, value)
            
            self.pc += 3
            if value != 0:
                self.jump(self.pc + target)
            
            return 2
                
        elif opcode in (0xD6, 0xD7):
            # XCHD A, @Rn
            value = self.load_rn(opcode & 0xF)
            
            tmp = self.a & 0xF
            self.a = (self.a & 0xF0) | (value & 0xF)
            self.store_rn(opcode & 0xF, (value & 0xF0) | tmp)
            
            self.pc += 1
            return 1
            
        elif opcode >= 0xD8 and opcode <= 0xDF:
            # DJNZ Rn, reladdr
            target = self.u8_to_s8(self.load_code(self.pc+1))
            
            value = self.load_rn(opcode & 0xF)
            value = (value - 1) & 0xFF
            self.store_rn(opcode & 0xF, value)
            
            self.pc += 2
            if value != 0:
                self.jump(self.pc + target)
            
            return 2
            
        elif opcode == 0xE4:
            # CLR A
            self.a = 0
            self.pc += 1

            return 1
            
        elif opcode == 0xF4:
            # CPL A
            self.a = self.a ^ 0xFF
            self.pc += 1

            return 1
            
        elif opcode == 0x74:
            # MOV A,#data
            data = self.load_code(self.pc+1)
            self.a = data
            self.pc += 2

            return 1
            
        elif opcode == 0x75:
            # MOV iram addr, #data
            addr = self.load_code(self.pc+1)
            data = self.load_code(self.pc+2)
            self.store_iram(addr, data)
            self.pc += 3

            return 2
            
        elif opcode >= 0x76 and opcode <= 0x7F:
            # MOV @Rn, #data
            # MOV Rn, #data
            data = self.load_code(self.pc+1)
            self.store_rn(opcode & 0xF, data)
            self.pc += 2

            return 1
            
        elif opcode == 0x83:
            # MOVC A, @A+PC
            self.pc += 1
            self.a = self.load_code(self.a + self.pc) # TODO: check which pc it's supposed to be!
            return 2
            
        elif opcode == 0x85:
            # MOV iram addr, iram addr
            src = self.load_code(self.pc+1)
            dst = self.load_code(self.pc+2)
            self.store_iram(dst, self.load_iram(src, False))
            self.pc += 3

            return 2
            
        elif opcode >= 0x86 and opcode <= 0x8F:
            # MOV iram addr, @Rn
            # MOV iram addr, Rn
            addr = self.load_code(self.pc+1)
            self.store_iram(addr, self.load_rn(opcode & 0xF))
            self.pc += 2

            return 2
            
        elif opcode == 0x90:
            # MOV DPTR, #data16
            # check endianness!
            data16 = (self.load_code(self.pc+1) << 8) | (self.load_code(self.pc+2))
            self.dptr = data16
            self.pc += 3

            return 2
            
        elif opcode == 0x92:
            # MOV bit addr, C
            addr = self.load_code(self.pc+1)
            self.store_bit_addr(addr, self.c_flag)
            self.pc += 2

            return 2
            
        elif opcode == 0xA2:
            # MOV C, bit addr
            addr = self.load_code(self.pc+1)
            self.c_flag = self.load_bit_addr(addr, False)
            self.pc += 2

            return 1
            
        elif opcode >= 0xA6 and opcode <= 0xAF:
            # MOV Rn, iram addr
            # MOV @Rn, iram addr
            addr = self.load_code(self.pc+1)
            self.store_rn(opcode & 0xF, self.load_iram(addr, False))
            self.pc += 2

            return 2
            
        elif opcode == 0xE5:
            # MOV A, iram addr
            addr = self.load_code(self.pc+1)
            value = self.load_iram(addr, False)
            self.a = value
            self.pc += 2

            return 1
            
        elif opcode >= 0xE6 and opcode <= 0xEF:
            # MOV A, Rn
            # MOV A, @Rn
            value = self.load_rn(opcode & 0xF)
            self.a = value
            self.pc += 1

            return 1
            
        elif opcode == 0xF5:
            # MOV iram addr, A
            addr = self.load_code(self.pc+1)
            self.store_iram(addr, self.a)
            self.pc += 2

            return 1
            
        elif opcode >= 0xF6 and opcode <= 0xFF:
            # MOV Rn, A
            # MOV @Rn, A
            self.store_rn(opcode & 0xF, self.a)
            self.pc += 1

            return 1
            
        elif opcode == 0x93:
            # MOVC A, @A+DPTR
            self.a = self.load_code(self.a + self.dptr)
            self.pc += 1

            return 2
                
        elif opcode == 0xE0:
            # MOVX A, @DPTR
            value = self.load_xmem_addr16(self.dptr)
            self.a = value
            self.pc += 1

            return 2
            
        elif opcode in (0xE2, 0xE3):
            # MOVX A, @Rn
            value = self.load_xmem_addr8(self.load_reg(opcode - 0xE2))
            self.a = value
            self.pc += 1

            return 2
            
        elif opcode == 0xF0:
            # MOVX @DPTR, A
            self.store_xmem_addr16(self.dptr, self.a)
            self.pc += 1

            return 2
            
        elif opcode in (0xF2, 0xF3):
            # MOVX @Rn, A
            self.store_xmem_addr8(self.load_reg(opcode - 0xF2), self.a)
            self.pc += 1

            return 2
            
        else:
            raise NotImplementedError("op: %02X" % opcode)
            
        #if self.pc == oldPc:
        #    raise Exception("pc didnt change?", hex(self.pc))
        raise Exception("unk length: %02X" % opcode)
    

    def update_timer2(self, cycles: int) -> None:
        down_counter = self.t2mod & 1
        # output_enable is not relevant yet TODO what is this

        if self.t2con & 0b100:
            if self.t2con & 0b01111011:
                raise NotImplementedError("t2con not handled", bin(self.t2con))
            
            self.timer2 += cycles
            if self.timer2 > 0xFFFF:
                self.timer2 &= 0xFFFF
                self.timer2 += self.rcap2

                self.t2con |= (1 << 7) # overflow


    def update_timer01(self, timer: int, cycles: int) -> None:
        if timer == 0:
            mode = self.tmod & 0b1111
            run = (self.tcon >> 4) & 1
            value = self.timer0

        elif timer == 1:
            mode = (self.tmod >> 4) & 0b1111
            run = (self.tcon >> 6) & 1
            value = self.timer1
        
        else:
            raise ValueError("Invalid timer %d" % timer)
        
        if mode & 0b1000:
            # GATE - we can't do that
            raise NotImplementedError("GATEn - NOT IMPLEMENTED.")

        if mode & 0b100:
            # C/Tn# - we can't do that
            raise NotImplementedError("C/Tn# - NOT IMPLEMENTED.")

        mode &= 0b11

        if run:
            # Timer0 running
            overflow = False
            
            if mode == 0b00:
                # 8-bit counter with 5-bit prescaler
                value &= 0x1FFF
                value += cycles 

                if value > 0x1FFF:
                    value &= 0x1FFF
                    overflow = True
            
            elif mode == 0b01:
                # 16-bit timer/counter
                value &= 0xFFFF
                value += cycles 

                if value > 0xFFFF:
                    value &= 0xFFFF
                    overflow = True
            
            elif mode == 0b10:
                # Mode 2: 8-bit auto-reload timer/counter (TLn).
                # Reloaded from THn at overflow.
                tmp = (value >> 8) & 0xFF
                value &= 0xFF
                value += cycles

                if value > 0xFF:
                    value &= 0xFF
                    value += tmp
                    overflow = True

                value |= (tmp << 8)

            elif mode == 0b11:
                # Mode 3: TL0 is an 8-bit timer/counter.
                raise NotImplementedError("Timer Mode 3 is not implemented")
                value &= 0xFF

                if value > 0xFF:
                    value -= 0xFF
                    overflow = True

            
            if overflow:
                if timer == 0:
                    self.tcon |= (1<<5)

                elif timer == 1:
                    self.tcon |= (1<<7)

                
        if timer == 0:
            self.timer0 = value
        elif timer == 1:
            self.timer1 = value
        
                
    def update_timers(self, cycles: int) -> None:
        self.update_timer01(0, cycles)
        self.update_timer01(1, cycles)
        self.update_timer2(cycles)

        if self.tcon & (1<<5):
            self.on_irq(0x0B, 1)

        if self.tcon & (1<<7):
            self.on_irq(0x1B, 3)
            
        if self.t2con & (1 << 7):
            self.on_irq(0x2B, 5)


    def update_serial(self, nb_instr: int) -> None:
        # TODO: handle serial AND timer (baudrate) (rn it's temporary)
        mode = (self.scon >> 6) & 0b11
        
        '''self.epic_dummy = getattr(self, "epic_dummy", 0) + 1
        if (self.epic_dummy % 1000) == 0:
            print("SCON RX")
            self.sbuf_in = 0x43 | (1<<7)
            self.scon |= (1<<0)
            self.scon |= (1<<2) # stop bit?'''

        if self.sbuf_out is not None:
            print("SBUF_OUT: %02X (aka %02X or %r)" % (self.sbuf_out, self.sbuf_out & 0x7F, chr(self.sbuf_out & 0x7F)))
            self.sbuf_out = None
            self.scon |= (1<<1)

        if self.scon & 0b11:
            self.on_irq(0x23, 4)


    def update_exti(self) -> None:
        # INT0 - P3.2
        # INT1 - P3.3
        for n in (0, 1):
            if self.tcon & (1<<(n*2)):
                # Interrupt on P3.2 falling edge
                if (self.p3_input_prev & (1<<(n+2))) and not (self.p3_input & (1<<(n+2))):
                    self.tcon |= (1<<(1+n*2))
            else:
                if not (self.p3_input & (1<<(n+2))):
                    self.tcon |= (1<<(1+n*2))

        self.p3_input_prev = self.p3_input

        # Check IRQ
        if self.tcon & (1<<1):
            if self.on_irq(0x03, 0):
                self.tcon &= ~(1 << 0)

        if self.tcon & (1<<3):
            if self.on_irq(0x13, 2):
                self.tcon &= ~(1 << 2)


    def run_instr(self) -> int:
        self.p3_input = self.io.get_port(3)
        self.update_exti()
        
        cycles = self.exec_instr()

        self.update_timers(cycles)
        self.update_serial(cycles)

        return cycles
    

    def dump_state(self) -> str:
        buf = ""
        buf += "PC: %04X\n" % self.pc
        buf += "A: %02X\n" % self.a
        buf += "B: %02X\n" % self.b
        buf += "DPTR: %04X\n" % self.dptr
        buf += "SP: %04X\n" % self.sp
        for i in range(8):
            buf += "R%d: %02X\n" % (i, self.load_reg(i))

        buf += "TIMER0: %04X\n" % (self.timer0)
        buf += "TIMER1: %04X\n" % (self.timer1)
        buf += "TIMER2: %04X\n" % (self.timer2)
        buf += "P0: %02X\n" % (self.p0_input)
        buf += "P1: %02X\n" % (self.p1_input)
        buf += "P2: %02X\n" % (self.p2_input)
        buf += "P3: %02X\n" % (self.p3_input)

        return buf