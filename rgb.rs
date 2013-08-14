use std::io;
use std::os;
use std::str;
use std::vec;

struct ROM {
    mem: ~[u8]
}

impl ROM {
    fn new(rom_data: ~[u8]) -> ROM {
        ROM { mem: rom_data }
    }

    fn read_word_be(&self, offset: u16) -> u16 {
        (self.mem[offset] as u16 << 8) + self.mem[offset + 1] as u16
    }

    fn hdr_checksum(&self) -> u8 {
        self.mem.slice(0x134, 0x14C).iter().fold(-1, |n, &v| n + v - 1)
    }

    fn rom_checksum(&self) -> u16 {
        let sum: u16 = self.mem.iter().fold(0, |n, &v|  n + v as u16);
        let ck_hi = self.mem[0x014E] as u16;
        let ck_lo = self.mem[0x014F] as u16;
        sum - ck_hi - ck_lo
    }

    fn dump_header (&self) {
        let title = str::from_bytes(self.mem.slice(0x134, 0x143));
        let ct = self.mem[0x147];
        print(fmt!("Title: %s\nType : %u\n", title, ct as uint));
        let stored_hdr_checksum = self.mem[0x014D];
        let hdr_ck_str =
            if self.hdr_checksum() == stored_hdr_checksum {
                "OK"
            } else {
                "NOT OK"
            };
        let stored_rom_checksum: u16 = self.read_word_be(0x014E);
        let rom_ck_str =
            if self.rom_checksum() == stored_rom_checksum {
                "OK"
            } else {
                "NOT OK"
            };
        println(fmt!("HDR Checksum: %02X (%s)", stored_hdr_checksum as uint, hdr_ck_str));
        println(fmt!("ROM Checksum: %04X (%s)", stored_rom_checksum as uint, rom_ck_str));
    }
}

struct MMU {
    bios_is_mapped: bool,
    bios: ~[u8],
    eram: ~[u8],
    ie: u8,
    oam: ~[u8],
    rom: ~ROM,
    vram: ~[u8],
    wram: ~[u8],
    zram: ~[u8]
}

impl MMU {
    fn new(rom: ~ROM, bios: ~[u8]) -> MMU {
        MMU {
            bios_is_mapped: true,
            bios: bios,
            eram: ~[], // FIXME
            ie: 0,
            oam: vec::from_elem(160, 0 as u8),
            rom: rom,
            vram: vec::from_elem(8192, 0 as u8),
            wram: vec::from_elem(8192, 0 as u8),
            zram: vec::from_elem(127, 0 as u8),
        }
    }

    fn interrupts_disable(&mut self) {
        self.ie = 0
    }

    fn vmem<'a> (&'a mut self, addr: u16) -> Option<&'a mut u8> {
        match addr & 0xF000 {
            0x0000 => {
	        if (self.bios_is_mapped) {
		    if(addr < 0x0100) {
		        return Some (&mut self.bios[addr]);
                    }
		}
		Some (&mut self.rom.mem[addr])
            }
              0x1000 | 0x2000 | 0x3000 // ROM0 (rest)
            | 0x4000 | 0x5000 | 0x6000 | 0x7000 // ROM1
            => {
                Some (&mut self.rom.mem[addr])
            }
            0x8000 | 0x9000 => {
                Some (&mut self.vram[addr & 0x1fff])
            }
            0xA000 | 0xB000 => {
                Some (&mut self.eram[addr & 0x1fff])
            }
            0xC000 | 0xD000 | 0xE000 => {
                Some (&mut self.wram[addr & 0x1fff])
            }
            0xF000 => {
                match addr & 0x0F00 {
                      0x000 | 0x100 | 0x200 | 0x300
                    | 0x400 | 0x500 | 0x600 | 0x700
                    | 0x800 | 0x900 | 0xA00 | 0xB00
		    | 0xC00 | 0xD00 => {
                        Some (&mut self.wram[addr & 0x1fff])
                    }
                    0xE00 => {
                        if(addr < 0xFEA0) {
			    Some (&mut self.oam[addr & 0xFF])
                        } else {
			    None
                        }
                    }
                    0xF00 => {
                        if addr == 0xFFFF {
                            Some (&mut self.ie)
                        } else if(addr >= 0xFF80) {
			    Some (&mut self.zram[addr & 0x7F])
			} else {
			    // I/O
			    None
			}
                    }
                    _ => { fail!("MMU::rb") }
                }
            }
            _ => { fail!("MMU::rb") }
        }
    }

    fn rb(&mut self, addr: u16) -> u8 {
        match self.vmem(addr) {
            Some (p) => *p,
            None => 0
        }
    }

    fn wb(&mut self, addr: u16, val: u8) {
        match self.vmem(addr) {
            Some (p) => *p = val,
            None => {}
        }
    }

    fn rw(&mut self, addr: u16) -> u16 {
        let lo = self.rb(addr);
        let hi = self.rb(addr + 1);
        u16_make(hi, lo)
    }

    fn ww(&mut self, addr: u16, val: u16) {
        let lo = u16_lo(val);
        let hi = u16_hi(val);
        self.wb(addr, lo);
        self.wb(addr, hi);
    }
}

fn u16_lo(n: u16) -> u8 {
    (n & 0xFF) as u8
}

fn u16_hi(n: u16) -> u8 {
    ((n & 0xFF00) >> 8) as u8
}

fn u16_make(hi: u8, lo: u8) -> u16 {
    (hi as u16 << 8) + lo as u16
}

fn u16_set_hi(w: u16, b: u8) -> u16 {
    u16_make(b, u16_lo(w))
}

fn u16_set_lo(w: u16, b: u8) -> u16 {
    u16_make(u16_hi(w), b)
}

enum Flag {
    F_Z,
    F_N,
    F_H,
    F_C
}

impl Flag {
    fn mask (&self) -> u8 {
        match *self {
            F_Z => 0x80,
            F_N => 0x40,
            F_H => 0x20,
            F_C => 0x10
        }
    }
}

enum R8 {
    R8_A,
    R8_F,
    R8_B,
    R8_C,
    R8_D,
    R8_E,
    R8_H,
    R8_L,
}

enum ALU_Op {
    Op_OR,
    Op_XOR,
    Op_AND,
    Op_ADD,
    Op_ADC,
}

enum Addressing_Mode {
    A_Indirect(u16), // FIXME should be R16
    A_Direct(R8),
    A_Immediate,
}

struct CPU {
    mmu: ~MMU,
    pc: u16,
    reg_af: u16,
    reg_bc: u16,
    reg_de: u16,
    reg_hl: u16,
    reg_sp: u16
}

impl CPU {
    fn new(mmu: ~MMU) -> CPU {
        CPU {
            mmu: mmu,
            pc: 0x0,
            reg_af : 0,
            reg_bc : 0,
            reg_de : 0,
            reg_hl : 0,
            reg_sp : 0
        }
    }

    fn r8(&self, r: R8) -> u8 {
        match r {
            R8_A => u16_hi(self.reg_af),
            R8_F => u16_lo(self.reg_af),
            R8_B => u16_hi(self.reg_bc),
            R8_C => u16_lo(self.reg_bc),
            R8_D => u16_hi(self.reg_de),
            R8_E => u16_lo(self.reg_de),
            R8_H => u16_hi(self.reg_hl),
            R8_L => u16_lo(self.reg_hl),
        }
    }

    fn w8(&mut self, r: R8, v: u8) {
        match r {
            R8_A => self.reg_af = u16_set_hi(self.reg_af, v),
            R8_F => self.reg_af = u16_set_lo(self.reg_af, v),
            R8_B => self.reg_bc = u16_set_hi(self.reg_bc, v),
            R8_C => self.reg_bc = u16_set_lo(self.reg_bc, v),
            R8_D => self.reg_de = u16_set_hi(self.reg_de, v),
            R8_E => self.reg_de = u16_set_lo(self.reg_de, v),
            R8_H => self.reg_hl = u16_set_hi(self.reg_hl, v),
            R8_L => self.reg_hl = u16_set_lo(self.reg_hl, v),
        }
    }

    fn flag_is_set(&self, f: Flag) -> bool {
        (self.r8(R8_F) & f.mask()) != 0
    }

    fn flag_is_reset(&self, f: Flag) -> bool {
        !self.flag_is_set(f)
    }

    fn flag_set_bool(&mut self, f: Flag, v: bool) {
        if v {
            self.flag_set(f)
        } else {
            self.flag_reset(f)
        }
    }

    fn flag_set(&mut self, f: Flag) {
        let flags = self.r8(R8_F);
        self.w8(R8_F, flags | f.mask())
    }

    fn flag_reset(&mut self, f: Flag) {
        let flags = self.r8(R8_F);
        self.w8(R8_F, flags & (! f.mask()))
    }

    fn interp(&mut self) {
        let opcode = self.mmu.rb(self.pc);
        let mut next_pc = self.pc + 1;
        println(fmt!("PC=%04X OP=%02X", self.pc as uint, opcode as uint));
        let arg_b = || {
            next_pc += 1;
            self.mmu.rb(self.pc + 1)
        };
        let arg_w = || {
            next_pc += 2;
            self.mmu.rw(self.pc + 1)
        };
        let call_cond = |flag| {
            let dest = arg_w();
            if self.flag_is_set(flag) {
                next_pc = dest
            }
        };
        let jr_cond = |flag, exp_value| {
            let offset = arg_b();
            if self.flag_is_set(flag) == exp_value {
                next_pc += offset as u16
            }
        };
        let alu_op = |op: ALU_Op, what: Option<R8>| {
            let a = self.r8(R8_A);
            let y = match what {
                Some(reg) => self.r8(reg),
                None => arg_b()
            };
            let z = match op {
                Op_OR  => a | y,
                Op_XOR => a ^ y,
                Op_AND => a & y,
                Op_ADD => a + y,
                Op_ADC => a + y + (if self.flag_is_set(F_C) { 1 } else { 0 })
            };
            self.w8(R8_A, z);
            let a2 = self.r8(R8_A);
            let honor_z = || {
                self.flag_set_bool(F_Z, a2 == 0);
            };
            match op {
                Op_OR | Op_XOR => {
                    honor_z();
                    self.flag_reset(F_N);
                    self.flag_reset(F_H);
                    self.flag_reset(F_C);
                }
                Op_AND => {
                    honor_z();
                    self.flag_reset(F_N);
                    self.flag_set(F_H);
                    self.flag_reset(F_C);
                }
                Op_ADD | Op_ADC => {
                    honor_z();
                    self.flag_reset(F_N);
                    // TODO honor h
                    // TODO honor c
                }
            }
        };
        let ld8 = |dest, src: Addressing_Mode| {
            let val = match src {
                A_Indirect(addr) => self.mmu.rb(addr),
                A_Direct(reg) => self.r8(reg),
                A_Immediate => arg_b(),
            };
            self.w8(dest, val)
        };
        let ld8_ind = |dest, src| { // LD (dest), src
            let v = self.r8(src);
            self.mmu.wb(dest, v);
        };
        let push_w = |val| {
            self.reg_sp -= 2;
            self.mmu.ww(self.reg_sp, val);
        };
        let pop_w = || {
            let val = self.mmu.rw(self.reg_sp);
            self.reg_sp += 2;
            val
        };
        let ret = || {
            next_pc = pop_w();
        };
        match opcode {
            0x00 => { // NOP
            }
            0x01 => { // LD BC, nn nn
                self.reg_bc = arg_w();
            }
            0x03 => { // INC BC
                self.reg_bc += 1;
            }
            0x08 => { // LD (nn nn), SP
                let addr = arg_w();
                self.mmu.ww(addr, self.reg_sp);
            }
            0x0B => { // DEC BC
                self.reg_bc -= 1;
            }
            0x0C => { // INC C
                let c = self.r8(R8_C);
                self.w8(R8_C, c + 1);
                let c2 = self.r8(R8_C);
                self.flag_set_bool(F_Z, c2 == 0);
                self.flag_reset(F_N);
                // TODO F_H
            }
            0x0D => { // DEC C
                let c = self.r8(R8_C);
                self.w8(R8_C, c-1);
                let c2 = self.r8(R8_C);
                self.flag_set_bool(F_Z, c2 == 0);
                self.flag_set(F_N)
                // TODO F_H
            }
            0x0E => { // LD C, nn
                ld8(R8_C, A_Immediate);
            }
            0x12 => { // LD (DE), A
                ld8_ind(self.reg_de, R8_A)
            }
            0x20 => { // JR NZ, nn
                jr_cond(F_Z, false)
            }
            0x21 => { // LD HL, nn nn
                self.reg_hl = arg_w()
            }
            0x22 => { // LDI (HL), A
                let a = self.r8(R8_A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl += 1;
            }
            0x2E => { // LD L, nn
                ld8(R8_L, A_Immediate)
            }
            0x30 => { // JR NC, nn
                jr_cond(F_C, false)
            }
            0x31 => { // LD SP, nn nn
                self.reg_sp = arg_w()
            }
            0x32 => { // LDD (HL), A
                let a = self.r8(R8_A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl -= 1;
            }
            0x38 => { // JR C, nn
                jr_cond(F_C, true)
            }
            0x3E => { // LD A, nn
                ld8(R8_A, A_Immediate)
            }
            0x66 => { // LD H, (HL)
                let val = self.mmu.rb(self.reg_hl);
                self.w8(R8_H, val)
            }
            0x73 => { // LD (HL), E
                ld8_ind(self.reg_hl, R8_E);
            }
            0x79 => { // LD A, C
                ld8(R8_A, A_Direct(R8_C))
            }
            0x7E => { // LD A,(HL)
                ld8(R8_A, A_Indirect(self.reg_hl))
            }
            0x83 => { // ADD A, E
                alu_op(Op_ADD, Some(R8_E))
            }
            0x88 => { // ADC A, B
                alu_op(Op_ADC, Some(R8_B));
            }
            0x89 => { // ADC A, C
                alu_op(Op_ADC, Some(R8_C));
            }
            0xAF => { // XOR A
                alu_op(Op_XOR, Some(R8_A));
            }
            0xB0 => { // OR B
                alu_op(Op_OR, Some(R8_B));
            }
            0xC0 => { // RET NZ
                if self.flag_is_reset(F_Z) {
                    ret()
                }
            }
            0xC3 => { // JP nn nn
                let dest = self.mmu.rw(self.pc + 1);
                next_pc = dest
            }
            0xC9 => { // RET
                ret()
            }
            0xCB => { // ext ops
                let op = arg_b();
                match op {
                    0x7C => { // BIT 7, H
                        let h = self.r8(R8_H);
                        self.flag_set_bool(F_Z, (h & (1 << 7)) == 0);
                        self.flag_reset(F_N);
                        self.flag_set(F_H);
                    }
                    _ => fail!(fmt!("Unknown ext op : CB %02X", op as uint))
                }
            }
            0xCC => { // CALL Z, nn nn
                call_cond(F_Z)
            }
            0xCD => { // CALL nn nn
                push_w(self.pc);
                let dest = arg_w(); // FIXME
                next_pc = dest
            }
            0xD5 => { // PUSH DE
                push_w(self.reg_de)
            }
            0xDC => { // CALL C, nn nn
                call_cond(F_C)
            }
            0xDD => {
                fail!(fmt!("Bad opcode : %02X", opcode as uint))
            }
            0xE2 => { // LD (FF00+C), A
                let a = self.r8(R8_A);
                let c = self.r8(R8_C);
                let addr = u16_make(0xFF, c);
                self.mmu.wb(addr, a);
            }
            0xE5 => { // PUSH HL
                push_w(self.reg_hl)
            }
            0xE6 => { // AND nn
                alu_op(Op_AND, None)
            }
            0xF6 => { // OR nn
                alu_op(Op_OR, None)
            }
            0xFE => { // CP A, nn
                let n = arg_b();
                let a = self.r8(R8_A);
                self.flag_set_bool(F_Z, a == n);
                self.flag_set(F_N);
                // TODO F_H
                // TODO F_C
            }
            0xF3 => { // DI
                self.mmu.interrupts_disable()
            }
            0xFF => { // RST 0x38
                // TODO save regs
                push_w(self.pc);
                next_pc = 0x38;
            }
            _ => {
                fail!(fmt!("Unknown opcode : %02X", opcode as uint))
            }
        }
        if (self.pc < 0x0100 && next_pc >= 0x0100) {
            // Jumping out of bios
            self.mmu.bios_is_mapped = false
        }
        // TODO detect PC wrap
        self.pc = next_pc;
    }
}

fn main() {
    println("rgb");
    let args = os::args();
    if args.len() <= 1 {
        println("Usage: rgb file.gb");
        os::set_exit_status(1);
        return
    }
    let file = os::args()[1];
    let rom_data = io::read_whole_file(&PosixPath(file)).expect(fmt!("Cannot read %s", file));
    let rom = ~ROM::new(rom_data);
    rom.dump_header();
    let bios = io::read_whole_file(&PosixPath("bios.dat")).expect("Cannot open bios");
    let mmu = MMU::new(rom, bios);
    let mut cpu = CPU::new(~mmu);
    loop {
        cpu.interp()
    }
}
