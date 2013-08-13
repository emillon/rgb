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
            oam: ~[], // FIXME
            rom: rom,
            vram: ~[], // FIXME
            wram: vec::from_elem(8192, 0 as u8),
            zram: ~[]  // FIXME
        }
    }

    fn vmem<'a> (&'a mut self, addr: u16) -> Option<&'a mut u8> {
        match addr & 0xF000 {
            0x0000 => {
	        if (self.bios_is_mapped) {
		    if(addr < 0x0100) {
		        return Some (&mut self.bios[addr]);
                    }
                    /* TODO
		    else if(Z80._r.pc == 0x0100)
		        MMU._inbios = 0;
                    */
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
                match addr & 0x1F00 {
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
                        if(addr >= 0xFF80) {
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
        (hi as u16 << 8) + lo as u16
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

enum Flag {
    F_Z
}

enum R8 {
    R8_B,
    R8_C
}

struct CPU {
    mmu: ~MMU,
    pc: u16,
    reg_a: u8,
    reg_bc: u16,
    reg_hl: u16,
    reg_sp: u16
}

impl CPU {
    fn new(mmu: ~MMU) -> CPU {
        CPU {
            mmu: mmu,
            pc: 0x100,
            reg_a : 0,
            reg_bc : 0,
            reg_hl : 0,
            reg_sp : 0
        }
    }

    fn r8(&self, r: R8) -> u8 {
        match r {
            R8_B => u16_hi(self.reg_bc),
            R8_C => u16_lo(self.reg_bc)
        }
    }

    fn flag_is_reset(&self, f: Flag) -> bool {
        true // TODO
    }

    fn interp(&mut self) {
        let opcode = self.mmu.rb(self.pc);
        let mut next_pc = self.pc + 1;
        match opcode {
            0x00 => { // NOP
            }
            0x01 => { // LD BC, nn nn
                let val = self.mmu.rw (self.pc + 1);
                next_pc += 2;
                self.reg_bc = val
            }
            0x0B => { // DEC BC
                self.reg_bc -= 1
            }
            0x20 => { // JR NZ, nn
                let off = self.mmu.rb(self.pc + 1);
                next_pc += 1;
                if self.flag_is_reset(F_Z) {
                    next_pc += off as u16
                }
            }
            0x21 => { // LD HL, nn nn
                let val = self.mmu.rw(self.pc + 1);
                next_pc += 2;
                self.reg_hl = val
            }
            0x22 => { // LDI (HL), A
                self.mmu.wb(self.reg_hl, self.reg_a);
                self.reg_hl += 1;
            }
            0x79 => { // LD A, C
                self.reg_a = self.r8(R8_C)
            }
            0xC3 => { // JP nn nn
                let dest = self.mmu.rw(self.pc + 1);
                next_pc = dest
            }
            0xC9 => { // RET
                next_pc = self.mmu.rw(self.reg_sp);
                self.reg_sp += 2;
            }
            0x31 => { // LD SP, nn nn
                let val = self.mmu.rw(self.pc + 1);
                next_pc += 2;
                self.reg_sp = val
            }
            0xAF => { // XOR A
                self.reg_a ^= self.reg_a
            }
            0xB0 => { // OR B
                self.reg_a |= self.r8(R8_B)
            }
            0xCD => { // CALL nn
                self.reg_sp -= 2;
                self.mmu.ww(self.reg_sp, self.pc);
                let dest = self.mmu.rw (self.pc + 1);
                next_pc = dest
            }
            0xE5 => { // PUSH HL
                self.reg_sp -= 2;
                self.mmu.ww(self.reg_sp, self.reg_hl);
            }
            0xF3 => { // DI
                /* TODO disable interrupts */
            }
            _ => {
                fail!(fmt!("Unknown opcode : %02X", opcode as uint))
            }
        }
        self.pc = next_pc
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
