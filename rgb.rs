use std::io;
use std::os;
use std::str;
use std::vec;

struct ROM {
    mem: ~[u8]
}

impl ROM {
    fn new(path: PosixPath) -> Result<~ROM, ~str> {
        match io::read_whole_file(&path) {
            Ok(v) => {
                println(fmt!("Read %u bytes", v.len()));
                Ok(~ROM { mem: v })
            }
            Err(s) => { Err (s) }
        }

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
    fn new(rom: ~ROM) -> MMU {
        MMU {
            bios_is_mapped: true,
            bios: ~[0xC3, 0x00, 0x01], // FIXME: just jump into the cartridge
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
        let lo = (val & 0xFF) as u8;
        let hi = ((val & 0xFF00) >> 8) as u8;
        self.wb(addr, lo);
        self.wb(addr, hi);
    }
}

struct CPU {
    mmu: ~MMU,
    pc: u16,
    reg_sp: u16
}

impl CPU {
    fn new(mmu: ~MMU) -> CPU {
        CPU {
            mmu: mmu,
            pc: 0,
            reg_sp : 0
        }
    }

    fn interp(&mut self) {
        let opcode = self.mmu.rb(self.pc);
        let mut next_pc = self.pc + 1;
        match opcode {
            0x00 => { // NOP
            }
            0xC3 => { // JP nn nn
                let dest = self.mmu.rw (self.pc + 1);
                next_pc = dest
            }
            0x31 => { // LD SP, nn nn
                let val = self.mmu.rw (self.pc + 1);
                next_pc += 2;
                self.reg_sp = val
            }
            0xCD => { // CALL nn
                self.reg_sp -= 2;
                self.mmu.ww(self.reg_sp, self.pc);
                let dest = self.mmu.rw (self.pc + 1);
                next_pc = dest
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
    let path = PosixPath(file);
    match ROM::new(path) {
        Ok (r) => {
            r.dump_header();
            let mmu = MMU::new(r);
            let mut cpu = CPU::new(~mmu);
            loop {
                cpu.interp()
            }
        }
        Err (s) => {
            println(fmt!("Cannot read %s: %s", file, s));
        }
    }
}
