use std::vec;

use bits::*;
use rom::ROM;

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
    pub fn new(rom: ~ROM, bios: ~[u8]) -> MMU {
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

    pub fn interrupts_disable(&mut self) {
        self.ie = 0
    }

    fn vmem<'a> (&'a mut self, addr: u16) -> Option<&'a mut u8> {
        match addr {
            0x0000..0x00FF => {
                if self.bios_is_mapped {
                    Some (&mut self.bios[addr])
                } else {
                    Some (&mut self.rom.mem[addr])
                }
            }
            0x0100..0x7FFF => { Some (&mut self.rom.mem[addr]) }
            0x8000..0x9FFF => { Some (&mut self.vram[addr & 0x1fff]) }
            0xA000..0xBFFF => { Some (&mut self.eram[addr & 0x1fff]) }
            0xC000..0xFDFF => { Some (&mut self.wram[addr & 0x1fff]) }
            0xFE00..0xFE9F => { Some (&mut self.oam[addr & 0xFF]) }
            0xFEA0..0xFEFF => { None }
            0xFF00..0xFF7F => { None } // I/O TODO
            0xFF80..0xFFFE => { Some (&mut self.zram[addr & 0x7F]) }
            0xFFFF         => { Some (&mut self.ie) }
            _ => {
                println(fmt!("%04X", addr as uint));
                fail!("MMU::rb")
            }
        }
    }

    pub fn rb(&mut self, addr: u16) -> u8 {
        match self.vmem(addr) {
            Some (p) => *p,
            None => 0
        }
    }

    pub fn wb(&mut self, addr: u16, val: u8) {
        match self.vmem(addr) {
            Some (p) => *p = val,
            None => {}
        }
    }

    pub fn rw(&mut self, addr: u16) -> u16 {
        let lo = self.rb(addr);
        let hi = self.rb(addr + 1);
        u16_make(hi, lo)
    }

    pub fn ww(&mut self, addr: u16, val: u16) {
        let lo = u16_lo(val);
        let hi = u16_hi(val);
        self.wb(addr, lo);
        self.wb(addr, hi);
    }
}
