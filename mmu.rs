use std::vec;

use bits::{
    u16_hi,
    u16_lo,
    u16_make,
};
use io::read_port;
use rom::ROM;


enum Mapping<T> {
    Map_Direct(T), // Read and Write to T
    Map_Zero,      // Read 0, Write nothing
    Map_IO(u8),    // I/O on specified port
}

pub struct MMU {
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

    fn vmem<'a> (&'a mut self, addr: u16) -> Mapping<&'a mut u8> {
        match addr {
            0x0000..0x00FF => {
                if self.bios_is_mapped {
                    Map_Direct(&mut self.bios[addr])
                } else {
                    Map_Direct(&mut self.rom.mem[addr])
                }
            }
            0x0100..0x7FFF => { Map_Direct(&mut self.rom.mem[addr]) }
            0x8000..0x9FFF => { Map_Direct(&mut self.vram[addr & 0x1fff]) }
            0xA000..0xBFFF => { Map_Direct(&mut self.eram[addr & 0x1fff]) }
            0xC000..0xFDFF => { Map_Direct(&mut self.wram[addr & 0x1fff]) }
            0xFE00..0xFE9F => { Map_Direct(&mut self.oam[addr & 0xFF]) }
            0xFEA0..0xFEFF => { Map_Zero }
            0xFF00..0xFF7F => { Map_IO(u16_lo(addr)) }
            0xFF80..0xFFFE => { Map_Direct(&mut self.zram[addr & 0x7F]) }
            0xFFFF         => { Map_Direct(&mut self.ie) }
            _ => {
                println(format!("{:04X}", addr as uint));
                fail!("MMU::rb")
            }
        }
    }

    pub fn rb(&mut self, addr: u16) -> u8 {
        match self.vmem(addr) {
            Map_Direct(p) => *p,
            Map_Zero => 0,
            Map_IO(port) => read_port(port),
        }
    }

    pub fn wb(&mut self, addr: u16, val: u8) {
        match self.vmem(addr) {
            Map_Direct(p) => *p = val,
            Map_Zero => {},
            Map_IO(addr) => fail!("Output on port {:04X}", addr as uint),
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
