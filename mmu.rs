use bits::{
    u16_hi,
    u16_lo,
    u16_make,
};
use io::read_port;
use rom::ROM;


enum Mapping<T> {
    MapDirect(T), // Read and Write to T
    MapZero,      // Read 0, Write nothing
    MapIO(u8),    // I/O on specified port
}

pub struct MMU {
    pub bios_is_mapped: bool,
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
    pub fn new(rom: ~ROM, bios: &[u8]) -> MMU {
        MMU {
            bios_is_mapped: true,
            bios: bios.to_owned(),
            eram: ~[], // FIXME
            ie: 0,
            oam: ~[0, ..160],
            rom: rom,
            vram: ~[0, ..8192],
            wram: ~[0, ..8192],
            zram: ~[0, ..127],
        }
    }

    pub fn interrupts_disable(&mut self) {
        self.ie = 0
    }

    fn vmem<'a> (&'a mut self, addr: u16) -> Mapping<&'a mut u8> {
        let addru = addr as uint;
        match addr {
            0x0000..0x00FF => {
                if self.bios_is_mapped {
                    MapDirect(&mut self.bios[addru])
                } else {
                    MapDirect(&mut self.rom.mem[addru])
                }
            }
            0x0100..0x7FFF => { MapDirect(&mut self.rom.mem[addru]) }
            0x8000..0x9FFF => { MapDirect(&mut self.vram[addru & 0x1fff]) }
            0xA000..0xBFFF => { MapDirect(&mut self.eram[addru & 0x1fff]) }
            0xC000..0xFDFF => { MapDirect(&mut self.wram[addru & 0x1fff]) }
            0xFE00..0xFE9F => { MapDirect(&mut self.oam[addru & 0xFF]) }
            0xFEA0..0xFEFF => { MapZero }
            0xFF00..0xFF7F => { MapIO(u16_lo(addr)) }
            0xFF80..0xFFFE => { MapDirect(&mut self.zram[addru & 0x7F]) }
            0xFFFF         => { MapDirect(&mut self.ie) }
            _ => {
                println!("{:04X}", addr as uint);
                fail!("MMU::rb")
            }
        }
    }

    pub fn rb(&mut self, addr: u16) -> u8 {
        match self.vmem(addr) {
            MapDirect(p) => *p,
            MapZero => 0,
            MapIO(port) => read_port(port),
        }
    }

    pub fn wb(&mut self, addr: u16, val: u8) {
        match self.vmem(addr) {
            MapDirect(p) => *p = val,
            MapZero => {},
            MapIO(addr) => fail!("Output on port {:04X}", addr as uint),
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
