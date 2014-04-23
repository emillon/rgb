use std::str;

pub struct ROM {
    pub mem: ~[u8]
}

impl ROM {
    pub fn new(rom_data: &[u8]) -> ROM {
        ROM { mem: rom_data.to_owned() }
    }

    fn read_word_be(&self, offset: u16) -> u16 {
        let offu = offset as uint;
        (self.mem[offu] as u16 << 8) + self.mem[offu + 1] as u16
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

    pub fn dump_header (&self) {
        let title = str::from_utf8(self.mem.slice(0x134, 0x143)).expect("Cannot decode title");
        let ct = self.mem[0x147];
        println!("Title: {:s}", title);
        println!("Type : {:u}", ct as uint);
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
        println!("HDR Checksum: {:02X} ({:s})", stored_hdr_checksum as uint, hdr_ck_str);
        println!("ROM Checksum: {:04X} ({:s})", stored_rom_checksum as uint, rom_ck_str);
    }
}
