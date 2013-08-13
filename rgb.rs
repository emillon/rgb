use std::io;
use std::os;
use std::str;

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

    fn dump_header (self) {
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
            r.dump_header()
        }
        Err (s) => {
            println(fmt!("Cannot read %s: %s", file, s));
        }
    }
}
