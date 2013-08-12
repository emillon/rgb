use std::io;
use std::os;
use std::str;

type Mem = ~[u8];

fn load_rom (file: &str) -> Result<Mem, ~str> {
    let path = PosixPath(file);
    let r = io::read_whole_file(&path);
    match r {
        Ok(m) => {
            println(fmt!("Read %u bytes", m.len()));
            return Ok(m)
        }
        Err(s) => {
            return Err(s)
        }
    }
}

fn read_mem (m: &Mem, n: u16) -> u8 {
    m[n]
}

fn write_mem (m: &mut Mem, n: u16, v: u8) {
    m[n] = v
}

fn dump_header (m: Mem) {
    let title = m.slice(0x134, 0x143);
    println(str::from_bytes(title))
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
    match load_rom(file) {
        Ok(mem) => {
            dump_header(mem)
        }
        Err(s) => {
            println(fmt!("Cannot read %s: %s", file, s));
        }
    }
}
