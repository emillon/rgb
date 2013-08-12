use std::io;
use std::os;

type Mem = ~[u8];

fn load_rom (file: &str, m: &Mem) -> Result<(), ~str> {
    let path = PosixPath(file);
    let r = io::read_whole_file(&path);
    match r {
        Ok(t) => {
            println(fmt!("Read %u bytes", t.len()));
            return Ok(())
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

fn main() {
    println("rgb");
    let args = os::args();
    if args.len() <= 1 {
        println("Usage: rgb file.gb");
        os::set_exit_status(1);
        return
    }
    let file = os::args()[1];
    let mem = ~[];
    match load_rom(file, &mem) {
        Ok(()) => {}
        Err(s) => {
            println(fmt!("Cannot read %s: %s", file, s));
        }
    }
}
