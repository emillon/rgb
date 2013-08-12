use std::io;
use std::os;
use std::str;

type Mem = ~[u8];

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
    let path = PosixPath(file);
    match io::read_whole_file(&path) {
        Ok(mem) => {
            println(fmt!("Read %u bytes", mem.len()));
            dump_header(mem)
        }
        Err(s) => {
            println(fmt!("Cannot read %s: %s", file, s));
        }
    }
}
