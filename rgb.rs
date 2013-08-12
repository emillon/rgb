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

    fn dump_header (self) {
        let title = self.mem.slice(0x134, 0x143);
        println(str::from_bytes(title))
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
