use std::io;
use std::os;

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
    let r = io::read_whole_file(&path);
    match r {
        Ok(t) => {
            println(fmt!("Read %u bytes", t.len()))
        }
        Err(s) => {
            println(fmt!("Cannot read %s: %s", file, s));
            os::set_exit_status(1);
            return
        }
    }
}
