use std::io;
use std::os;

use cpu::CPU;
use mmu::MMU;
use rom::ROM;

#[main]
fn main() {
    println("rgb");
    let args = os::args();
    if args.len() <= 1 {
        println("Usage: rgb file.gb");
        os::set_exit_status(1);
        return
    }
    let file = os::args()[1];
    let rom_data = io::read_whole_file(&PosixPath(file)).expect(fmt!("Cannot read %s", file));
    let rom = ~ROM::new(rom_data);
    rom.dump_header();
    let bios = io::read_whole_file(&PosixPath("bios.dat")).expect("Cannot open bios");
    let mmu = MMU::new(rom, bios);
    let mut cpu = CPU::new(~mmu);
    loop {
        cpu.interp()
    }
}
