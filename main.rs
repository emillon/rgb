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
    let rom_data = io::File::open(&Path::new(file)).read_to_end();
    let rom = ~ROM::new(rom_data);
    rom.dump_header();
    let bios = io::File::open(&Path::new("bios.dat")).read_to_end();
    let mmu = MMU::new(rom, bios);
    let mut cpu = CPU::new(~mmu);
    loop {
        cpu.interp()
    }
}
