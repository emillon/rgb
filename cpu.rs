use std::cell::Cell;

use bits::{
    u16_hi,
    u16_lo,
    u16_make,
    u16_set_hi,
    u16_set_lo,
};
use mmu::MMU;

enum Flag {
    FZ,
    FN,
    FH,
    FC
}

impl Flag {
    fn mask (&self) -> u8 {
        match *self {
            FZ => 0x80,
            FN => 0x40,
            FH => 0x20,
            FC => 0x10
        }
    }
}

enum R8 {
    R8A,
    R8F,
    R8B,
    R8C,
    R8D,
    R8E,
    R8H,
    R8L,
}

enum ALUOp {
    OpOR,
    OpXOR,
    OpAND,
    OpADD,
    OpADC,
    OpSUB,
    OpSBC,
}

enum AddressingMode {
    AIndirect(u16), // FIXME should be R16
    ADirect(R8),
    AImmediate,
}

pub struct CPU {
    mmu: ~MMU,
    pc: u16,
    reg_af: u16,
    reg_bc: u16,
    reg_de: u16,
    reg_hl: u16,
    reg_sp: u16
}

impl CPU {
    pub fn new(mmu: ~MMU) -> CPU {
        CPU {
            mmu: mmu,
            pc: 0x0,
            reg_af : 0,
            reg_bc : 0,
            reg_de : 0,
            reg_hl : 0,
            reg_sp : 0
        }
    }

    fn r8(&self, r: R8) -> u8 {
        match r {
            R8A => u16_hi(self.reg_af),
            R8F => u16_lo(self.reg_af),
            R8B => u16_hi(self.reg_bc),
            R8C => u16_lo(self.reg_bc),
            R8D => u16_hi(self.reg_de),
            R8E => u16_lo(self.reg_de),
            R8H => u16_hi(self.reg_hl),
            R8L => u16_lo(self.reg_hl),
        }
    }

    fn w8(&mut self, r: R8, v: u8) {
        match r {
            R8A => self.reg_af = u16_set_hi(self.reg_af, v),
            R8F => self.reg_af = u16_set_lo(self.reg_af, v),
            R8B => self.reg_bc = u16_set_hi(self.reg_bc, v),
            R8C => self.reg_bc = u16_set_lo(self.reg_bc, v),
            R8D => self.reg_de = u16_set_hi(self.reg_de, v),
            R8E => self.reg_de = u16_set_lo(self.reg_de, v),
            R8H => self.reg_hl = u16_set_hi(self.reg_hl, v),
            R8L => self.reg_hl = u16_set_lo(self.reg_hl, v),
        }
    }

    fn flag_is_set(&self, f: Flag) -> bool {
        (self.r8(R8F) & f.mask()) != 0
    }

    fn flag_set_bool(&mut self, f: Flag, v: bool) {
        if v {
            self.flag_set(f)
        } else {
            self.flag_reset(f)
        }
    }

    fn flag_set(&mut self, f: Flag) {
        let flags = self.r8(R8F);
        self.w8(R8F, flags | f.mask())
    }

    fn flag_reset(&mut self, f: Flag) {
        let flags = self.r8(R8F);
        self.w8(R8F, flags & (! f.mask()))
    }

    pub fn interp(&mut self) {
        let opcode = self.mmu.rb(self.pc);
        let next_pc = Cell::new(self.pc + 1);
        info!("PC={:04X} OP={:02X}", self.pc as uint, opcode as uint);
        let arg_b = |c: &mut CPU| {
            next_pc.set(next_pc.get() + 1);
            c.mmu.rb(c.pc + 1)
        };
        let arg_w = |c: &mut CPU| {
            next_pc.set(next_pc.get() + 2);
            c.mmu.rw(c.pc + 1)
        };
        let call_cond = |c: &mut CPU, flag| {
            let dest = arg_w(c);
            if c.flag_is_set(flag) {
                next_pc.set(dest)
            }
        };
        let jr_cond = |c: &mut CPU, flag, exp_value| {
            let offset = arg_b(c);
            if c.flag_is_set(flag) == exp_value {
                next_pc.set(next_pc.get() + offset as u16)
            }
        };
        let alu_op = |c:&mut CPU, op: ALUOp, what: Option<R8>| {
            let a = c.r8(R8A);
            let y = match what {
                Some(reg) => c.r8(reg),
                None => arg_b(c)
            };
            let carry = |c: &mut CPU| {
                if c.flag_is_set(FC) {
                    1
                } else {
                    0
                }
            };
            let z = match op {
                OpOR  => a | y,
                OpXOR => a ^ y,
                OpAND => a & y,
                OpADD => a + y,
                OpADC => a + y + carry(c),
                OpSUB => a - y,
                OpSBC => a - y - carry(c),
            };
            c.w8(R8A, z);
            let a2 = c.r8(R8A);
            let honor_z = |c: &mut CPU| {
                c.flag_set_bool(FZ, a2 == 0);
            };
            match op {
                OpOR | OpXOR => {
                    honor_z(c);
                    c.flag_reset(FN);
                    c.flag_reset(FH);
                    c.flag_reset(FC);
                }
                OpAND => {
                    honor_z(c);
                    c.flag_reset(FN);
                    c.flag_set(FH);
                    c.flag_reset(FC);
                }
                OpADD | OpADC => {
                    honor_z(c);
                    c.flag_reset(FN);
                    // TODO honor h
                    // TODO honor c
                }
                OpSUB | OpSBC => {
                    honor_z(c);
                    c.flag_set(FN);
                    // TODO honor h
                    // TODO honor c
                }
            }
        };
        let ld8 = |c: &mut CPU, dest, src: AddressingMode| {
            let val = match src {
                AIndirect(addr) => c.mmu.rb(addr),
                ADirect(reg) => c.r8(reg),
                AImmediate => arg_b(c),
            };
            c.w8(dest, val)
        };
        let ld8_ind = |c: &mut CPU, dest, src| { // LD (dest), src
            let v = c.r8(src);
            c.mmu.wb(dest, v);
        };
        let push_w = |c: &mut CPU, val| {
            c.reg_sp -= 2;
            c.mmu.ww(c.reg_sp, val);
        };
        let pop_w = |c: &mut CPU| {
            let val = c.mmu.rw(c.reg_sp);
            c.reg_sp += 2;
            val
        };
        let ret : |&mut CPU| = |c| {
            next_pc.set(pop_w(c))
        };
        let ret_cond = |c: &mut CPU, flag, expected| {
            if c.flag_is_set(flag) == expected {
                ret(c)
            }
        };
        match opcode {
            0x00 => { // NOP
            }
            0x01 => { // LD BC, nn nn
                let a = arg_w(self);
                self.reg_bc = a;
            }
            0x03 => { // INC BC
                self.reg_bc += 1;
            }
            0x08 => { // LD (nn nn), SP
                let addr = arg_w(self);
                self.mmu.ww(addr, self.reg_sp);
            }
            0x0B => { // DEC BC
                self.reg_bc -= 1;
            }
            0x0C => { // INC C
                let c = self.r8(R8C);
                self.w8(R8C, c + 1);
                let c2 = self.r8(R8C);
                self.flag_set_bool(FZ, c2 == 0);
                self.flag_reset(FN);
                // TODO FH
            }
            0x0D => { // DEC C
                let c = self.r8(R8C);
                self.w8(R8C, c-1);
                let c2 = self.r8(R8C);
                self.flag_set_bool(FZ, c2 == 0);
                self.flag_set(FN)
                // TODO FH
            }
            0x0E => { // LD C, nn
                ld8(self, R8C, AImmediate);
            }
            0x0F => { // RRCA
                let a = self.r8(R8A);
                let lsb = a & 0x01;
                let new = (a >> 1) & (lsb << 7);
                self.w8(R8A, new);
                self.flag_reset(FZ);
                self.flag_reset(FN);
                self.flag_reset(FH);
                self.flag_set_bool(FC, lsb != 0);
            }
            0x11 => { // LD DE, nn nn
                self.reg_de = arg_w(self);
            }
            0x12 => { // LD (DE), A
                ld8_ind(self, self.reg_de, R8A)
            }
            0x1F => { // RRA
                let carry = if self.flag_is_set(FC) {
                    1
                } else {
                    0
                };
                let a = self.r8(R8A);
                let lsb = a & 0x01;
                self.flag_set_bool(FC, lsb != 0);
                let new = (a >> 1) & (carry << 7);
                self.w8(R8A, new);
            }
            0x20 => { // JR NZ, nn
                jr_cond(self, FZ, false)
            }
            0x21 => { // LD HL, nn nn
                self.reg_hl = arg_w(self)
            }
            0x22 => { // LDI (HL), A
                let a = self.r8(R8A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl += 1;
            }
            0x28 => { // JR Z, nn
                jr_cond(self, FZ, true)
            }
            0x2E => { // LD L, nn
                ld8(self, R8L, AImmediate)
            }
            0x30 => { // JR NC, nn
                jr_cond(self, FC, false)
            }
            0x31 => { // LD SP, nn nn
                self.reg_sp = arg_w(self)
            }
            0x32 => { // LDD (HL), A
                let a = self.r8(R8A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl -= 1;
            }
            0x38 => { // JR C, nn
                jr_cond(self, FC, true)
            }
            0x3E => { // LD A, nn
                ld8(self, R8A, AImmediate)
            }
            0x66 => { // LD H, (HL)
                let val = self.mmu.rb(self.reg_hl);
                self.w8(R8H, val)
            }
            0x73 => { // LD (HL), E
                ld8_ind(self, self.reg_hl, R8E);
            }
            0x79 => { // LD A, C
                ld8(self, R8A, ADirect(R8C))
            }
            0x7E => { // LD A,(HL)
                ld8(self, R8A, AIndirect(self.reg_hl))
            }
            0x83 => { // ADD A, E
                alu_op(self, OpADD, Some(R8E))
            }
            0x88 => { // ADC A, B
                alu_op(self, OpADC, Some(R8B));
            }
            0x89 => { // ADC A, C
                alu_op(self, OpADC, Some(R8C));
            }
            0x93 => { // SUB E
                alu_op(self, OpSUB, Some(R8E));
            }
            0x9A => { // SBC D
                alu_op(self, OpSBC, Some(R8D));
            }
            0xAF => { // XOR A
                alu_op(self, OpXOR, Some(R8A));
            }
            0xB0 => { // OR B
                alu_op(self, OpOR, Some(R8B));
            }
            0xB7 => { // OR A
                alu_op(self, OpOR, Some(R8A));
            }
            0xC0 => { // RET NZ
                ret_cond(self, FZ, false);
            }
            0xC3 => { // JP nn nn
                let dest = self.mmu.rw(self.pc + 1);
                next_pc.set(dest)
            }
            0xC8 => { // RET Z
                ret_cond(self, FZ, true);
            }
            0xC9 => { // RET
                ret(self);
            }
            0xCB => { // ext ops
                let op = arg_b(self);
                match op {
                    0x7C => { // BIT 7, H
                        let h = self.r8(R8H);
                        self.flag_set_bool(FZ, (h & (1 << 7)) == 0);
                        self.flag_reset(FN);
                        self.flag_set(FH);
                    }
                    _ => fail!("Unknown ext op : CB {:02X}", op as uint)
                }
            }
            0xCC => { // CALL Z, nn nn
                call_cond(self, FZ)
            }
            0xCD => { // CALL nn nn
                push_w(self, self.pc);
                let dest = arg_w(self); // FIXME
                next_pc.set(dest)
            }
            0xD5 => { // PUSH DE
                push_w(self, self.reg_de)
            }
            0xDC => { // CALL C, nn nn
                call_cond(self, FC)
            }
            0xDD => {
                fail!("Bad opcode : {:02X}", opcode as uint)
            }
            0xDE => { // SBC nn
                alu_op(self, OpSBC, None)
            }
            0xE2 => { // LD (FF00+C), A
                let a = self.r8(R8A);
                let c = self.r8(R8C);
                let addr = u16_make(0xFF, c);
                self.mmu.wb(addr, a);
            }
            0xE5 => { // PUSH HL
                push_w(self, self.reg_hl)
            }
            0xE6 => { // AND nn
                alu_op(self, OpAND, None)
            }
            0xF6 => { // OR nn
                alu_op(self, OpOR, None)
            }
            0xFA => { // LD A, (nn nn)
                let addr = arg_w(self);
                let val = self.mmu.rb(addr);
                self.w8(R8A, val);
            }
            0xFE => { // CP A, nn
                let n = arg_b(self);
                let a = self.r8(R8A);
                self.flag_set_bool(FZ, a == n);
                self.flag_set(FN);
                // TODO FH
                // TODO FC
            }
            0xF3 => { // DI
                self.mmu.interrupts_disable()
            }
            0xFF => { // RST 0x38
                // TODO save regs
                push_w(self, self.pc);
                next_pc.set(0x38);
            }
            _ => {
                fail!("Unknown opcode : {:02X}", opcode as uint)
            }
        }
        if self.pc < 0x0100 && next_pc.get() >= 0x0100 {
            // Jumping out of bios
            self.mmu.bios_is_mapped = false;
        }
        // TODO detect PC wrap
        self.pc = next_pc.get();
    }
}
