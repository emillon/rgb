use bits::*;
use mmu::MMU;

enum Flag {
    F_Z,
    F_N,
    F_H,
    F_C
}

impl Flag {
    fn mask (&self) -> u8 {
        match *self {
            F_Z => 0x80,
            F_N => 0x40,
            F_H => 0x20,
            F_C => 0x10
        }
    }
}

enum R8 {
    R8_A,
    R8_F,
    R8_B,
    R8_C,
    R8_D,
    R8_E,
    R8_H,
    R8_L,
}

enum ALU_Op {
    Op_OR,
    Op_XOR,
    Op_AND,
    Op_ADD,
    Op_ADC,
}

enum Addressing_Mode {
    A_Indirect(u16), // FIXME should be R16
    A_Direct(R8),
    A_Immediate,
}

struct CPU {
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
            R8_A => u16_hi(self.reg_af),
            R8_F => u16_lo(self.reg_af),
            R8_B => u16_hi(self.reg_bc),
            R8_C => u16_lo(self.reg_bc),
            R8_D => u16_hi(self.reg_de),
            R8_E => u16_lo(self.reg_de),
            R8_H => u16_hi(self.reg_hl),
            R8_L => u16_lo(self.reg_hl),
        }
    }

    fn w8(&mut self, r: R8, v: u8) {
        match r {
            R8_A => self.reg_af = u16_set_hi(self.reg_af, v),
            R8_F => self.reg_af = u16_set_lo(self.reg_af, v),
            R8_B => self.reg_bc = u16_set_hi(self.reg_bc, v),
            R8_C => self.reg_bc = u16_set_lo(self.reg_bc, v),
            R8_D => self.reg_de = u16_set_hi(self.reg_de, v),
            R8_E => self.reg_de = u16_set_lo(self.reg_de, v),
            R8_H => self.reg_hl = u16_set_hi(self.reg_hl, v),
            R8_L => self.reg_hl = u16_set_lo(self.reg_hl, v),
        }
    }

    fn flag_is_set(&self, f: Flag) -> bool {
        (self.r8(R8_F) & f.mask()) != 0
    }

    fn flag_is_reset(&self, f: Flag) -> bool {
        !self.flag_is_set(f)
    }

    fn flag_set_bool(&mut self, f: Flag, v: bool) {
        if v {
            self.flag_set(f)
        } else {
            self.flag_reset(f)
        }
    }

    fn flag_set(&mut self, f: Flag) {
        let flags = self.r8(R8_F);
        self.w8(R8_F, flags | f.mask())
    }

    fn flag_reset(&mut self, f: Flag) {
        let flags = self.r8(R8_F);
        self.w8(R8_F, flags & (! f.mask()))
    }

    pub fn interp(&mut self) {
        let opcode = self.mmu.rb(self.pc);
        let mut next_pc = self.pc + 1;
        info!("PC=%04X OP=%02X", self.pc as uint, opcode as uint);
        let arg_b = || {
            next_pc += 1;
            self.mmu.rb(self.pc + 1)
        };
        let arg_w = || {
            next_pc += 2;
            self.mmu.rw(self.pc + 1)
        };
        let call_cond = |flag| {
            let dest = arg_w();
            if self.flag_is_set(flag) {
                next_pc = dest
            }
        };
        let jr_cond = |flag, exp_value| {
            let offset = arg_b();
            if self.flag_is_set(flag) == exp_value {
                next_pc += offset as u16
            }
        };
        let alu_op = |op: ALU_Op, what: Option<R8>| {
            let a = self.r8(R8_A);
            let y = match what {
                Some(reg) => self.r8(reg),
                None => arg_b()
            };
            let z = match op {
                Op_OR  => a | y,
                Op_XOR => a ^ y,
                Op_AND => a & y,
                Op_ADD => a + y,
                Op_ADC => a + y + (if self.flag_is_set(F_C) { 1 } else { 0 })
            };
            self.w8(R8_A, z);
            let a2 = self.r8(R8_A);
            let honor_z = || {
                self.flag_set_bool(F_Z, a2 == 0);
            };
            match op {
                Op_OR | Op_XOR => {
                    honor_z();
                    self.flag_reset(F_N);
                    self.flag_reset(F_H);
                    self.flag_reset(F_C);
                }
                Op_AND => {
                    honor_z();
                    self.flag_reset(F_N);
                    self.flag_set(F_H);
                    self.flag_reset(F_C);
                }
                Op_ADD | Op_ADC => {
                    honor_z();
                    self.flag_reset(F_N);
                    // TODO honor h
                    // TODO honor c
                }
            }
        };
        let ld8 = |dest, src: Addressing_Mode| {
            let val = match src {
                A_Indirect(addr) => self.mmu.rb(addr),
                A_Direct(reg) => self.r8(reg),
                A_Immediate => arg_b(),
            };
            self.w8(dest, val)
        };
        let ld8_ind = |dest, src| { // LD (dest), src
            let v = self.r8(src);
            self.mmu.wb(dest, v);
        };
        let push_w = |val| {
            self.reg_sp -= 2;
            self.mmu.ww(self.reg_sp, val);
        };
        let pop_w = || {
            let val = self.mmu.rw(self.reg_sp);
            self.reg_sp += 2;
            val
        };
        let ret = || {
            next_pc = pop_w();
        };
        let ret_cond = |flag, expected| {
            if self.flag_is_set(flag) == expected {
                ret()
            }
        };
        match opcode {
            0x00 => { // NOP
            }
            0x01 => { // LD BC, nn nn
                self.reg_bc = arg_w();
            }
            0x03 => { // INC BC
                self.reg_bc += 1;
            }
            0x08 => { // LD (nn nn), SP
                let addr = arg_w();
                self.mmu.ww(addr, self.reg_sp);
            }
            0x0B => { // DEC BC
                self.reg_bc -= 1;
            }
            0x0C => { // INC C
                let c = self.r8(R8_C);
                self.w8(R8_C, c + 1);
                let c2 = self.r8(R8_C);
                self.flag_set_bool(F_Z, c2 == 0);
                self.flag_reset(F_N);
                // TODO F_H
            }
            0x0D => { // DEC C
                let c = self.r8(R8_C);
                self.w8(R8_C, c-1);
                let c2 = self.r8(R8_C);
                self.flag_set_bool(F_Z, c2 == 0);
                self.flag_set(F_N)
                // TODO F_H
            }
            0x0E => { // LD C, nn
                ld8(R8_C, A_Immediate);
            }
            0x0F => { // RRCA
                let a = self.r8(R8_A);
                let lsb = a & 0x01;
                let new = (a >> 1) & (lsb << 7);
                self.w8(R8_A, new);
                self.flag_reset(F_Z);
                self.flag_reset(F_N);
                self.flag_reset(F_H);
                self.flag_set_bool(F_C, lsb != 0);
            }
            0x12 => { // LD (DE), A
                ld8_ind(self.reg_de, R8_A)
            }
            0x20 => { // JR NZ, nn
                jr_cond(F_Z, false)
            }
            0x21 => { // LD HL, nn nn
                self.reg_hl = arg_w()
            }
            0x22 => { // LDI (HL), A
                let a = self.r8(R8_A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl += 1;
            }
            0x2E => { // LD L, nn
                ld8(R8_L, A_Immediate)
            }
            0x30 => { // JR NC, nn
                jr_cond(F_C, false)
            }
            0x31 => { // LD SP, nn nn
                self.reg_sp = arg_w()
            }
            0x32 => { // LDD (HL), A
                let a = self.r8(R8_A);
                self.mmu.wb(self.reg_hl, a);
                self.reg_hl -= 1;
            }
            0x38 => { // JR C, nn
                jr_cond(F_C, true)
            }
            0x3E => { // LD A, nn
                ld8(R8_A, A_Immediate)
            }
            0x66 => { // LD H, (HL)
                let val = self.mmu.rb(self.reg_hl);
                self.w8(R8_H, val)
            }
            0x73 => { // LD (HL), E
                ld8_ind(self.reg_hl, R8_E);
            }
            0x79 => { // LD A, C
                ld8(R8_A, A_Direct(R8_C))
            }
            0x7E => { // LD A,(HL)
                ld8(R8_A, A_Indirect(self.reg_hl))
            }
            0x83 => { // ADD A, E
                alu_op(Op_ADD, Some(R8_E))
            }
            0x88 => { // ADC A, B
                alu_op(Op_ADC, Some(R8_B));
            }
            0x89 => { // ADC A, C
                alu_op(Op_ADC, Some(R8_C));
            }
            0xAF => { // XOR A
                alu_op(Op_XOR, Some(R8_A));
            }
            0xB0 => { // OR B
                alu_op(Op_OR, Some(R8_B));
            }
            0xC0 => { // RET NZ
                ret_cond(F_Z, false);
            }
            0xC3 => { // JP nn nn
                let dest = self.mmu.rw(self.pc + 1);
                next_pc = dest
            }
            0xC8 => { // RET Z
                ret_cond(F_Z, true);
            }
            0xC9 => { // RET
                ret()
            }
            0xCB => { // ext ops
                let op = arg_b();
                match op {
                    0x7C => { // BIT 7, H
                        let h = self.r8(R8_H);
                        self.flag_set_bool(F_Z, (h & (1 << 7)) == 0);
                        self.flag_reset(F_N);
                        self.flag_set(F_H);
                    }
                    _ => fail!("Unknown ext op : CB %02X", op as uint)
                }
            }
            0xCC => { // CALL Z, nn nn
                call_cond(F_Z)
            }
            0xCD => { // CALL nn nn
                push_w(self.pc);
                let dest = arg_w(); // FIXME
                next_pc = dest
            }
            0xD5 => { // PUSH DE
                push_w(self.reg_de)
            }
            0xDC => { // CALL C, nn nn
                call_cond(F_C)
            }
            0xDD => {
                fail!("Bad opcode : %02X", opcode as uint)
            }
            0xE2 => { // LD (FF00+C), A
                let a = self.r8(R8_A);
                let c = self.r8(R8_C);
                let addr = u16_make(0xFF, c);
                self.mmu.wb(addr, a);
            }
            0xE5 => { // PUSH HL
                push_w(self.reg_hl)
            }
            0xE6 => { // AND nn
                alu_op(Op_AND, None)
            }
            0xF6 => { // OR nn
                alu_op(Op_OR, None)
            }
            0xFE => { // CP A, nn
                let n = arg_b();
                let a = self.r8(R8_A);
                self.flag_set_bool(F_Z, a == n);
                self.flag_set(F_N);
                // TODO F_H
                // TODO F_C
            }
            0xF3 => { // DI
                self.mmu.interrupts_disable()
            }
            0xFF => { // RST 0x38
                // TODO save regs
                push_w(self.pc);
                next_pc = 0x38;
            }
            _ => {
                fail!("Unknown opcode : %02X", opcode as uint)
            }
        }
        if (self.pc < 0x0100 && next_pc >= 0x0100) {
            // Jumping out of bios
            self.mmu.bios_is_mapped = false
        }
        // TODO detect PC wrap
        self.pc = next_pc;
    }
}
