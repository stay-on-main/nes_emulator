enum StatusBit {
    Carry = 1 << 0,
    Zero = 1 << 1,
    Interrupt = 1 << 2,
    Decimal = 1 << 3,
    Break = 1 << 4,
    Overflow = 1 << 6,
    Sign = 1 << 7,
}

pub struct Cpu {
    a: u8,
    pc: u16,
    s: u8,
    x: u8,
    y: u8,
    p: u8,
    bus: Bus,

    op: u8,
    op_addr: u16,
}

impl Cpu {
    fn fetch_u8(&mut self) -> u8 {
        let b = self.bus.read(self.pc);
        self.pc += 1;
        b
    }

    fn get_bit(&self, bit: StatusBit) -> bool {
        self.p & (bit as u8) != 0
    }

    fn update_bit(&mut self, bit: StatusBit, op: u8) {
        match bit {
            StatusBit::Zero => {
                if op == 0 {
                    self.p |= bit as u8;
                } else {
                    self.p &= !(bit as u8);
                }
            },
            StatusBit::Sign => {
                if (op & 0x80) != 0 {
                    self.p |= bit as u8;
                } else {
                    self.p &= !(bit as u8);
                }
            },
            _ => {
                if op != 0 {
                    self.p |= bit as u8;
                } else {
                    self.p &= !(bit as u8);
                }
            },
        }
    }

    fn update_bit_sign_and_zero(&mut self, byte: u8) {
        self.update_bit(StatusBit::Sign, byte);
        self.update_bit(StatusBit::Zero, byte);
    }

    fn stack_push(&mut self, b: u8) {
        self.bus.write(0x100 + self.s as u16, b);
    }

    fn stack_pull(&mut self) -> u8 {
        self.bus.read(0x100 + self.s as u16)
    }

    fn adc(&mut self) {
        let mut temp = (self.a as u16) + (self.op as u16);

        if self.get_bit(StatusBit::Carry) {
            temp += 1;
        }
        // The overflow flag is set when
        // the sign of the addends is the same and
        // differs from the sign of the sum
        self.update_bit(StatusBit::Overflow, !(self.a ^ self.op) & (self.a ^ (temp as u8)) & 0x80);
        self.update_bit(StatusBit::Carry, if temp > 0xFF {1} else {0});
        self.op = temp as u8;
        self.update_bit_sign_and_zero(self.op);
        self.a = self.op;
    }

    fn and(&mut self) {
        self.op &= self.a;
        self.update_bit_sign_and_zero(self.op);
        self.a = self.op;
    }

    fn cmp(&mut self) {
        let src = self.a.wrapping_sub(self.op);
        self.update_bit(StatusBit::Carry, if self.a >= self.op {1} else {0});
        self.update_bit_sign_and_zero(src);
    }

    fn cpy(&mut self) {
        let src = self.y.wrapping_sub(self.op);
        self.update_bit(StatusBit::Carry, if self.y >= self.op {1} else {0});
        self.update_bit_sign_and_zero(src);
    }

    fn cpx(&mut self) {
        let src = self.x.wrapping_sub(self.op);
        self.update_bit(StatusBit::Carry, if self.x >= self.op {1} else {0});
        self.update_bit_sign_and_zero(src);
    }

    fn eor(&mut self) {
        self.a ^= self.op;
        self.update_bit_sign_and_zero(self.a);
    }

    fn lda(&mut self) {
        self.a = self.op;
        self.update_bit_sign_and_zero(self.a);
    }

    fn ldy(&mut self) {
        self.y = self.op;
        self.update_bit_sign_and_zero(self.y);
    }

    fn ldx(&mut self) {
        self.x = self.op;
        self.update_bit_sign_and_zero(self.x);
    }

    fn ora(&mut self) {
        self.a |= self.op;
        self.update_bit_sign_and_zero(self.a);
    }

    fn sbc(&mut self) {
        let mut temp = (self.a as i32) - (self.op as i32);
        
        if !self.get_bit(StatusBit::Carry) {
            temp -= 1;
        }

        self.update_bit(StatusBit::Overflow, ((self.a ^ (temp as u8)) & 0x80) & ((self.a ^ self.op) & 0x80));
        self.update_bit(StatusBit::Carry, if self.a >= self.op {1} else {0});
        self.a = temp as u8;
        self.update_bit_sign_and_zero(self.a);
    }

    fn lax(&mut self) {
        self.lda();
        self.tax();
    }

    fn bit(&mut self) {
        self.update_bit(StatusBit::Sign, self.op);
        self.update_bit(StatusBit::Overflow, 0x40 & self.op);
        self.update_bit(StatusBit::Zero, self.op & self.a);
    }

    fn skb(&mut self) { }

    fn ign(&mut self) { }

    fn nop(&mut self) { }

    fn dex(&mut self) {
        self.x = self.x.wrapping_sub(1);
        self.update_bit_sign_and_zero(self.x);
    }

    fn dey(&mut self) {
        self.y = self.y.wrapping_sub(1);
        self.update_bit_sign_and_zero(self.y);
    }

    fn iny(&mut self) {
        self.y = self.y.wrapping_add(1);
        self.update_bit_sign_and_zero(self.y);
    }

    fn inx(&mut self) {
        self.x = self.x.wrapping_add(1);
        self.update_bit_sign_and_zero(self.x);
    }

    fn clc(&mut self) {
        self.update_bit(StatusBit::Carry, 0x00);
    }

    fn cld(&mut self) {
        self.update_bit(StatusBit::Decimal, 0x00);
    }

    fn cli(&mut self) {
        self.update_bit(StatusBit::Interrupt, 0x00);
    }

    fn clv(&mut self) {
        self.update_bit(StatusBit::Overflow, 0x00);
    }

    fn sec(&mut self) {
        self.update_bit(StatusBit::Carry, 0xff);
    }

    fn sed(&mut self) {
        self.update_bit(StatusBit::Decimal, 0xff);
    }

    fn sei(&mut self) {
        self.update_bit(StatusBit::Interrupt, 0xff);
    }

    fn tax(&mut self) {
        self.x = self.a;
        self.update_bit_sign_and_zero(self.x);
    }

    fn tay(&mut self) {
        self.y = self.a;
        self.update_bit_sign_and_zero(self.y);
    }

    fn tsx(&mut self) {
        self.x = self.s;
        self.update_bit_sign_and_zero(self.x);
    }

    fn txa(&mut self) {
        self.a = self.x;
        self.update_bit_sign_and_zero(self.a);
    }

    fn txs(&mut self) {
        self.s = self.x;
    }

    fn tya(&mut self) {
        self.a = self.y;
        self.update_bit_sign_and_zero(self.a);
    }

    fn sta(&mut self) {
        self.bus.write(self.op_addr, self.a);
    }

    fn stx(&mut self) {
        self.bus.write(self.op_addr, self.x);
    }

    fn sty(&mut self) {
        self.bus.write(self.op_addr, self.y);
    }

    fn sax(&mut self) {
        self.op = self.x & self.a;
        self.bus.write(self.op_addr, self.op);
    }

    fn dec(&mut self) {
        self.op = self.op.wrapping_sub(1);
        self.update_bit_sign_and_zero(self.op);
        self.bus.write(self.op_addr, self.op);
    }

    fn dcp(&mut self) {
        self.dec();
        self.cmp();
    }

    fn rra(&mut self) {
        self.ror();
        self.adc();
    }

    fn ror(&mut self) {
        let carry = self.op & 0x01;
        self.op >>= 1;
        
        if self.get_bit(StatusBit::Carry) {
            self.op |= 0x80;
        }
        
        self.update_bit(StatusBit::Carry, carry);
        self.update_bit_sign_and_zero(self.op);
        self.bus.write(self.op_addr, self.op);
    }

    fn rol(&mut self) {
        let carry = self.op & 0x80;
        self.op <<= 1;

        if self.get_bit(StatusBit::Carry) {
            self.op |= 0x01;
        }

        self.update_bit(StatusBit::Carry, carry);
        self.update_bit_sign_and_zero(self.op);
        self.bus.write(self.op_addr, self.op);
    }

    fn slo(&mut self) {
        self.asl();
        self.ora();
    }

    fn sre(&mut self) {
        self.lsr();
        self.eor();
    }

    fn isb(&mut self) {
        self.inc();
        self.sbc();
    }

    fn asl(&mut self) {
        self.update_bit(StatusBit::Carry, self.op & 0x80);
        self.op <<= 1;
        self.update_bit_sign_and_zero(self.op);
        self.a = self.op;
    }

    fn inc(&mut self) {
        self.op = self.op.wrapping_add(1);
        self.update_bit_sign_and_zero(self.op);
        self.bus.write(self.op_addr, self.op);
    }

    fn lsr(&mut self) {
        self.update_bit(StatusBit::Carry, self.op & 0x01);
        self.op >>= 1;
        self.update_bit_sign_and_zero(self.op);
        self.a = self.op;
    }

    fn rla(&mut self) {
        self.rol();
        self.and();
    }
    //=============================
    fn acc(&mut self) {
        self.op = self.a;
    }
    
    fn imp(&mut self) {

    }
    
    fn imm(&mut self) {
        self.op = self.fetch_u8();
    }

    fn abs(&mut self) {
        let l = self.fetch_u8() as u16;
        let h = self.fetch_u8() as u16;
        self.op_addr = (h << 8) | l;
        self.op = self.bus.read(self.op_addr);
    }
    
    fn zp(&mut self) {
        self.op_addr = self.fetch_u8() as u16;
        self.op = self.bus.read(self.op_addr);
    }

    fn zp_x(&mut self) {
        self.op_addr  = self.fetch_u8().wrapping_add(self.x) as u16;
        self.op = self.bus.read(self.op_addr);
    }

    fn zp_y(&mut self) {
        self.op_addr  = self.fetch_u8().wrapping_add(self.y) as u16;
        self.op = self.bus.read(self.op_addr);
    }
    
    fn abs_index(&mut self, index: u8) {
        let l = self.fetch_u8();
        // 3
        let mut h = self.fetch_u8() as u16;
        let overflow = (l as u16 + index as u16) > 0xff;
        let l = l.wrapping_add(index) as u16;
        // 4
        // The high byte of the effective address may be invalid
        // at this time, i.e. it may be smaller by $100.
        if overflow {
            // page boundary was crossed
            h += 1;
            //self.bus.clock();
        }
        // 5
        self.op_addr = (h << 8) | l;
        self.op = self.bus.read(self.op_addr);
    }

    fn abs_x(&mut self) {
        self.abs_index(self.x);
    }

    fn abs_y(&mut self) {
        self.abs_index(self.y);
    }

    // BCC, BCS, BNE, BEQ, BPL, BMI, BVC, BVS
    fn rel(&mut self) {
        // 2
        let operand = self.fetch_u8() as i8;
        //self.bus.read(self.pc);
        //if condition {
            // 3
            //self.bus.clock();
            let new_pc = (self.pc as i32 + operand as i32) as u16;

            if new_pc >> 8 != self.pc >> 8 {
                //self.bus.clock();
            }

            self.op_addr = new_pc;
            //self.pcl += operand;
        //}
    }
    
    fn ind_x(&mut self) {
        // 2
        let addr = self.fetch_u8();
        // 3
        let addr = addr.wrapping_add(self.x);
        // 4
        let l = self.bus.read(addr as u16) as u16;
        // 5
        let h = self.bus.read(addr.wrapping_add(1) as u16) as u16;
        // 6
        self.op_addr = (h << 8) | l;
        self.op = self.bus.read(addr);
    }
    
    fn ind_y(&mut self) {
        // 2
        let pointer_addr = self.fetch_u8() as u16;
        // 3
        let pointer_l = self.bus.read(pointer_addr);
        // 4
        let mut pointer_h = self.bus.read((pointer_addr + 1) & 0xff) as u16;
        let overflow = (pointer_l as u16 + self.y as u16) > 0xff;
        let pointer_l = pointer_l.wrapping_add(self.y) as u16;
        // 5
        // The high byte of the effective address may be invalid
        // at this time, i.e. it may be smaller by $100.
        if overflow {
            pointer_h += 1;
        }

        self.op_addr = (pointer_h << 8) | pointer_l;
        self.op = self.bus.read(self.op_addr);
    }

    fn bcc(&mut self) {
        if self.get_bit(StatusBit::Carry) == false {
            self.pc = self.op_addr;
        }
    }

    fn bcs(&mut self) {
        if self.get_bit(StatusBit::Carry) == true {
            self.pc = self.op_addr;
        }
    }

    fn beq(&mut self) {
        if self.get_bit(StatusBit::Zero) == true {
            self.pc = self.op_addr;
        }
    }

    fn bmi(&mut self) {
        if self.get_bit(StatusBit::Sign) == true {
            self.pc = self.op_addr;
        }
    }

    fn bne(&mut self) {
        if self.get_bit(StatusBit::Zero) == false {
            self.pc = self.op_addr;
        }
    }

    fn bpl(&mut self) {
        if self.get_bit(StatusBit::Sign) == false {
            self.pc = self.op_addr;
        }
    }

    fn bvc(&mut self) {
        if self.get_bit(StatusBit::Overflow) == false {
            self.pc = self.op_addr;
        }
    }

    fn bvs(&mut self) {
        if self.get_bit(StatusBit::Overflow) == true {
            self.pc = self.op_addr;
        }
    }

    fn brk(&mut self) {
        // 2
        self.fetch_u8();
        // 3
        self.stack_push(self.pc as u8);
        self.s -= 1;
        // 4
        self.stack_push((self.pc >> 8) as u8) ;
        self.s -= 1;
        // 5
        self.stack_push(self.p | (1 << 6));// (StatusBit::Break as u8);
        self.s -= 1;
        // 6
        self.pc = self.bus.read(0xfffe) as u16; // pcl
        // 7
        self.pc |= (self.bus.read(0xffff) as u16) << 8; // pch
    }
    
    fn rti(&mut self) {
        self.s += 1;
        self.p = self.stack_pull() | (1 << 5);
        self.s += 1;
        self.pc = self.stack_pull() as u16;
        self.s += 1;
        self.pc |= (self.stack_pull() as u16) << 8;
    }
    
    fn rts(&mut self) {
        self.s += 1;
        // 4
        self.pc = self.stack_pull() as u16; // pcl
        self.s += 1;
        // 5
        self.pc |= (self.stack_pull() as u16) << 8; // pch
        // 6
        self.pc += 1;
    }
    
    fn pha(&mut self) {
        self.stack_push(self.a);
        self.s -= 1;
    }
    
    fn php(&mut self) {
        self.stack_push(self.p | (StatusBit::Break as u8));
        self.s -= 1;
    }

    fn pla(&mut self) {
        self.s += 1;
        self.a = self.stack_pull();
        self.update_bit_sign_and_zero(self.a);
    }
    
    fn plp(&mut self) {
        self.s += 1;
        self.p = self.stack_pull();
        self.p &= !(StatusBit::Break as u8);
        self.p |=  1 << 5;
    }

    fn jsr(&mut self) {
        let abl = self.fetch_u8();
        self.stack_push((self.pc >> 8) as u8);
        self.s -= 1;
        self.stack_push(self.pc as u8);
        self.s -= 1;
        self.pc = (self.fetch_u8() as u16) << 8;
        self.pc |= abl as u16;
    }

    fn jmp(&mut self) {
        self.pc = self.op_addr;
    }

    fn ind(&mut self) {
        // 2
        let l = self.fetch_u8() as u16;
        // 3
        let h = (self.fetch_u8() as u16) << 8;
        // 4
        let low = self.bus.read(h | l);
        // 5
        // The PCH will always be fetched from the same page
        // than PCL, i.e. page boundary crossing is not handled.
        self.op_addr = (self.bus.read(h | ((l + 1) & 0xff)) as u16) << 8; // pch
        self.op_addr |= low as u16;
    }

    fn info(&self, opcode: u8) -> Option<(fn (&mut Cpu), fn (&mut Cpu), &'static str)> {
        Some(match opcode {
            // ADC - Add with Carry
            0x69 => (Cpu::imm, Cpu::adc, "ADC"),
            0x65 => (Cpu::zp, Cpu::adc, "ADC"),
            0x75 => (Cpu::zp_x, Cpu::adc, "ADC"),
            0x6D => (Cpu::abs, Cpu::adc, "ADC"),
            0x7D => (Cpu::abs_x, Cpu::adc, "ADC"),
            0x79 => (Cpu::abs_y, Cpu::adc, "ADC"),
            0x61 => (Cpu::ind_x, Cpu::adc, "ADC"),
            0x71 => (Cpu::ind_y, Cpu::adc, "ADC"),
            // AND - Logical AND
            0x29 => (Cpu::imm, Cpu::and, "AND"),
            0x25 => (Cpu::zp, Cpu::and, "AND"),
            0x35 => (Cpu::zp_x, Cpu::and, "AND"),
            0x2D => (Cpu::abs, Cpu::and, "AND"),
            0x3D => (Cpu::abs_x, Cpu::and, "AND"),
            0x39 => (Cpu::abs_y, Cpu::and, "AND"),
            0x21 => (Cpu::ind_x,Cpu::and, "AND"),
            0x31 => (Cpu::ind_y, Cpu::and, "AND"),
            // ASL - Arithmetic Shift Left
            0x0A => (Cpu::acc, Cpu::asl, "ASL"),
            0x06 => (Cpu::zp, Cpu::asl, "ASL"),
            0x16 => (Cpu::zp_x, Cpu::asl, "ASL"),
            0x0E => (Cpu::abs, Cpu::asl, "ASL"),
            0x1E => (Cpu::abs_x, Cpu::asl, "ASL"),
            // BCC - Branch if Carry Clear
            0x90 => (Cpu::rel, Cpu::bcc, "BCC"),
            // BCS - Branch if Carry Set
            0xB0 => (Cpu::rel, Cpu::bcs, "BCS"),
            // BEQ - Branch if Equal
            0xF0 => (Cpu::rel, Cpu::beq, "BEQ"),
            // BIT - Bit Test
            0x24 => (Cpu::zp, Cpu::bit, "BIT"),
            0x2C => (Cpu::abs, Cpu::bit, "BIT"),
            // BMI - Branch if Minus
            0x30 => (Cpu::rel, Cpu::bmi, "BMI"),
            // BNE - Branch if Not Equal
            0xD0 => (Cpu::rel, Cpu::bne, "BNE"),
            // BPL - Branch if Positive
            0x10 => (Cpu::rel, Cpu::bpl, "BPL"),
            // BRK - Force Interrupt
            0x00 => (Cpu::imp, Cpu::brk, "BRK"),
            // BVC - Branch if Overflow Clear
            0x50 => (Cpu::rel, Cpu::bvc, "BVC"),
            // BVS - Branch if Overflow Set
            0x70 => (Cpu::rel, Cpu::bvs, "BVS"),
            // CLC - Clear Carry Flag
            0x18 => (Cpu::imp, Cpu::clc, "CLC"),
            // CLD - Clear Decimal Mode
            0xD8 => (Cpu::imp, Cpu::cld, "CLD"),
            // CLI - Clear Interrupt Disable
            0x58 => (Cpu::imp, Cpu::cli, "CLI"),
            // CLV - Clear Overflow Flag
            0xB8 => (Cpu::imp, Cpu::clv, "CLV"),
            // CMP - Compare
            0xC9 => (Cpu::imm, Cpu::cmp, "CMP"),
            0xC5 => (Cpu::zp, Cpu::cmp, "CMP"),
            0xD5 => (Cpu::zp_x, Cpu::cmp, "CMP"),
            0xCD => (Cpu::abs, Cpu::cmp, "CMP"),
            0xDD => (Cpu::abs_x, Cpu::cmp, "CMP"),
            0xD9 => (Cpu::abs_y, Cpu::cmp, "CMP"),
            0xC1 => (Cpu::ind_x,Cpu::cmp, "CMP"),
            0xD1 => (Cpu::ind_y, Cpu::cmp, "CMP"),
            // CPX - Compare X Register
            0xE0 => (Cpu::imm, Cpu::cpx, "CPX"),
            0xE4 => (Cpu::zp, Cpu::cpx, "CPX"),
            0xEC => (Cpu::abs, Cpu::cpx, "CPX"),
            // CPY - Compare Y Register
            0xC0 => (Cpu::imm, Cpu::cpy, "CPY"),
            0xC4 => (Cpu::zp, Cpu::cpy, "CPY"),
            0xCC => (Cpu::abs, Cpu::cpy, "CPY"),
            // DCP
            0xC3 => (Cpu::ind_x, Cpu::dcp, "DCP"),
            0xC7 => (Cpu::zp, Cpu::dcp, "DCP"),
            0xCF => (Cpu::abs, Cpu::dcp, "DCP"),
            0xD3 => (Cpu::ind_y, Cpu::dcp, "DCP"),
            0xD7 => (Cpu::zp_x, Cpu::dcp, "DCP"),
            0xDB => (Cpu::abs_y, Cpu::dcp, "DCP"),
            0xDF => (Cpu::abs_x, Cpu::dcp, "DCP"),
            // DEC - Decrement Memory
            0xC6 => (Cpu::zp, Cpu::dec, "DEC"),
            0xD6 => (Cpu::zp_x, Cpu::dec, "DEC"),
            0xCE => (Cpu::abs, Cpu::dec, "DEC"),
            0xDE => (Cpu::abs_x, Cpu::dec, "DEC"),
            // DEX - Decrement X Register
            0xCA => (Cpu::imp, Cpu::dex, "DEX"),
            // DEY - Decrement Y Register
            0x88 => (Cpu::imp, Cpu::dey, "DEY"),
            // EOR - Exclusive OR
            0x49 => (Cpu::imm, Cpu::eor, "EOR"),
            0x45 => (Cpu::zp, Cpu::eor, "EOR"),
            0x55 => (Cpu::zp_x, Cpu::eor, "EOR"),
            0x4D => (Cpu::abs, Cpu::eor, "EOR"),
            0x5D => (Cpu::abs_x, Cpu::eor, "EOR"),
            0x59 => (Cpu::abs_y, Cpu::eor, "EOR"),
            0x41 => (Cpu::ind_x, Cpu::eor, "EOR"),
            0x51 => (Cpu::ind_y, Cpu::eor, "EOR"),
            // IGN
            0x04 | 0x44 | 0x64 => (Cpu::zp, Cpu::ign, "IGN"),
            0x0C => (Cpu::abs, Cpu::ign, "IGN"),
            0x14 | 0x34 | 0x54 | 0x74 | 0xD4 | 0xF4 => (Cpu::zp_x, Cpu::ign, "IGN"),
            0x1C | 0x3C | 0x5C | 0x7C | 0xDC | 0xFC => (Cpu::abs_x, Cpu::ign, "IGN"),
            // INC - Increment Memory
            0xE6 => (Cpu::zp, Cpu::inc, "INC"),
            0xF6 => (Cpu::zp_x, Cpu::inc, "INC"),
            0xEE => (Cpu::abs, Cpu::inc, "INC"),
            0xFE => (Cpu::abs_x, Cpu::inc, "INC"),
            // INX - Increment X Register
            0xE8 => (Cpu::imp, Cpu::inx, "INX"),
            // INY - Increment Y Register
            0xC8 => (Cpu::imp, Cpu::iny, "INY"),
            // ISC
            0xE3 => (Cpu::ind_x, Cpu::isb, "ISB"),
            0xE7 => (Cpu::zp, Cpu::isb, "ISB"),
            0xEF => (Cpu::abs, Cpu::isb, "ISB"),
            0xF3 => (Cpu::ind_y, Cpu::isb, "ISB"),
            0xF7 => (Cpu::zp_x, Cpu::isb, "ISB"),
            0xFB => (Cpu::abs_y, Cpu::isb, "ISB"),
            0xFF => (Cpu::abs_x, Cpu::isb, "ISB"),
            // JMP - Jump
            0x4C => (Cpu::abs, Cpu::jmp, "JMP"),
            0x6C => (Cpu::ind, Cpu::jmp, "JMP"),
            // JSR - Jump to Subroutine
            0x20 => (Cpu::imp, Cpu::jsr, "JSR"),
            // LAX
            0xA3 => (Cpu::ind_x, Cpu::lax, "LAX"),
            0xA7 => (Cpu::zp, Cpu::lax, "LAX"),
            0xAF => (Cpu::abs, Cpu::lax, "LAX"),
            0xB3 => (Cpu::ind_y, Cpu::lax, "LAX"),
            0xB7 => (Cpu::zp_y, Cpu::lax, "LAX"),
            0xBF => (Cpu::abs_y, Cpu::lax, "LAX"),
            // LDA - Load Accumulator
            0xA9 => (Cpu::imm, Cpu::lda, "LDA"),
            0xA5 => (Cpu::zp, Cpu::lda, "LDA"),
            0xB5 => (Cpu::zp_x, Cpu::lda, "LDA"),
            0xAD => (Cpu::abs, Cpu::lda, "LDA"),
            0xBD => (Cpu::abs_x, Cpu::lda, "LDA"),
            0xB9 => (Cpu::abs_y, Cpu::lda, "LDA"),
            0xA1 => (Cpu::ind_x,Cpu::lda, "LDA"),
            0xB1 => (Cpu::ind_y, Cpu::lda, "LDA"),
            // LDX - Load X Register
            0xA2 => (Cpu::imm, Cpu::ldx, "LDX"),
            0xA6 => (Cpu::zp, Cpu::ldx, "LDX"),
            0xB6 => (Cpu::zp_y, Cpu::ldx, "LDX"),
            0xAE => (Cpu::abs, Cpu::ldx, "LDX"),
            0xBE => (Cpu::abs_y, Cpu::ldx, "LDX"),
            // LDY - Load Y Register
            0xA0 => (Cpu::imm, Cpu::ldy, "LDY"),
            0xA4 => (Cpu::zp, Cpu::ldy, "LDY"),
            0xB4 => (Cpu::zp_x, Cpu::ldy, "LDY"),
            0xAC => (Cpu::abs, Cpu::ldy, "LDY"),
            0xBC => (Cpu::abs_x, Cpu::ldy, "LDY"),
            // LSR - Logical Shift Right
            0x4A => (Cpu::acc, Cpu::lsr, "LSR"),
            0x46 => (Cpu::zp, Cpu::lsr, "LSR"),
            0x56 => (Cpu::zp_x, Cpu::lsr, "LSR"),
            0x4E => (Cpu::abs, Cpu::lsr, "LSR"),
            0x5E => (Cpu::abs_x, Cpu::lsr, "LSR"),
            // NOP - No Operation
            0x1A | 0x3A | 0x5A | 0x7A | 0xDA | 0xEA | 0xFA => (Cpu::imp, Cpu::nop, "NOP"),
            // ORA - Logical Inclusive OR
            0x09 => (Cpu::imm, Cpu::ora, "ORA"),
            0x05 => (Cpu::zp, Cpu::ora, "ORA"),
            0x15 => (Cpu::zp_x, Cpu::ora, "ORA"),
            0x0D => (Cpu::abs, Cpu::ora, "ORA"),
            0x1D => (Cpu::abs_x, Cpu::ora, "ORA"),
            0x19 => (Cpu::abs_y, Cpu::ora, "ORA"),
            0x01 => (Cpu::ind_x,Cpu::ora, "ORA"),
            0x11 => (Cpu::ind_y, Cpu::ora, "ORA"),
            // PHA - Push Accumulator
            0x48 => (Cpu::imp, Cpu::pha, "PHA"),
            // PHP - Push Processor Status
            0x08 => (Cpu::imp, Cpu::php, "PHP"),
            // PLA - Pull Accumulator
            0x68 => (Cpu::imp, Cpu::pla, "PLA"),
            // PLP - Pull Processor Status
            0x28 => (Cpu::imp, Cpu::plp, "PLP"),
            // RLA
            0x23 => (Cpu::ind_x, Cpu::rla, "RLA"),
            0x27 => (Cpu::zp, Cpu::rla, "RLA"),
            0x2F => (Cpu::abs, Cpu::rla, "RLA"),
            0x33 => (Cpu::ind_y, Cpu::rla, "RLA"),
            0x37 => (Cpu::zp_x, Cpu::rla, "RLA"),
            0x3B => (Cpu::abs_y, Cpu::rla, "RLA"),
            0x3F => (Cpu::abs_x, Cpu::rla, "RLA"),
            // ROL - Rotate Left
            0x2A => (Cpu::acc, Cpu::rol, "ROL"),
            0x26 => (Cpu::zp, Cpu::rol, "ROL"),
            0x36 => (Cpu::zp_x, Cpu::rol, "ROL"),
            0x2E => (Cpu::abs, Cpu::rol, "ROL"),
            0x3E => (Cpu::abs_x, Cpu::rol, "ROL"),
            // ROR - Rotate Right
            0x6A => (Cpu::acc, Cpu::ror, "ROR"),
            0x66 => (Cpu::zp, Cpu::ror, "ROR"),
            0x76 => (Cpu::zp_x, Cpu::ror, "ROR"),
            0x6E => (Cpu::abs, Cpu::ror, "ROR"),
            0x7E => (Cpu::abs_x, Cpu::ror, "ROR"),
            // RRA
            0x63 => (Cpu::ind_x, Cpu::rra, "RRA"),
            0x67 => (Cpu::zp, Cpu::rra, "RRA"),
            0x6F => (Cpu::abs, Cpu::rra, "RRA"),
            0x73 => (Cpu::ind_y, Cpu::rra, "RRA"),
            0x77 => (Cpu::zp_x, Cpu::rra, "RRA"),
            0x7B => (Cpu::abs_y, Cpu::rra, "RRA"),
            0x7F => (Cpu::abs_x, Cpu::rra, "RRA"),
            // RTI - Return from Interrupt
            0x40 => (Cpu::imp, Cpu::rti, "RTI"),
            // RTS - Return from Subroutine
            0x60 => (Cpu::imp, Cpu::rts, "RTS"),
            // SAX
            0x83 => (Cpu::ind_x, Cpu::sax, "SAX"),
            0x87 => (Cpu::zp, Cpu::sax, "SAX"),
            0x8F => (Cpu::abs, Cpu::sax, "SAX"),
            0x97 => (Cpu::zp_y, Cpu::sax, "SAX"),
            // SBC - Subtract with Carry
            0xE9 | 0xEB => (Cpu::imm, Cpu::sbc, "SBC"),
            0xE5 => (Cpu::zp, Cpu::sbc, "SBC"),
            0xF5 => (Cpu::zp_x, Cpu::sbc, "SBC"),
            0xED => (Cpu::abs, Cpu::sbc, "SBC"),
            0xFD => (Cpu::abs_x, Cpu::sbc, "SBC"),
            0xF9 => (Cpu::abs_y, Cpu::sbc, "SBC"),
            0xE1 => (Cpu::ind_x,Cpu::sbc, "SBC"),
            0xF1 => (Cpu::ind_y, Cpu::sbc, "SBC"),
            // SEC - Set Carry Flag
            0x38 => (Cpu::imp, Cpu::sec, "SEC"),
            // SED - Set Decimal Flag
            0xF8 => (Cpu::imp, Cpu::sed, "SED"),
            // SEI - Set Interrupt Disable
            0x78 => (Cpu::imp, Cpu::sei, "SEI"),
            // SLO
            0x03 => (Cpu::ind_x, Cpu::slo, "SLO"),
            0x07 => (Cpu::zp, Cpu::slo, "SLO"),
            0x0F => (Cpu::abs, Cpu::slo, "SLO"),
            0x13 => (Cpu::ind_y, Cpu::slo, "SLO"),
            0x17 => (Cpu::zp_x, Cpu::slo, "SLO"),
            0x1B => (Cpu::abs_y, Cpu::slo, "SLO"),
            0x1F => (Cpu::abs_x, Cpu::slo, "SLO"),
            // SRE
            0x43 => (Cpu::ind_x, Cpu::sre, "SRE"),
            0x47 => (Cpu::zp, Cpu::sre, "SRE"),
            0x4f => (Cpu::abs, Cpu::sre, "SRE"),
            0x53 => (Cpu::ind_y, Cpu::sre, "SRE"),
            0x57 => (Cpu::zp_x, Cpu::sre, "SRE"),
            0x5B => (Cpu::abs_y, Cpu::sre, "SRE"),
            0x5F => (Cpu::abs_x, Cpu::sre, "SRE"),
            // STA - Store Accumulator
            0x85 => (Cpu::zp, Cpu::sta, "STA"),
            0x95 => (Cpu::zp_x, Cpu::sta, "STA"),
            0x8D => (Cpu::abs, Cpu::sta, "STA"),
            0x9D => (Cpu::abs_x, Cpu::sta, "STA"),
            0x99 => (Cpu::abs_y, Cpu::sta, "STA"),
            0x81 => (Cpu::ind_x, Cpu::sta, "STA"),
            0x91 => (Cpu::ind_y, Cpu::sta, "STA"),
            // STX - Store X Register
            0x86 => (Cpu::zp, Cpu::stx, "STX"),
            0x96 => (Cpu::zp_y, Cpu::stx, "STX"),
            0x8E => (Cpu::abs, Cpu::stx, "STX"),
            // STY - Store Y Register
            0x84 => (Cpu::zp, Cpu::sty, "STY"),
            0x94 => (Cpu::zp_x, Cpu::sty, "STY"),
            0x8C => (Cpu::abs, Cpu::sty, "STY"),
            // SKB
            0x80 | 0x82 | 0x89 | 0xC2 | 0xE2 => (Cpu::imm, Cpu::skb, "SKB"),
            // TAX - Transfer Accumulator to X
            0xAA => (Cpu::imp, Cpu::tax, "TAX"),
            // TAY - Transfer Accumulator to Y
            0xA8 => (Cpu::imp, Cpu::tay, "TAY"),
            // TSX - Transfer Stack Pointer to X
            0xBA => (Cpu::imp, Cpu::tsx, "TSX"),
            // TXA - Transfer X to Accumulator
            0x8A => (Cpu::imp, Cpu::txa, "TXA"),
            // TXS - Transfer X to Stack Pointer
            0x9A => (Cpu::imp, Cpu::txs, "TXS"),
            // TYA - Transfer Y to Accumulator
            0x98 => (Cpu::imp, Cpu::tya, "TYA"),
            _ => return None,
        })
    }

    fn clock(&mut self) {
        
    }
}