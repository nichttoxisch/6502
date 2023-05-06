enum FLAGS6502 {
  C = 1 << 0, // Carry Bit
  Z = 1 << 1, // Zero
  I = 1 << 2, // Disable Interrupts
  D = 1 << 3, // Decimal Mode
  B = 1 << 4, // Break
  U = 1 << 5, // Unused
  V = 1 << 6, // Overflow
  N = 1 << 7, // Negative
}

interface INSTRUCTION {
  name: string;
  operate(): uint8;
  addrmode(): uint8;
  cycles: uint8;
}

class olc6502 {
  private bus: Bus;

  a: uint8 = 0x00; // Accumulator Register
  x: uint8 = 0x00; // X Register
  y: uint8 = 0x00; // Y Register

  stkp: uint8 = 0x00; // Stack Pointer
  pc: uint16 = 0x0000; // Program Counter

  status: uint8 = 0x00; // Status Register

  private lookup: Array<INSTRUCTION> = new Array<INSTRUCTION>();

  constructor(b: Bus) {
    this.bus = b;

    this.lookup = [
      { name: 'BRK', operate: this.BRK, addrmode: this.IMM, cycles: 7 },
      { name: 'ORA', operate: this.ORA, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 3 },
      { name: 'ORA', operate: this.ORA, addrmode: this.ZP0, cycles: 3 },
      { name: 'ASL', operate: this.ASL, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'PHP', operate: this.PHP, addrmode: this.IMP, cycles: 3 },
      { name: 'ORA', operate: this.ORA, addrmode: this.IMM, cycles: 2 },
      { name: 'ASL', operate: this.ASL, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'ORA', operate: this.ORA, addrmode: this.ABS, cycles: 4 },
      { name: 'ASL', operate: this.ASL, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BPL', operate: this.BPL, addrmode: this.REL, cycles: 2 },
      { name: 'ORA', operate: this.ORA, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'ORA', operate: this.ORA, addrmode: this.ZPX, cycles: 4 },
      { name: 'ASL', operate: this.ASL, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'CLC', operate: this.CLC, addrmode: this.IMP, cycles: 2 },
      { name: 'ORA', operate: this.ORA, addrmode: this.ABY, cycles: 4 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'ORA', operate: this.ORA, addrmode: this.ABX, cycles: 4 },
      { name: 'ASL', operate: this.ASL, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: 'JSR', operate: this.JSR, addrmode: this.ABS, cycles: 6 },
      { name: 'AND', operate: this.AND, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: 'BIT', operate: this.BIT, addrmode: this.ZP0, cycles: 3 },
      { name: 'AND', operate: this.AND, addrmode: this.ZP0, cycles: 3 },
      { name: 'ROL', operate: this.ROL, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'PLP', operate: this.PLP, addrmode: this.IMP, cycles: 4 },
      { name: 'AND', operate: this.AND, addrmode: this.IMM, cycles: 2 },
      { name: 'ROL', operate: this.ROL, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'BIT', operate: this.BIT, addrmode: this.ABS, cycles: 4 },
      { name: 'AND', operate: this.AND, addrmode: this.ABS, cycles: 4 },
      { name: 'ROL', operate: this.ROL, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BMI', operate: this.BMI, addrmode: this.REL, cycles: 2 },
      { name: 'AND', operate: this.AND, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'AND', operate: this.AND, addrmode: this.ZPX, cycles: 4 },
      { name: 'ROL', operate: this.ROL, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'SEC', operate: this.SEC, addrmode: this.IMP, cycles: 2 },
      { name: 'AND', operate: this.AND, addrmode: this.ABY, cycles: 4 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'AND', operate: this.AND, addrmode: this.ABX, cycles: 4 },
      { name: 'ROL', operate: this.ROL, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: 'RTI', operate: this.RTI, addrmode: this.IMP, cycles: 6 },
      { name: 'EOR', operate: this.EOR, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 3 },
      { name: 'EOR', operate: this.EOR, addrmode: this.ZP0, cycles: 3 },
      { name: 'LSR', operate: this.LSR, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'PHA', operate: this.PHA, addrmode: this.IMP, cycles: 3 },
      { name: 'EOR', operate: this.EOR, addrmode: this.IMM, cycles: 2 },
      { name: 'LSR', operate: this.LSR, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'JMP', operate: this.JMP, addrmode: this.ABS, cycles: 3 },
      { name: 'EOR', operate: this.EOR, addrmode: this.ABS, cycles: 4 },
      { name: 'LSR', operate: this.LSR, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BVC', operate: this.BVC, addrmode: this.REL, cycles: 2 },
      { name: 'EOR', operate: this.EOR, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'EOR', operate: this.EOR, addrmode: this.ZPX, cycles: 4 },
      { name: 'LSR', operate: this.LSR, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'CLI', operate: this.CLI, addrmode: this.IMP, cycles: 2 },
      { name: 'EOR', operate: this.EOR, addrmode: this.ABY, cycles: 4 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'EOR', operate: this.EOR, addrmode: this.ABX, cycles: 4 },
      { name: 'LSR', operate: this.LSR, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: 'RTS', operate: this.RTS, addrmode: this.IMP, cycles: 6 },
      { name: 'ADC', operate: this.ADC, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 3 },
      { name: 'ADC', operate: this.ADC, addrmode: this.ZP0, cycles: 3 },
      { name: 'ROR', operate: this.ROR, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'PLA', operate: this.PLA, addrmode: this.IMP, cycles: 4 },
      { name: 'ADC', operate: this.ADC, addrmode: this.IMM, cycles: 2 },
      { name: 'ROR', operate: this.ROR, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'JMP', operate: this.JMP, addrmode: this.IND, cycles: 5 },
      { name: 'ADC', operate: this.ADC, addrmode: this.ABS, cycles: 4 },
      { name: 'ROR', operate: this.ROR, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BVS', operate: this.BVS, addrmode: this.REL, cycles: 2 },
      { name: 'ADC', operate: this.ADC, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'ADC', operate: this.ADC, addrmode: this.ZPX, cycles: 4 },
      { name: 'ROR', operate: this.ROR, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'SEI', operate: this.SEI, addrmode: this.IMP, cycles: 2 },
      { name: 'ADC', operate: this.ADC, addrmode: this.ABY, cycles: 4 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'ADC', operate: this.ADC, addrmode: this.ABX, cycles: 4 },
      { name: 'ROR', operate: this.ROR, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: 'STA', operate: this.STA, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'STY', operate: this.STY, addrmode: this.ZP0, cycles: 3 },
      { name: 'STA', operate: this.STA, addrmode: this.ZP0, cycles: 3 },
      { name: 'STX', operate: this.STX, addrmode: this.ZP0, cycles: 3 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 3 },
      { name: 'DEY', operate: this.DEY, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: 'TXA', operate: this.TXA, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'STY', operate: this.STY, addrmode: this.ABS, cycles: 4 },
      { name: 'STA', operate: this.STA, addrmode: this.ABS, cycles: 4 },
      { name: 'STX', operate: this.STX, addrmode: this.ABS, cycles: 4 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'BCC', operate: this.BCC, addrmode: this.REL, cycles: 2 },
      { name: 'STA', operate: this.STA, addrmode: this.IZY, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'STY', operate: this.STY, addrmode: this.ZPX, cycles: 4 },
      { name: 'STA', operate: this.STA, addrmode: this.ZPX, cycles: 4 },
      { name: 'STX', operate: this.STX, addrmode: this.ZPY, cycles: 4 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'TYA', operate: this.TYA, addrmode: this.IMP, cycles: 2 },
      { name: 'STA', operate: this.STA, addrmode: this.ABY, cycles: 5 },
      { name: 'TXS', operate: this.TXS, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 5 },
      { name: 'STA', operate: this.STA, addrmode: this.ABX, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'LDY', operate: this.LDY, addrmode: this.IMM, cycles: 2 },
      { name: 'LDA', operate: this.LDA, addrmode: this.IZX, cycles: 6 },
      { name: 'LDX', operate: this.LDX, addrmode: this.IMM, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'LDY', operate: this.LDY, addrmode: this.ZP0, cycles: 3 },
      { name: 'LDA', operate: this.LDA, addrmode: this.ZP0, cycles: 3 },
      { name: 'LDX', operate: this.LDX, addrmode: this.ZP0, cycles: 3 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 3 },
      { name: 'TAY', operate: this.TAY, addrmode: this.IMP, cycles: 2 },
      { name: 'LDA', operate: this.LDA, addrmode: this.IMM, cycles: 2 },
      { name: 'TAX', operate: this.TAX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'LDY', operate: this.LDY, addrmode: this.ABS, cycles: 4 },
      { name: 'LDA', operate: this.LDA, addrmode: this.ABS, cycles: 4 },
      { name: 'LDX', operate: this.LDX, addrmode: this.ABS, cycles: 4 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'BCS', operate: this.BCS, addrmode: this.REL, cycles: 2 },
      { name: 'LDA', operate: this.LDA, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'LDY', operate: this.LDY, addrmode: this.ZPX, cycles: 4 },
      { name: 'LDA', operate: this.LDA, addrmode: this.ZPX, cycles: 4 },
      { name: 'LDX', operate: this.LDX, addrmode: this.ZPY, cycles: 4 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'CLV', operate: this.CLV, addrmode: this.IMP, cycles: 2 },
      { name: 'LDA', operate: this.LDA, addrmode: this.ABY, cycles: 4 },
      { name: 'TSX', operate: this.TSX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'LDY', operate: this.LDY, addrmode: this.ABX, cycles: 4 },
      { name: 'LDA', operate: this.LDA, addrmode: this.ABX, cycles: 4 },
      { name: 'LDX', operate: this.LDX, addrmode: this.ABY, cycles: 4 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 4 },
      { name: 'CPY', operate: this.CPY, addrmode: this.IMM, cycles: 2 },
      { name: 'CMP', operate: this.CMP, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: 'CPY', operate: this.CPY, addrmode: this.ZP0, cycles: 3 },
      { name: 'CMP', operate: this.CMP, addrmode: this.ZP0, cycles: 3 },
      { name: 'DEC', operate: this.DEC, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'INY', operate: this.INY, addrmode: this.IMP, cycles: 2 },
      { name: 'CMP', operate: this.CMP, addrmode: this.IMM, cycles: 2 },
      { name: 'DEX', operate: this.DEX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: 'CPY', operate: this.CPY, addrmode: this.ABS, cycles: 4 },
      { name: 'CMP', operate: this.CMP, addrmode: this.ABS, cycles: 4 },
      { name: 'DEC', operate: this.DEC, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BNE', operate: this.BNE, addrmode: this.REL, cycles: 2 },
      { name: 'CMP', operate: this.CMP, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'CMP', operate: this.CMP, addrmode: this.ZPX, cycles: 4 },
      { name: 'DEC', operate: this.DEC, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'CLD', operate: this.CLD, addrmode: this.IMP, cycles: 2 },
      { name: 'CMP', operate: this.CMP, addrmode: this.ABY, cycles: 4 },
      { name: 'NOP', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'CMP', operate: this.CMP, addrmode: this.ABX, cycles: 4 },
      { name: 'DEC', operate: this.DEC, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: 'CPX', operate: this.CPX, addrmode: this.IMM, cycles: 2 },
      { name: 'SBC', operate: this.SBC, addrmode: this.IZX, cycles: 6 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: 'CPX', operate: this.CPX, addrmode: this.ZP0, cycles: 3 },
      { name: 'SBC', operate: this.SBC, addrmode: this.ZP0, cycles: 3 },
      { name: 'INC', operate: this.INC, addrmode: this.ZP0, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 5 },
      { name: 'INX', operate: this.INX, addrmode: this.IMP, cycles: 2 },
      { name: 'SBC', operate: this.SBC, addrmode: this.IMM, cycles: 2 },
      { name: 'NOP', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.SBC, addrmode: this.IMP, cycles: 2 },
      { name: 'CPX', operate: this.CPX, addrmode: this.ABS, cycles: 4 },
      { name: 'SBC', operate: this.SBC, addrmode: this.ABS, cycles: 4 },
      { name: 'INC', operate: this.INC, addrmode: this.ABS, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'BEQ', operate: this.BEQ, addrmode: this.REL, cycles: 2 },
      { name: 'SBC', operate: this.SBC, addrmode: this.IZY, cycles: 5 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 8 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'SBC', operate: this.SBC, addrmode: this.ZPX, cycles: 4 },
      { name: 'INC', operate: this.INC, addrmode: this.ZPX, cycles: 6 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 6 },
      { name: 'SED', operate: this.SED, addrmode: this.IMP, cycles: 2 },
      { name: 'SBC', operate: this.SBC, addrmode: this.ABY, cycles: 4 },
      { name: 'NOP', operate: this.NOP, addrmode: this.IMP, cycles: 2 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
      { name: '???', operate: this.NOP, addrmode: this.IMP, cycles: 4 },
      { name: 'SBC', operate: this.SBC, addrmode: this.ABX, cycles: 4 },
      { name: 'INC', operate: this.INC, addrmode: this.ABX, cycles: 7 },
      { name: '???', operate: this.XXX, addrmode: this.IMP, cycles: 7 },
    ];
  }

  write = (addr: uint16, data: uint8) => {
    this.bus.cpuWrite(addr, data);
  };
  read = (addr: uint16): uint8 => {
    return this.bus.cpuRead(addr);
  };

  clock = () => {
    if (this.cycles === 0) {
      this.opcode = this.read(this.pc);
      this.pc++;

      const op = this.lookup[this.opcode];
      this.cycles = op.cycles;
      const add_c1: uint8 = op.addrmode();
      const add_c2: uint8 = op.operate();

      this.cycles += add_c1 & add_c2;
    }

    this.cycles--;
  };

  complete = (): boolean => {
    return this.cycles == 0;
  };

  reset = () => {
    this.a = 0;
    this.x = 0;
    this.y = 0;
    this.stkp = 0xfd;
    this.status = 0x00 | FLAGS6502.U;

    this.addr_abs = 0xfffc;
    const lo = this.read(this.addr_abs + 0);
    const hi = this.read(this.addr_abs + 1);

    this.pc = (hi << 8) | lo;

    this.addr_rel = 0x000;
    this.addr_abs = 0x000;
    this.fetched = 0x00;

    this.cycles = 8;
  };

  addr_abs: uint16 = 0x0000;
  addr_rel: uint16 = 0x0000;
  opcode: uint8 = 0x00;
  cycles: uint8 = 0;

  // * Fetching
  fetched: uint8 = 0x00;
  fetch = (): uint8 => {
    if (!(this.lookup[this.opcode].addrmode === this.IMP))
      this.fetched = this.read(this.addr_abs);
    return this.fetched;
  };

  // * Interups
  irq = () => {
    if (this.getFlag(FLAGS6502.I) === 0) {
      this.write(0x0100 + this.stkp, (this.pc >> 8) & 0x00ff);
      this.stkp--;
      this.write(0x0100 + this.stkp, this.pc & 0x00ff);
      this.stkp--;

      this.setFlag(FLAGS6502.B, false);
      this.setFlag(FLAGS6502.U, true);
      this.setFlag(FLAGS6502.I, true);
      this.write(0x0100 + this.stkp, this.status);
      this.stkp--;

      this.addr_abs = 0xfffe;
      const lo = this.read(this.addr_abs + 0);
      const hi = this.read(this.addr_abs + 1);
      this.pc = (hi << 8) | lo;

      this.cycles = 7;
    }
  };
  nmi = () => {
    this.write(0x0100 + this.stkp, (this.pc >> 8) & 0x00ff);
    this.stkp--;
    this.write(0x0100 + this.stkp, this.pc & 0x00ff);
    this.stkp--;

    this.setFlag(FLAGS6502.B, false);
    this.setFlag(FLAGS6502.U, true);
    this.setFlag(FLAGS6502.I, true);
    this.write(0x0100 + this.stkp, this.status);
    this.stkp--;

    this.addr_abs = 0xfffa;
    const lo = this.read(this.addr_abs + 0);
    const hi = this.read(this.addr_abs + 1);
    this.pc = (hi << 8) | lo;

    this.cycles = 8;
  };

  // * Addressing Modes
  IMP = (): uint8 => {
    this.fetched = this.a;
    return 0;
  };
  IMM = (): uint8 => {
    this.addr_abs = this.pc++;
    return 0;
  };
  ZP0 = (): uint8 => {
    this.addr_abs = this.read(this.pc);
    this.pc++;
    this.addr_abs &= 0x00ff;
    return 0;
  };
  ZPX = (): uint8 => {
    this.addr_abs = this.read(this.pc) + this.x;
    this.pc++;
    this.addr_abs &= 0x00ff;
    return 0;
  };
  ZPY = (): uint8 => {
    this.addr_abs = this.read(this.pc) + this.y;
    this.pc++;
    this.addr_abs &= 0x00ff;
    return 0;
  };
  REL = (): uint8 => {
    this.addr_rel = this.read(this.pc);
    this.pc++;
    if (this.addr_rel & 0x80) this.addr_rel |= 0xff00;
    return 0;
  };
  ABS = (): uint8 => {
    const lo = this.read(this.pc);
    this.pc++;
    const hi = this.read(this.pc);
    this.pc++;

    this.addr_abs = (hi << 8) | lo;

    return 0;
  };
  ABX = (): uint8 => {
    const lo = this.read(this.pc);
    this.pc++;
    const hi = this.read(this.pc);
    this.pc++;

    this.addr_abs = (hi << 8) | lo;
    this.addr_abs += this.x;

    if ((this.addr_abs & 0xff00) != hi << 8) return 1;
    else return 0;
  };
  ABY = (): uint8 => {
    const lo = this.read(this.pc);
    this.pc++;
    const hi = this.read(this.pc);
    this.pc++;

    this.addr_abs = (hi << 8) | lo;
    this.addr_abs += this.y;

    if ((this.addr_abs & 0xff00) != hi << 8) return 1;
    else return 0;
  };
  IND = (): uint8 => {
    const ptr_lo = this.read(this.pc);
    this.pc++;
    const ptr_hi = this.read(this.pc);
    this.pc++;

    const ptr = (ptr_hi << 8) | ptr_lo;

    if (ptr_lo === 0x00ff)
      this.addr_abs = (this.read(ptr & 0x00ff) << 8) | this.read(ptr + 0);
    else this.addr_abs = (this.read(ptr + 1) << 8) | this.read(ptr + 0);

    return 0;
  };
  IZX = (): uint8 => {
    const t = this.read(this.pc);
    this.pc++;

    const lo = this.read((t + this.x + 0) & 0x00ff);
    const hi = this.read((t + this.x + 1) & 0x00ff);

    this.addr_abs = (hi << 8) | lo;

    return 0;
  };
  IZY = (): uint8 => {
    const t = this.read(this.pc);
    this.pc++;

    const lo = this.read(t & 0x00ff);
    const hi = this.read((t + 1) & 0x00ff);

    this.addr_abs = (hi << 8) | lo;
    this.addr_abs += this.y;

    if ((this.addr_abs & 0xff00) !== hi << 8) return 1;
    else return 0;
  };

  // * Opcodes
  ADC = (): uint8 => {
    this.fetch();
    const tmp = this.a + this.fetched + this.getFlag(FLAGS6502.C);
    this.setFlag(FLAGS6502.C, tmp > 255);
    this.setFlag(FLAGS6502.Z, (tmp & 0x00ff) === 0);
    this.setFlag(
      FLAGS6502.V,
      (~(this.a ^ this.fetched) & (this.a ^ tmp) & 0x0080) > 0
    );
    this.setFlag(FLAGS6502.N, (tmp & 0x80) > 0);

    this.a = tmp & 0x00ff;

    return 1;
  };
  AND = (): uint8 => {
    this.fetch();
    this.a = this.a & this.fetched;

    this.setFlag(FLAGS6502.Z, this.a === 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) != 0);

    return 1;
  };
  ASL = (): uint8 => {
    this.fetch();
    const temp = this.fetched << 1;

    this.setFlag(FLAGS6502.C, (temp & 0xff00) > 0);
    this.setFlag(FLAGS6502.Z, temp === 0x00);
    this.setFlag(FLAGS6502.N, (temp & 0x80) != 0);

    if (this.lookup[this.opcode].addrmode == this.IMP) this.a = temp & 0x00ff;
    else this.write(this.addr_abs, temp & 0x00ff);

    return 0;
  };
  BCC = (): uint8 => {
    if (this.getFlag(FLAGS6502.C) === 0) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BCS = (): uint8 => {
    if (this.getFlag(FLAGS6502.C) === 1) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BEQ = (): uint8 => {
    if (this.getFlag(FLAGS6502.Z) === 1) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BIT = (): uint8 => {
    this.fetch();
    const temp = this.a & this.fetched;

    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x00);
    this.setFlag(FLAGS6502.N, (this.fetched & (1 << 7)) > 0);
    this.setFlag(FLAGS6502.V, (this.fetched & (1 << 6)) > 0);

    return 0;
  };
  BMI = (): uint8 => {
    if (this.getFlag(FLAGS6502.N) === 1) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BNE = (): uint8 => {
    if (this.getFlag(FLAGS6502.Z) == 0) {
      this.cycles++;

      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) != (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs & 0xffff;
    }
    return 0;
  };
  BPL = (): uint8 => {
    if (this.getFlag(FLAGS6502.N) === 0) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BRK = (): uint8 => {
    this.pc++;
    this.setFlag(FLAGS6502.I, true);
    this.write(0x0100 + this.stkp, (this.pc >> 8) & 0x00ff);
    this.stkp--;
    this.write(0x0100 + this.stkp, this.pc & 0x00ff);
    this.stkp--;

    this.setFlag(FLAGS6502.B, true);
    this.write(0x0100 + this.stkp, this.status);
    this.stkp--;
    this.setFlag(FLAGS6502.B, true);

    this.pc = this.read(0xfffe) | (this.read(0xffff) << 8);

    return 0;
  };
  BVC = (): uint8 => {
    if (this.getFlag(FLAGS6502.V) === 0) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  BVS = (): uint8 => {
    if (this.getFlag(FLAGS6502.V) === 1) {
      this.cycles++;
      this.addr_abs = this.pc + this.addr_rel;

      if ((this.addr_abs & 0xff00) !== (this.pc & 0xff00)) this.cycles++;

      this.pc = this.addr_abs;
    }
    return 0;
  };
  CLC = (): uint8 => {
    this.setFlag(FLAGS6502.C, false);
    return 0;
  };
  CLD = (): uint8 => {
    this.setFlag(FLAGS6502.D, false);
    return 0;
  };
  CLI = (): uint8 => {
    this.setFlag(FLAGS6502.I, false);
    return 0;
  };
  CLV = (): uint8 => {
    this.setFlag(FLAGS6502.V, false);
    return 0;
  };
  CMP = (): uint8 => {
    this.fetch();
    const temp = this.a - this.fetched;

    this.setFlag(FLAGS6502.C, this.a >= this.fetched);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x0000);
    this.setFlag(FLAGS6502.N, (temp & 0x0080) > 0);
    return 1;
  };
  CPX = (): uint8 => {
    this.fetch();
    const temp = this.x - this.fetched;

    this.setFlag(FLAGS6502.C, this.x >= this.fetched);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x0000);
    this.setFlag(FLAGS6502.N, (temp & 0x0080) > 0);
    return 1;
  };
  CPY = (): uint8 => {
    this.fetch();
    const temp = this.y - this.fetched;

    this.setFlag(FLAGS6502.C, this.y >= this.fetched);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x0000);
    this.setFlag(FLAGS6502.N, (temp & 0x0080) > 0);
    return 1;
  };
  DEC = (): uint8 => {
    this.fetch();
    const temp = this.fetched - 1;
    this.write(this.addr_abs, temp & 0x00ff);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x0000);
    this.setFlag(FLAGS6502.N, (temp & 0x0080) > 0);
    return 0;
  };
  DEX = (): uint8 => {
    this.x--;
    this.setFlag(FLAGS6502.Z, (this.x & 0x00ff) == 0x0000);
    this.setFlag(FLAGS6502.N, (this.x & 0x0080) > 0);
    return 0;
  };
  DEY = (): uint8 => {
    this.y--;
    this.setFlag(FLAGS6502.Z, this.y == 0x00);
    this.setFlag(FLAGS6502.N, (this.y & 0x80) != 0);
    return 0;
  };
  EOR = (): uint8 => {
    this.fetch();
    this.a = this.a ^ this.fetched;
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) != 0);
    return 1;
  };
  INC = (): uint8 => {
    this.fetch();
    const temp = this.fetched + 1;
    this.write(this.addr_abs, temp & 0x00ff);
    this.setFlag(FLAGS6502.Z, temp == 0x00);
    this.setFlag(FLAGS6502.N, (temp & 0x80) != 0);
    return 0;
  };
  INX = (): uint8 => {
    this.x++;
    this.setFlag(FLAGS6502.Z, this.x == 0x00);
    this.setFlag(FLAGS6502.N, (this.x & 0x80) != 0);
    return 0;
  };
  INY = (): uint8 => {
    this.y++;
    this.setFlag(FLAGS6502.Z, this.y == 0x00);
    this.setFlag(FLAGS6502.N, (this.y & 0x80) != 0);
    return 0;
  };
  JMP = (): uint8 => {
    this.pc = this.addr_abs;
    return 0;
  };
  JSR = (): uint8 => {
    this.pc--;
    this.write(0x0100 + this.stkp, (this.pc >> 8) & 0x00ff);
    this.stkp--;
    this.write(0x0100 + this.stkp, this.pc & 0x00ff);
    this.stkp--;

    this.pc = this.addr_abs;

    return 0;
  };
  LDA = (): uint8 => {
    this.fetch();
    this.a = this.fetched;
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) != 0);
    return 1;
  };
  LDX = (): uint8 => {
    this.fetch();
    this.x = this.fetched;
    this.setFlag(FLAGS6502.Z, this.x == 0x00);
    this.setFlag(FLAGS6502.N, (this.x & 0x80) != 0);
    return 1;
  };
  LDY = (): uint8 => {
    this.fetch();
    this.y = this.fetched;
    this.setFlag(FLAGS6502.Z, this.y == 0x00);
    this.setFlag(FLAGS6502.N, (this.y & 0x80) != 0);
    return 0;
  };
  LSR = (): uint8 => {
    this.fetch();

    this.setFlag(FLAGS6502.C, (this.fetched & 0x000f) > 0);
    const temp = this.fetched >> 1;
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x00);
    this.setFlag(FLAGS6502.N, (temp & 0x80) != 0);

    if (this.lookup[this.opcode].addrmode == this.IMP) this.a = temp & 0x00ff;
    else this.write(this.addr_abs, temp & 0x00ff);

    return 0;
  };
  NOP = (): uint8 => {
    switch (this.opcode) {
      case 0x1c:
      case 0x3c:
      case 0x5c:
      case 0x7c:
      case 0xdc:
      case 0xfc:
        return 1;
        break;
    }
    return 0;
  };
  ORA = (): uint8 => {
    this.fetch();
    this.a = this.a | this.fetched;
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) > 0);

    return 1;
  };
  PHA = (): uint8 => {
    this.write(0x0100 + this.stkp, this.a);
    this.stkp--;
    return 0;
  };
  PHP = (): uint8 => {
    this.write(0x100 + this.stkp, this.status | FLAGS6502.B | FLAGS6502.U);
    this.setFlag(FLAGS6502.B, false);
    this.setFlag(FLAGS6502.U, false);
    this.stkp--;
    return 0;
  };
  PLA = (): uint8 => {
    this.stkp++;
    this.a = this.read(0x0100 + this.stkp);
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) > 0);
    return 0;
  };
  PLP = (): uint8 => {
    this.stkp++;
    this.status = this.read(0x100 + this.stkp);
    this.setFlag(FLAGS6502.U, true);
    return 0;
  };
  ROL = (): uint8 => {
    this.fetch();

    const temp = (this.fetched << 1) | this.getFlag(FLAGS6502.C);

    this.setFlag(FLAGS6502.C, (temp & 0xff00) > 0);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x00);
    this.setFlag(FLAGS6502.N, (temp & 0x80) != 0);

    if (this.lookup[this.opcode].addrmode == this.IMP) this.a = temp & 0x00ff;
    else this.write(this.addr_abs, temp & 0x00ff);

    return 0;
  };
  ROR = (): uint8 => {
    this.fetch();

    const temp = (this.getFlag(FLAGS6502.C) << 7) | (this.fetched >> 1);

    this.setFlag(FLAGS6502.C, (this.fetched & 0x01) > 0);
    this.setFlag(FLAGS6502.Z, (temp & 0x00ff) == 0x00);
    this.setFlag(FLAGS6502.N, (temp & 0x80) != 0);

    if (this.lookup[this.opcode].addrmode == this.IMP) this.a = temp & 0x00ff;
    else this.write(this.addr_abs, temp & 0x00ff);

    return 0;
  };
  RTI = (): uint8 => {
    this.stkp++;
    this.status = this.read(0x0100 + this.stkp);
    this.status &= ~FLAGS6502.B;
    this.status &= ~FLAGS6502.U;

    this.stkp++;
    this.pc = this.read(0x0100 + this.stkp);
    this.stkp++;
    this.pc |= this.read(0x0100 + this.stkp) << 8;

    return 0;
  };
  RTS = (): uint8 => {
    this.stkp++;
    this.pc = this.read(0x0100 + this.stkp);
    this.stkp++;
    this.pc |= this.read(0x0100 + this.stkp) << 8;

    this.pc++;
    return 0;
  };
  SBC = (): uint8 => {
    this.fetch();
    const value = this.fetched ^ 0x00ff;
    const tmp = this.a + value + this.getFlag(FLAGS6502.C);

    this.setFlag(FLAGS6502.C, tmp > 255);
    this.setFlag(FLAGS6502.Z, (tmp & 0x00ff) === 0);
    this.setFlag(
      FLAGS6502.V,
      (~(this.a ^ this.fetched) & (this.a ^ tmp) & 0x0080) > 0
    );
    this.setFlag(FLAGS6502.N, (tmp & 0x80) > 0);

    this.a = tmp & 0x00ff;

    return 0;
  };
  SEC = (): uint8 => {
    this.setFlag(FLAGS6502.C, true);
    return 0;
  };
  SED = (): uint8 => {
    this.setFlag(FLAGS6502.D, true);
    return 0;
  };
  SEI = (): uint8 => {
    this.setFlag(FLAGS6502.I, true);
    return 0;
  };
  STA = (): uint8 => {
    this.write(this.addr_abs, this.a);
    return 0;
  };
  STX = (): uint8 => {
    this.write(this.addr_abs, this.x);
    return 0;
  };
  STY = (): uint8 => {
    this.write(this.addr_abs, this.y);
    return 0;
  };
  TAX = (): uint8 => {
    this.x = this.a;
    this.setFlag(FLAGS6502.Z, this.x == 0x00);
    this.setFlag(FLAGS6502.N, (this.x & 0x80) != 0);
    return 0;
  };
  TAY = (): uint8 => {
    this.y = this.a;
    this.setFlag(FLAGS6502.Z, this.y == 0x00);
    this.setFlag(FLAGS6502.N, (this.y & 0x80) != 0);
    return 0;
  };
  TSX = (): uint8 => {
    this.x = this.stkp;
    this.setFlag(FLAGS6502.Z, this.x == 0x00);
    this.setFlag(FLAGS6502.N, (this.x & 0x80) != 0);
    return 0;
  };
  TXA = (): uint8 => {
    this.a = this.a;
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) != 0);
    return 0;
  };
  TXS = (): uint8 => {
    this.stkp = this.x;
    return 0;
  };
  TYA = (): uint8 => {
    this.a = this.y;
    this.setFlag(FLAGS6502.Z, this.a == 0x00);
    this.setFlag(FLAGS6502.N, (this.a & 0x80) != 0);
    return 0;
  };

  XXX = (): uint8 => {
    return 0;
  };

  // * Flags
  getFlag = (f: FLAGS6502): uint8 => {
    return (this.status & f) > 0 ? 1 : 0;
  };
  setFlag = (f: FLAGS6502, v: boolean) => {
    if (v) this.status |= f;
    else this.status &= ~f;
  };

  // * Disassembler
  disassemble = (nStart: number, nStop: number): Map<number, string> => {
    let addr = nStart;
    let value = 0x00;
    let lo = 0x00;
    let hi = 0x00;
    let mapLines = new Map<number, string>();
    let line_addr = 0;

    while (addr <= nStop) {
      line_addr = addr;

      let sInst = '$' + toHex(addr, 4) + ': ';

      let opc = this.read(addr);
      addr++;

      sInst += this.lookup[opc].name + ' ';

      switch (this.lookup[opc].addrmode) {
        case this.IMP: {
          sInst += ' {IMP}';
          break;
        }
        case this.IMM: {
          value = this.bus.cpuRead(addr);
          addr++;
          sInst += '#$' + toHex(value, 2) + ' {IMM}';
          break;
        }
        case this.ZP0: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = 0x00;
          sInst += '$' + toHex(lo, 2) + ' {ZP0}';
          break;
        }
        case this.ZPX: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = 0x00;
          sInst += '$' + toHex(lo, 2) + ', X {ZPX}';
          break;
        }
        case this.ZPY: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = 0x00;
          sInst += '$' + toHex(lo, 2) + ', Y {ZPY}';
          break;
        }
        case this.IZX: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = 0x00;
          sInst += '($' + toHex(lo, 2) + ', X) {IZX}';
          break;
        }
        case this.IZY: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = 0x00;
          sInst += '($' + toHex(lo, 2) + ', Y) {IZY}';
          break;
        }
        case this.ABS: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = this.bus.cpuRead(addr);
          addr++;
          sInst += '$' + toHex((hi << 8) | lo, 4) + ' {ABS}';
          break;
        }
        case this.ABX: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = this.bus.cpuRead(addr);
          addr++;
          sInst += '$' + toHex((hi << 8) | lo, 4) + ', X {ABX}';
          break;
        }
        case this.ABY: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = this.bus.cpuRead(addr);
          addr++;
          sInst += '$' + toHex((hi << 8) | lo, 4) + ', Y {ABY}';
          break;
        }
        case this.IND: {
          lo = this.bus.cpuRead(addr);
          addr++;
          hi = this.bus.cpuRead(addr);
          addr++;
          sInst += '$' + toHex((hi << 8) | lo, 4) + ') {IND}';
          break;
        }
        case this.REL: {
          value = this.bus.cpuRead(addr);
          addr++;
          sInst +=
            '$' + toHex(value, 2) + ' [$' + toHex(addr + value, 4) + '] {REL}';
          break;
        }
      }
      mapLines.set(line_addr, sInst);
    }

    return mapLines;
  };
}
