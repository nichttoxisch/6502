class Bus {
  cpu: olc6502;
  cpuRam: Array<uint8>;
  cpuProgramStart: number = 0;
  cpuProgramEnd: number = 0;

  private nSystemClockCounter = 0;

  constructor() {
    this.cpu = new olc6502(this);
    this.cpuRam = new Array<uint8>(64 * 1024).fill(0);
  }

  reset() {
    this.cpu.reset();
    this.nSystemClockCounter = 0;
  }

  setStartAddr(addr: number) {
    nes.cpuWrite(0xfffc, addr & 0x00ff);
    nes.cpuWrite(0xfffd, (addr & 0xff00) >> 8);
    this.reset();
  }

  loadProgram(program: Array<number> | string) {
    if (typeof program == 'string') {
      let byteCode: Array<number> = [];
      for (let byte of program.split(' ')) {
        let val = parseInt(byte, 16);
        byteCode.push(val);
      }

      let lo = byteCode[0];
      let hi = byteCode[1] << 8;
      let loadAddr = lo | hi;

      console.log(byteCode);

      for (let i = 2; i < byteCode.length; i++) {
        nes.cpuWrite(loadAddr + i - 2, byteCode[i]);
      }

      nes.cpuWrite(0xfffc, byteCode[0]);
      nes.cpuWrite(0xfffd, byteCode[1]);

      this.cpuProgramStart = loadAddr;
      this.cpuProgramEnd = loadAddr + byteCode.length;
    }

    this.reset();
  }

  clock() {
    if (
      this.cpu.pc < this.cpuProgramEnd - 2 &&
      this.cpu.pc >= this.cpuProgramStart
    )
      do {
        nes.cpu.clock();
      } while (!nes.cpu.complete());
    else console.log('End');
  }

  cpuWrite(addr: uint16, data: uint8) {
    if (addr >= 0x0000 && addr <= 0xffff) {
      this.cpuRam[addr] = data;
    }
  }

  cpuRead(addr: uint16): uint8 {
    let data = 0x00;

    if (addr >= 0x0000 && addr <= 0xffff) {
      data = this.cpuRam[addr & 0xffff];
    }

    return data;
  }
}
