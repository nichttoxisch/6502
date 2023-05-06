let nes: Bus;
let font_pixel: p5.Font;
let code: string;
let asmMap: Map<number, string>;

function setup() {
  nes = new Bus();

  frameRate(1000);

  createCanvas(680, 480);
  background([0, 0, 255]);

  font_pixel = loadFont('../font/Pixel.ttf');
  textFont(font_pixel);
  textSize(8);

  code =
    '00 06 a2 00 a0 00 8a 99 00 02 48 e8 c8 c0 10 d0 f5 68 99 00 02 c8 c0 20 d0 f7';
  nes.loadProgram(code);

  addEventListener('keydown', (e) => {
    if (e.code == 'Space') {
      nes.clock();
    }

    if (e.code == 'KeyR') {
      nes.cpu.reset();
    }
  });

  document.querySelector('#selectFile').addEventListener('change', (e) => {
    let data: number[] = [];

    const file = (e.target as HTMLInputElement).files[0];

    var reader = new FileReader();
    reader.onload = function (e) {
      let v = e.target.result.toString();
      for (var i = 0; i < v.length; i++) {
        data.push(v.charCodeAt(i));
      }

      console.log(data);

      for (let i in data) nes.cpuRam[i] = data[i];
      nes.setStartAddr(0x0400);
    };

    reader.onerror = function (e) {
      console.log('Error : ' + e.type);
    };

    reader.readAsBinaryString(file);
  });
}

const WHITE = [255, 255, 255];
const BLUE = [0, 0, 255];
const GREEN = [0, 255, 0];
const RED = [255, 0, 0];
const CYAN = [0, 255, 255];

function drawString(x: number, y: number, str: string, color = WHITE) {
  fill(color);
  text(str, x, y);
}

function drawRam(
  x: number,
  y: number,
  nAddr: number,
  nRows: number,
  nColumns: number
) {
  const nRamX = x;
  let nRamY = y;
  for (let row = 0; row < nRows; row++) {
    let sOffset = '$' + toHex(nAddr, 4) + ':';
    for (let col = 0; col < nColumns; col++) {
      sOffset += ' ' + hex(nes.cpuRead(nAddr), 2);
      nAddr += 1;
    }

    drawString(nRamX, nRamY, sOffset);
    nRamY += 10;
  }
}

function drawByteCode(
  x: number,
  y: number,
  nAddr: number,
  nRows: number,
  nColumns: number
) {
  const nRamX = x;
  let nRamY = y;
  for (let row = 0; row < nRows; row++) {
    let sOffset = '$' + toHex(nAddr, 4) + ':';
    for (let col = 0; col < nColumns; col++) {
      sOffset += ' ' + hex(nes.cpuRead(nAddr), 2);
      nAddr += 1;
    }

    drawString(nRamX, nRamY, sOffset);
    noStroke();
    fill(CYAN);
    rect((nes.cpu.pc & 0x000f) * 24 + 58, y + 1, 15, 1);
    nRamY += 10;
  }
}

function drawCpu(x: number, y: number) {
  fill([255, 255, 255]);

  drawString(x, y, 'STATUS:');
  drawString(x + 64, y, 'N', nes.cpu.status & FLAGS6502.N ? GREEN : RED);
  drawString(x + 80, y, 'V', nes.cpu.status & FLAGS6502.V ? GREEN : RED);
  drawString(x + 96, y, '-', nes.cpu.status & FLAGS6502.U ? GREEN : RED);
  drawString(x + 112, y, 'B', nes.cpu.status & FLAGS6502.B ? GREEN : RED);
  drawString(x + 128, y, 'D', nes.cpu.status & FLAGS6502.D ? GREEN : RED);
  drawString(x + 144, y, 'I', nes.cpu.status & FLAGS6502.I ? GREEN : RED);
  drawString(x + 160, y, 'Z', nes.cpu.status & FLAGS6502.Z ? GREEN : RED);
  drawString(x + 178, y, 'C', nes.cpu.status & FLAGS6502.C ? GREEN : RED);
  drawString(x, y + 10, 'PC: $' + toHex(nes.cpu.pc, 4));
  drawString(x, y + 20, 'A: $' + toHex(nes.cpu.a, 2) + '  [' + nes.cpu.a + ']');
  drawString(x, y + 30, 'X: $' + toHex(nes.cpu.x, 2) + '  [' + nes.cpu.x + ']');
  drawString(x, y + 40, 'Y: $' + toHex(nes.cpu.y, 2) + '  [' + nes.cpu.y + ']');
  drawString(x, y + 50, 'Stack P: $' + hex(nes.cpu.stkp, 4));
}

function drawAsm(x: number, y: number, anLines: number) {
  let pc = nes.cpu.pc;
  let posY = 0;
  for (let i = -anLines; i < anLines; i++) {
    if (asmMap.has(pc + i)) {
      drawString(x, posY + y, asmMap.get(pc + i), i == 0 ? CYAN : WHITE);
      posY += 10;
    }
  }
}

function draw() {
  asmMap = nes.cpu.disassemble(nes.cpu.pc - 30, nes.cpu.pc + 30);

  background(BLUE);

  drawCpu(448, 10);
  drawRam(2, 10, 0x0000, 16, 16);
  drawByteCode(2, 182, nes.cpu.pc & 0xfff0, 16, 16);
  drawAsm(448, 82, 27);
}
