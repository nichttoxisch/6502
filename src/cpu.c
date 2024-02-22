#include "inc/cpu.h"

void memory_init(memory_t *memory) {}

uint8_t memory_read_u8(const memory_t *memory, uint16_t addr) {
  return memory->buffer[addr];
}

void memory_write_u8(memory_t *memory, uint16_t addr, uint8_t value) {
  memory->buffer[addr] = value;
}

void cpu_init(cpu_t *cpu, memory_t *memory) {
  cpu->memory = memory;
  memory_init(cpu->memory);

  cpu->pc = cpu_read_u16(cpu, 0xfffc);

  cpu->sp = 0x00fd;
  cpu->flag.i = 1;
  cpu->flag.u = 1;
  cpu->a = cpu->x = cpu->y = 0;
}

uint8_t cpu_read_u8(cpu_t *cpu, uint16_t addr) {
  uint8_t data = memory_read_u8(cpu->memory, addr);
  return data;
}

uint16_t cpu_read_u16(cpu_t *cpu, uint16_t addr) {
  uint16_t lo = cpu_read_u8(cpu, addr);
  uint16_t hi = cpu_read_u8(cpu, addr + 1) << 8;
  return hi | lo;
}

void cpu_write_u8(cpu_t *cpu, uint16_t addr, uint8_t value) {
  memory_write_u8(cpu->memory, addr, value);
}

void cpu_write_u16(cpu_t *cpu, uint16_t addr, uint16_t value) {
  cpu_write_u8(cpu, addr, value & 0xff);
  cpu_write_u8(cpu, addr + 1, (value >> 8) & 0xff);
}

uint8_t cpu_fetch_u8(cpu_t *cpu) {
  uint8_t data = cpu_read_u8(cpu, cpu->pc);
  cpu->pc += 1;
  return data;
}

uint16_t cpu_fetch_u16(cpu_t *cpu) {
  uint16_t data = cpu_read_u16(cpu, cpu->pc);
  cpu->pc += 2;
  return data;
}

void cpu_push_u8(cpu_t *cpu, uint8_t data) {
  cpu_write_u8(cpu, 0x0100 + cpu->sp, data);
  cpu->sp -= 1;
}
void cpu_push_u16(cpu_t *cpu, uint16_t data) {
  cpu_write_u8(cpu, 0x0100 + cpu->sp, (data >> 8) & 0xff);
  cpu->sp -= 1;
  cpu_write_u8(cpu, 0x0100 + cpu->sp, data & 0xff);
  cpu->sp -= 1;
}
uint8_t cpu_pop_u8(cpu_t *cpu) {
  cpu->sp += 1;
  return cpu_read_u8(cpu, 0x0100 + cpu->sp);
}
uint16_t cpu_pop_u16(cpu_t *cpu) {
  cpu->sp += 1;
  uint16_t lo = cpu_read_u16(cpu, 0x0100 + cpu->sp);
  cpu->sp += 1;
  uint16_t hi = cpu_read_u16(cpu, 0x0100 + cpu->sp) << 8;
  return hi | lo;
}

void cpu_step(cpu_t *cpu) {
  uint8_t opc = cpu_fetch_u8(cpu);
  cpu->opc = opc;
  cpu_instr_t instr = cpu_instr_lookup[opc];
  cpu->cycles += instr.cycles;

  uint16_t fetched = instr.fetch_data(cpu);
  instr.operate(cpu, fetched);
}

void cpu_execute(cpu_t *cpu, uint64_t cycles) {
  while (cpu->cycles < cycles || cycles == 0) {
    cpu_step(cpu);
  }
}

void cpu_execute_irq(cpu_t *cpu) {
  if (cpu->flag.i == 0) {
    cpu_push_u16(cpu, cpu->pc);

    cpu->flag.b = 0;
    cpu->flag.u = 1;
    cpu->flag.i = 1;
    cpu_push_u8(cpu, cpu->sr);

    cpu->pc = cpu_read_u16(cpu, 0xfffa);
  }
}

void cpu_execute_nmi(cpu_t *cpu) {
  cpu_push_u16(cpu, cpu->pc);

  cpu->flag.b = 0;
  cpu->flag.u = 1;
  cpu->flag.i = 1;
  cpu_push_u8(cpu, cpu->sr);

  cpu->pc = cpu_read_u16(cpu, 0xfffe);
}

#define TERMINAL_COLOR_RST "\033[0m"
#define TERMINAL_COLOR_GRN "\033[32m"
#define TERMINAL_COLOR_RED "\033[31m"

void cpu_state(cpu_t *cpu) {
  // printf("  P: %02x ", cpu->sr);
  // printf("%sN", (cpu->flag.n) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sV", (cpu->flag.v) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%s-", (cpu->flag.u) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sB", (cpu->flag.b) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sD", (cpu->flag.d) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sI", (cpu->flag.i) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sZ", (cpu->flag.z) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf("%sC", (cpu->flag.c) ? TERMINAL_COLOR_GRN : TERMINAL_COLOR_RED);
  // printf(TERMINAL_COLOR_RST "\r\n");
  // printf("  PC: $%04x SP: $%04x\r\n", cpu->pc, cpu->sp);
  // printf("  A: $%02x (%d) X: $%02x (%d) Y: $%02x (%d)\r\n", cpu->a, cpu->a,
  //        cpu->x, cpu->x, cpu->y, cpu->y);

  // C000  4C F5 C5  JMP $C5F5                       A:00 X:00 Y:00 P:24 SP:FD
  // PPU:  0, 21 CYC:7

  printf("%04X  ", cpu->pc);
  cpu_disassemble(cpu, cpu->pc, cpu->pc, stdout);
  printf("\t\tA:%02X X:%02X Y:%02X P:%02X SP:%02X\r\n", cpu->a, cpu->x, cpu->y,
         cpu->sr, cpu->sp);
}

void cpu_log(cpu_t *cpu, FILE *f) {
  fprintf(f, "%04X  ", cpu->pc);
  cpu_disassemble(cpu, cpu->pc, cpu->pc, f);
  fprintf(f, "\t\tA:%02X X:%02X Y:%02X P:%02X SP:%02X\r\n", cpu->a, cpu->x,
          cpu->y, cpu->sr, cpu->sp);
}

void cpu_disassemble(cpu_t *cpu, uint16_t from, uint16_t to, FILE *f) {
  if (from <= to) {
    uint8_t opc = cpu_read_u8(cpu, from);
    cpu_instr_t instr = cpu_instr_lookup[opc];

    //

    if (instr.fetch_data == &cpu_addr_mode_imp) {
      fprintf(f, "%02X        %s      ", opc, instr.name);
    } else if (instr.fetch_data == &cpu_addr_mode_imm) {
      uint8_t byte = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s #$%02X", opc, byte, instr.name, byte);
    } else if (instr.fetch_data == &cpu_addr_mode_zp0) {
      uint8_t byte = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s $%02X  ", opc, byte, instr.name, byte);
    } else if (instr.fetch_data == &cpu_addr_mode_zpx) {
      uint8_t byte = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s $%02X,X", opc, byte, instr.name, byte);
    } else if (instr.fetch_data == &cpu_addr_mode_zpy) {
      uint8_t byte = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s $%02X,Y", opc, byte, instr.name, byte);
    } else if (instr.fetch_data == &cpu_addr_mode_rel) {
      uint16_t addr = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     ", opc, addr);
      if (addr & 0x80) {
        addr |= 0xff00;
      }
      fprintf(f, "%s $%04X", instr.name, (uint16_t)(addr + from + 2));
    } else if (instr.fetch_data == &cpu_addr_mode_abs) {
      uint16_t addr = cpu_read_u16(cpu, from + 1);
      fprintf(f, "%02X %02X %02X  %s $%04X", opc, addr & 0xff,
              (addr >> 8) & 0xff, instr.name, addr);
    } else if (instr.fetch_data == &cpu_addr_mode_abx) {
      uint16_t addr = cpu_read_u16(cpu, from + 1);
      fprintf(f, "%02X %02X %02X  %s $%04X,X", opc, addr & 0xff,
              (addr >> 8) & 0xff, instr.name, addr);
    } else if (instr.fetch_data == &cpu_addr_mode_aby) {
      uint16_t addr = cpu_read_u16(cpu, from + 1);
      fprintf(f, "%02X %02X %02X  %s $%04X,Y", opc, addr & 0xff,
              (addr >> 8) & 0xff, instr.name, addr);
    } else if (instr.fetch_data == &cpu_addr_mode_ind) {
      uint16_t addr = cpu_read_u16(cpu, from + 1);
      fprintf(f, "%02X %02X %02X  %s $%04X", opc, addr & 0xff,
              (addr >> 8) & 0xff, instr.name, addr);
    } else if (instr.fetch_data == &cpu_addr_mode_izx) {
      uint16_t addr = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s ($%02X,X)", opc, addr, instr.name, addr);
    } else if (instr.fetch_data == &cpu_addr_mode_izy) {
      uint16_t addr = cpu_read_u8(cpu, from + 1);
      fprintf(f, "%02X %02X     %s ($%02X),Y", opc, addr, instr.name, addr);
    }
  }
}

// * CARTRIDGE *

void cartridge_init(cartridge_t *cart, const char *path) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "[ERROR] Could not open '%s'\r\n", path);
    exit(1);
  };

  fread(&cart->header, sizeof(cart->header), 1, f);

  if (cart->header.mapper1 & 0x04) {
    fseek(f, 512, SEEK_CUR);
  }

  cart->mapper_id =
      ((cart->header.mapper2 >> 4) << 4) | (cart->header.mapper1 >> 4);

  uint8_t file_type = 1;

  if (file_type == 0) {
    fprintf(stderr, "[ERROR] File type not implemented\r\n");
  }

  if (file_type == 1) {
    cart->prg_banks = cart->header.prg_rom_chunks;
    cart->prg_memory_size = 16384 * cart->prg_banks;
    cart->prg_memory = malloc(cart->prg_memory_size);
    fread(cart->prg_memory, cart->prg_memory_size, 1, f);

    cart->chr_banks = cart->header.chr_rom_chunks;
    cart->chr_memory_size = 8192 * cart->chr_banks;
    cart->chr_memory = malloc(cart->chr_memory_size);
    fread(cart->chr_memory, cart->chr_memory_size, 1, f);
  }

  if (file_type == 2) {
    fprintf(stderr, "[ERROR] File type not implemented\r\n");
  }

  fclose(f);
}

// * MAPPER *

void cpu_mapper_init(cpu_mapper_t *mapper, uint8_t prg_banks,
                     uint8_t chr_banks) {
  mapper->chr_banks = chr_banks;
  mapper->prg_banks = prg_banks;
  mapper->type = m000;
}

uint8_t cpu_mapper_read_u8(cpu_mapper_t *mapper, uint16_t addr,
                           uint32_t *mapped_addr) {
  (void)addr;
  (void)mapped_addr;

  switch (mapper->type) {
    case (m000): {
      if (addr >= 0x8000 && addr <= 0xffff) {
        *mapped_addr = addr & (mapper->prg_banks > 1 ? 0x7fff : 0x3fff);
        return 1;
      }
    }
    default:
      fprintf(stderr, "[ERROR] Papper type not implemented.\r\n");
      exit(1);
  }
}

uint8_t cpu_mapper_write_u8(cpu_mapper_t *mapper, uint16_t addr,
                            uint32_t *mapped_addr) {
  (void)addr;
  (void)mapped_addr;

  switch (mapper->type) {
    case (m000): {
      return 0;
    }
    default:
      fprintf(stderr, "[ERROR] Papper type not implemented.\r\n");
      exit(1);
  }
}

// * INSTRUCTIONS *

uint16_t cpu_addr_mode_imp(cpu_t *cpu) { return cpu->a; }

uint16_t cpu_addr_mode_imm(cpu_t *cpu) {
  cpu->addr_abs = cpu->pc;
  return cpu_fetch_u8(cpu);
}

uint16_t cpu_addr_mode_zp0(cpu_t *cpu) {
  uint8_t addr = cpu_fetch_u8(cpu);
  cpu->addr_abs = addr;

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_zpx(cpu_t *cpu) {
  uint16_t addr = cpu_fetch_u8(cpu);
  addr += cpu->x;
  addr &= 0xff;
  cpu->addr_abs = addr;

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_zpy(cpu_t *cpu) {
  uint16_t addr = cpu_fetch_u8(cpu);
  addr += cpu->y;
  addr &= 0xff;
  cpu->addr_abs = addr;

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_rel(cpu_t *cpu) {
  uint16_t addr = cpu_fetch_u8(cpu);
  if (addr & 0x80) {
    addr |= 0xff00;
  }
  cpu->addr_abs = addr;

  return addr;
}

uint16_t cpu_addr_mode_abs(cpu_t *cpu) {
  uint16_t addr = cpu_fetch_u16(cpu);
  cpu->addr_abs = addr;

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_abx(cpu_t *cpu) {
  uint16_t addr_abs = cpu_fetch_u16(cpu);
  uint16_t addr = addr_abs + cpu->x;
  cpu->addr_abs = addr;

  // TODO:Implement potential page crossing extra cycles

  // if ((addr_abs & 0xff00) != (addr & 0xff)) {
  //   cpu->cycles += 1;
  // }

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_aby(cpu_t *cpu) {
  uint16_t addr_abs = cpu_fetch_u16(cpu);
  uint16_t addr = addr_abs + cpu->y;
  cpu->addr_abs = addr;

  // if ((addr_abs & 0xff00) != (addr & 0xff00)) {
  //   cpu->cycles += 1;
  // }

  return cpu_read_u8(cpu, addr);
}

uint16_t cpu_addr_mode_ind(cpu_t *cpu) {
  uint16_t ptr = cpu_fetch_u16(cpu);

  uint16_t addr;
  if ((ptr & 0xff) == 0xff) {  // Simulating page boundary hardware bug
    addr = (cpu_read_u8(cpu, ptr & 0xff00) << 8) | cpu_read_u8(cpu, ptr);
  } else {
    addr = cpu_read_u16(cpu, ptr);
  }
  cpu->addr_abs = addr;

  return cpu_read_u8(cpu, addr);
}
uint16_t cpu_addr_mode_izx(cpu_t *cpu) {
  uint16_t t = cpu_fetch_u8(cpu);
  uint16_t addr = (t + cpu->x);
  uint16_t lo = cpu_read_u8(cpu, addr & 0xff);
  uint16_t hi = cpu_read_u8(cpu, (addr + 1) & 0xff) << 8;
  uint16_t addr_abs = hi | lo;
  cpu->addr_abs = addr_abs;
  return cpu_read_u8(cpu, addr_abs);
}

uint16_t cpu_addr_mode_izy(cpu_t *cpu) {
  uint16_t t = cpu_fetch_u8(cpu);
  uint16_t lo = cpu_read_u8(cpu, t & 0xff);
  uint16_t hi = cpu_read_u8(cpu, (t + 1) & 0xff) << 8;
  uint16_t addr_abs = hi | lo;
  uint16_t addr = addr_abs + cpu->y;
  cpu->addr_abs = addr;

  // if ((addr_abs & 0xff00) != (addr & 0xff00)) {
  //   cpu->cycles += 1;
  // }

  return cpu_read_u16(cpu, addr);
}

void cpu_instr_adc(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = (uint16_t)cpu->a + data + ((uint16_t)cpu->flag.c ? 1 : 0);
  cpu->flag.c = tmp > 255;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
  cpu->flag.v =
      ((~((uint16_t)cpu->a ^ data) & ((uint16_t)cpu->a ^ tmp)) & 0x0080) > 0;
  cpu->a = tmp & 0xff;
}

void cpu_instr_and(cpu_t *cpu, uint16_t data) {
  cpu->a = cpu->a & data;
  cpu->flag.z = cpu->a == 0x00;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_asl(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = data << 1;
  cpu->flag.c = (tmp & 0xff00) > 0;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;

  if (cpu_instr_lookup[cpu->opc].fetch_data == &cpu_addr_mode_imp)
    cpu->a = tmp & 0x00FF;
  else
    cpu_write_u8(cpu, cpu->addr_abs, tmp & 0x00FF);
}

void cpu_instr_bcc(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.c == 0) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_bcs(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.c == 1) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_beq(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.z == 1) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_bit(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = cpu->a & data;

  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (data & (1 << 7)) > 0;
  cpu->flag.v = (data & (1 << 6)) > 0;
}

void cpu_instr_bmi(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.n == 1) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_bne(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.z == 0) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_bpl(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.n == 0) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_brk(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->pc += 1;

  cpu->flag.i = 1;
  cpu_push_u16(cpu, cpu->pc);

  cpu->flag.b = 1;
  cpu_push_u8(cpu, cpu->sp);
  cpu->flag.b = 0;

  cpu->pc = cpu_read_u16(cpu, 0xfffe);
}

void cpu_instr_bvc(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.v == 0) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_bvs(cpu_t *cpu, uint16_t data) {
  if (cpu->flag.v == 1) {
    cpu->cycles += 1;
    uint16_t addr = data + cpu->pc;
    if ((addr & 0xff00) != (cpu->pc & 0xff00)) {
      cpu->cycles += 1;
    }

    cpu->pc = addr;
  }
}

void cpu_instr_clc(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.c = 0;
}

void cpu_instr_cld(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.d = 0;
}

void cpu_instr_cli(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.i = 0;
}

void cpu_instr_clv(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.v = 0;
}

void cpu_instr_cmp(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = (uint16_t)cpu->a - data;
  cpu->flag.c = cpu->a >= data;
  cpu->flag.z = (tmp & 0xff) == 0x00;
  cpu->flag.n = (tmp & 0x0080) > 0;
}

void cpu_instr_sax(cpu_t *cpu, uint16_t data) {
  uint8_t tmp = cpu->a & cpu->x;
  cpu_write_u8(cpu, cpu->addr_abs, tmp);
}

void cpu_instr_cpx(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = (uint16_t)cpu->x - data;
  cpu->flag.c = cpu->x >= data;
  cpu->flag.z = (tmp & 0xff) == 0x00;
  cpu->flag.n = (tmp & 0x0080) > 0;
}

void cpu_instr_cpy(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = (uint16_t)cpu->y - data;
  cpu->flag.c = cpu->y >= data;
  cpu->flag.z = (tmp & 0xff) == 0x00;
  cpu->flag.n = (tmp & 0x0080) > 0;
}

void cpu_instr_dec(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = data - 1;
  cpu_write_u8(cpu, cpu->addr_abs, tmp);
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
}

void cpu_instr_dex(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->x -= 1;
  cpu->flag.z = cpu->x == 0;
  cpu->flag.n = (cpu->x & 0x80) > 0;
}

void cpu_instr_dey(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->y -= 1;
  cpu->flag.z = cpu->y == 0;
  cpu->flag.n = (cpu->y & 0x80) > 0;
}

void cpu_instr_dcm(cpu_t *cpu, uint16_t data) {
  uint8_t tmp = data - 1;
  cpu_write_u8(cpu, cpu->addr_abs, tmp);
  cpu_instr_cmp(cpu, tmp);
}

void cpu_instr_eor(cpu_t *cpu, uint16_t data) {
  cpu->a = cpu->a ^ data;
  cpu->flag.z = cpu->a == 0;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_inc(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = data + 1;
  cpu_write_u8(cpu, cpu->addr_abs, tmp);
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
}

void cpu_instr_inx(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->x += 1;
  cpu->flag.z = cpu->x == 0;
  cpu->flag.n = (cpu->x & 0x80) > 0;
}

void cpu_instr_iny(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->y += 1;
  cpu->flag.z = cpu->y == 0;
  cpu->flag.n = (cpu->y & 0x80) > 0;
}

void cpu_instr_isc(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = data + 1;
  cpu_write_u8(cpu, cpu->addr_abs, tmp);
  cpu_instr_sbc(cpu, tmp);
}

void cpu_instr_jmp(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->pc = cpu->addr_abs;
}

void cpu_instr_jsr(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->pc -= 1;
  cpu_push_u16(cpu, cpu->pc);
  cpu->pc = cpu->addr_abs;
}

void cpu_instr_lda(cpu_t *cpu, uint16_t data) {
  cpu->a = data;
  cpu->flag.z = cpu->a == 0;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_ldx(cpu_t *cpu, uint16_t data) {
  cpu->x = data;
  cpu->flag.z = cpu->x == 0;
  cpu->flag.n = (cpu->x & 0x80) > 0;
}

void cpu_instr_ldy(cpu_t *cpu, uint16_t data) {
  cpu->y = data;
  cpu->flag.z = cpu->y == 0;
  cpu->flag.n = (cpu->y & 0x80) > 0;
}

void cpu_instr_lax(cpu_t *cpu, uint16_t data) {
  cpu->a = data;
  cpu->x = data;
  cpu->flag.z = cpu->a == 0;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_lsr(cpu_t *cpu, uint16_t data) {
  cpu->flag.c = data & 0x001;
  uint16_t tmp = data >> 1;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
  if (cpu_instr_lookup[cpu->opc].fetch_data == &cpu_addr_mode_imp) {
    cpu->a = tmp & 0xff;
  } else {
    cpu_write_u8(cpu, cpu->addr_abs, tmp & 0x00ff);
  }
}

void cpu_instr_nop(cpu_t *cpu, uint16_t data) {
  (void)cpu;
  (void)data;

  switch (cpu->opc) {
    case 0x04:
    case 0x44:
    case 0x64: {
      uint8_t addr = cpu_fetch_u8(cpu);
      cpu_write_u8(cpu, addr, 0x00);
    } break;
    case 0x0c: {
      uint8_t addr = cpu_fetch_u8(cpu);
      uint8_t v = cpu_fetch_u8(cpu);
      cpu_write_u8(cpu, addr, v);
    } break;
    case 0x14:
    case 0x34:
    case 0x54:
    case 0x74:
    case 0xD4:
    case 0xF4: {
      uint8_t addr = cpu_fetch_u8(cpu) + cpu->x;
      cpu_write_u8(cpu, addr & 0xff, 0x00);
    } break;
    case 0x80: {
      cpu_fetch_u8(cpu);
    } break;

    case 0x1C:
    case 0x3C:
    case 0x5C:
    case 0x7C:
    case 0xDC:
    case 0xFC: {
      uint8_t addr = cpu_fetch_u16(cpu) + cpu->x;
      cpu_write_u8(cpu, addr, 0xa9);
    } break;

    default:
      break;
  }

  // TODO: implement proper cycling rules
}

void cpu_instr_ora(cpu_t *cpu, uint16_t data) {
  cpu->a = cpu->a | data;
  cpu->flag.z = cpu->a == 0x00;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_pha(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu_push_u8(cpu, cpu->a);
}

void cpu_instr_php(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->flag.b = 1;
  cpu->flag.u = 1;
  cpu_push_u8(cpu, cpu->sr);

  cpu->flag.b = 0;
}

void cpu_instr_pla(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->a = cpu_pop_u8(cpu);
  cpu->flag.z = cpu->a == 0x00;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_plp(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->sr = cpu_pop_u8(cpu);
  cpu->flag.b = 0;
  cpu->flag.u = 1;
}

void cpu_instr_rol(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = (uint16_t)(data << 1) | cpu->flag.c;

  cpu->flag.c = (tmp & 0xff00) > 0;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
  if (cpu_instr_lookup[cpu->opc].fetch_data == cpu_addr_mode_imp) {
    cpu->a = tmp & 0x00FF;
  } else {
    cpu_write_u8(cpu, cpu->addr_abs, tmp & 0x00FF);
  }
}

void cpu_instr_ror(cpu_t *cpu, uint16_t data) {
  uint16_t tmp = ((uint16_t)cpu->flag.c << 7) | (uint16_t)(data >> 1);

  cpu->flag.c = data & 0x01;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
  if (cpu_instr_lookup[cpu->opc].fetch_data == cpu_addr_mode_imp) {
    cpu->a = tmp & 0x00FF;
  } else {
    cpu_write_u8(cpu, cpu->addr_abs, tmp & 0x00FF);
  }
}

void cpu_instr_rti(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->sr = cpu_pop_u8(cpu);
  cpu->flag.b = 0;
  cpu->flag.u = 1;

  cpu->pc = cpu_pop_u16(cpu);
}

void cpu_instr_rts(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->pc = cpu_pop_u16(cpu);
  cpu->pc++;
}

void cpu_instr_sbc(cpu_t *cpu, uint16_t data) {
  uint16_t value = ((uint16_t)data) ^ 0x00ff;
  uint16_t tmp = (uint16_t)cpu->a + value + (uint16_t)cpu->flag.c;
  cpu->flag.c = tmp > 255;
  cpu->flag.z = (tmp & 0xff) == 0;
  cpu->flag.n = (tmp & 0x80) > 0;
  cpu->flag.v = ((tmp ^ (uint16_t)cpu->a) & (tmp ^ value) & 0x0080) > 0;
  cpu->a = tmp & 0xff;
}

void cpu_instr_sec(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.c = 1;
}

void cpu_instr_sed(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.d = 1;
}

void cpu_instr_sei(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->flag.i = 1;
}

void cpu_instr_sta(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu_write_u8(cpu, cpu->addr_abs, cpu->a);
}

void cpu_instr_stx(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu_write_u8(cpu, cpu->addr_abs, cpu->x);
}

void cpu_instr_sty(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu_write_u8(cpu, cpu->addr_abs, cpu->y);
}

void cpu_instr_tax(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->x = cpu->a;
  cpu->flag.z = cpu->x == 0;
  cpu->flag.n = (cpu->x & 0x80) > 0;
}

void cpu_instr_tay(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->y = cpu->a;
  cpu->flag.z = cpu->y == 0;
  cpu->flag.n = (cpu->y & 0x80) > 0;
}

void cpu_instr_tsx(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->x = cpu->sp;
  cpu->flag.z = cpu->x == 0;
  cpu->flag.n = (cpu->x & 0x80) > 0;
}

void cpu_instr_txa(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->a = cpu->x;
  cpu->flag.z = cpu->a == 0;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_txs(cpu_t *cpu, uint16_t data) {
  (void)data;

  cpu->sp = cpu->x;
}

void cpu_instr_tya(cpu_t *cpu, uint16_t data) {
  (void)data;
  cpu->a = cpu->y;
  cpu->flag.z = cpu->a == 0;
  cpu->flag.n = (cpu->a & 0x80) > 0;
}

void cpu_instr_xxx(cpu_t *cpu, uint16_t data) {
  (void)cpu;
  (void)data;
  switch (cpu->opc) {
    default:
      printf("[ERROR]: Undefined instruction call\r\n");
      exit(1);
      break;
  }
}

const cpu_instr_t cpu_instr_lookup[0x100] = {
    [0x00] = {" brk", cpu_instr_brk, cpu_addr_mode_imm, 7},
    [0x01] = {" ora", cpu_instr_ora, cpu_addr_mode_izx, 6},
    [0x02] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x03] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x04] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 3},
    [0x05] = {" ora", cpu_instr_ora, cpu_addr_mode_zp0, 3},
    [0x06] = {" asl", cpu_instr_asl, cpu_addr_mode_zp0, 5},
    [0x07] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x08] = {" php", cpu_instr_php, cpu_addr_mode_imp, 3},
    [0x09] = {" ora", cpu_instr_ora, cpu_addr_mode_imm, 2},
    [0x0a] = {" asl", cpu_instr_asl, cpu_addr_mode_imp, 2},
    [0x0b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x0c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x0d] = {" ora", cpu_instr_ora, cpu_addr_mode_abs, 4},
    [0x0e] = {" asl", cpu_instr_asl, cpu_addr_mode_abs, 6},
    [0x0f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x10] = {" bpl", cpu_instr_bpl, cpu_addr_mode_rel, 2},
    [0x11] = {" ora", cpu_instr_ora, cpu_addr_mode_izy, 5},
    [0x12] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x13] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x14] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x15] = {" ora", cpu_instr_ora, cpu_addr_mode_zpx, 4},
    [0x16] = {" asl", cpu_instr_asl, cpu_addr_mode_zpx, 6},
    [0x17] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x18] = {" clc", cpu_instr_clc, cpu_addr_mode_imp, 2},
    [0x19] = {" ora", cpu_instr_ora, cpu_addr_mode_aby, 4},
    [0x1a] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x1b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x1c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x1d] = {" ora", cpu_instr_ora, cpu_addr_mode_abx, 4},
    [0x1e] = {" asl", cpu_instr_asl, cpu_addr_mode_abx, 7},
    [0x1f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x20] = {" jsr", cpu_instr_jsr, cpu_addr_mode_abs, 6},
    [0x21] = {" and", cpu_instr_and, cpu_addr_mode_izx, 6},
    [0x22] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x23] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x24] = {" bit", cpu_instr_bit, cpu_addr_mode_zp0, 3},
    [0x25] = {" and", cpu_instr_and, cpu_addr_mode_zp0, 3},
    [0x26] = {" rol", cpu_instr_rol, cpu_addr_mode_zp0, 5},
    [0x27] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x28] = {" plp", cpu_instr_plp, cpu_addr_mode_imp, 4},
    [0x29] = {" and", cpu_instr_and, cpu_addr_mode_imm, 2},
    [0x2a] = {" rol", cpu_instr_rol, cpu_addr_mode_imp, 2},
    [0x2b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x2c] = {" bit", cpu_instr_bit, cpu_addr_mode_abs, 4},
    [0x2d] = {" and", cpu_instr_and, cpu_addr_mode_abs, 4},
    [0x2e] = {" rol", cpu_instr_rol, cpu_addr_mode_abs, 6},
    [0x2f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x30] = {" bmi", cpu_instr_bmi, cpu_addr_mode_rel, 2},
    [0x31] = {" and", cpu_instr_and, cpu_addr_mode_izy, 5},
    [0x32] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x33] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x34] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x35] = {" and", cpu_instr_and, cpu_addr_mode_zpx, 4},
    [0x36] = {" rol", cpu_instr_rol, cpu_addr_mode_zpx, 6},
    [0x37] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x38] = {" sec", cpu_instr_sec, cpu_addr_mode_imp, 2},
    [0x39] = {" and", cpu_instr_and, cpu_addr_mode_aby, 4},
    [0x3a] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x3b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x3c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x3d] = {" and", cpu_instr_and, cpu_addr_mode_abx, 4},
    [0x3e] = {" rol", cpu_instr_rol, cpu_addr_mode_abx, 7},
    [0x3f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x40] = {" rti", cpu_instr_rti, cpu_addr_mode_imp, 6},
    [0x41] = {" eor", cpu_instr_eor, cpu_addr_mode_izx, 6},
    [0x42] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x43] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x44] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 3},
    [0x45] = {" eor", cpu_instr_eor, cpu_addr_mode_zp0, 3},
    [0x46] = {" lsr", cpu_instr_lsr, cpu_addr_mode_zp0, 5},
    [0x47] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x48] = {" pha", cpu_instr_pha, cpu_addr_mode_imp, 3},
    [0x49] = {" eor", cpu_instr_eor, cpu_addr_mode_imm, 2},
    [0x4a] = {" lsr", cpu_instr_lsr, cpu_addr_mode_imp, 2},
    [0x4b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x4c] = {" jmp", cpu_instr_jmp, cpu_addr_mode_abs, 3},
    [0x4d] = {" eor", cpu_instr_eor, cpu_addr_mode_abs, 4},
    [0x4e] = {" lsr", cpu_instr_lsr, cpu_addr_mode_abs, 6},
    [0x4f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x50] = {" bvc", cpu_instr_bvc, cpu_addr_mode_rel, 2},
    [0x51] = {" eor", cpu_instr_eor, cpu_addr_mode_izy, 5},
    [0x52] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x53] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x54] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x55] = {" eor", cpu_instr_eor, cpu_addr_mode_zpx, 4},
    [0x56] = {" lsr", cpu_instr_lsr, cpu_addr_mode_zpx, 6},
    [0x57] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x58] = {" cli", cpu_instr_cli, cpu_addr_mode_imp, 2},
    [0x59] = {" eor", cpu_instr_eor, cpu_addr_mode_aby, 4},
    [0x5a] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x5b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x5c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x5d] = {" eor", cpu_instr_eor, cpu_addr_mode_abx, 4},
    [0x5e] = {" lsr", cpu_instr_lsr, cpu_addr_mode_abx, 7},
    [0x5f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x60] = {" rts", cpu_instr_rts, cpu_addr_mode_imp, 6},
    [0x61] = {" adc", cpu_instr_adc, cpu_addr_mode_izx, 6},
    [0x62] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x63] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x64] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 3},
    [0x65] = {" adc", cpu_instr_adc, cpu_addr_mode_zp0, 3},
    [0x66] = {" ror", cpu_instr_ror, cpu_addr_mode_zp0, 5},
    [0x67] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x68] = {" pla", cpu_instr_pla, cpu_addr_mode_imp, 4},
    [0x69] = {" adc", cpu_instr_adc, cpu_addr_mode_imm, 2},
    [0x6a] = {" ror", cpu_instr_ror, cpu_addr_mode_imp, 2},
    [0x6b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x6c] = {" jmp", cpu_instr_jmp, cpu_addr_mode_ind, 5},
    [0x6d] = {" adc", cpu_instr_adc, cpu_addr_mode_abs, 4},
    [0x6e] = {" ror", cpu_instr_ror, cpu_addr_mode_abs, 6},
    [0x6f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x70] = {" bvs", cpu_instr_bvs, cpu_addr_mode_rel, 2},
    [0x71] = {" adc", cpu_instr_adc, cpu_addr_mode_izy, 5},
    [0x72] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x73] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0x74] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x75] = {" adc", cpu_instr_adc, cpu_addr_mode_zpx, 4},
    [0x76] = {" ror", cpu_instr_ror, cpu_addr_mode_zpx, 6},
    [0x77] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x78] = {" sei", cpu_instr_sei, cpu_addr_mode_imp, 2},
    [0x79] = {" adc", cpu_instr_adc, cpu_addr_mode_aby, 4},
    [0x7a] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x7b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x7c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0x7d] = {" adc", cpu_instr_adc, cpu_addr_mode_abx, 4},
    [0x7e] = {" ror", cpu_instr_ror, cpu_addr_mode_abx, 7},
    [0x7f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0x80] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x81] = {" sta", cpu_instr_sta, cpu_addr_mode_izx, 6},
    [0x82] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x83] = {"*sax", cpu_instr_sax, cpu_addr_mode_izx, 6},
    [0x84] = {" sty", cpu_instr_sty, cpu_addr_mode_zp0, 3},
    [0x85] = {" sta", cpu_instr_sta, cpu_addr_mode_zp0, 3},
    [0x86] = {" stx", cpu_instr_stx, cpu_addr_mode_zp0, 3},
    [0x87] = {"*sax", cpu_instr_sax, cpu_addr_mode_zp0, 3},
    [0x88] = {" dey", cpu_instr_dey, cpu_addr_mode_imp, 2},
    [0x89] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0x8a] = {" txa", cpu_instr_txa, cpu_addr_mode_imp, 2},
    [0x8b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x8c] = {" sty", cpu_instr_sty, cpu_addr_mode_abs, 4},
    [0x8d] = {" sta", cpu_instr_sta, cpu_addr_mode_abs, 4},
    [0x8e] = {" stx", cpu_instr_stx, cpu_addr_mode_abs, 4},
    [0x8f] = {"*sax", cpu_instr_sax, cpu_addr_mode_abs, 4},
    [0x90] = {" bcc", cpu_instr_bcc, cpu_addr_mode_rel, 2},
    [0x91] = {" sta", cpu_instr_sta, cpu_addr_mode_izy, 6},
    [0x92] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0x93] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0x94] = {" sty", cpu_instr_sty, cpu_addr_mode_zpx, 4},
    [0x95] = {" sta", cpu_instr_sta, cpu_addr_mode_zpx, 4},
    [0x96] = {" stx", cpu_instr_stx, cpu_addr_mode_zpy, 4},
    [0x97] = {"*sax", cpu_instr_sax, cpu_addr_mode_zpy, 4},
    [0x98] = {" tya", cpu_instr_tya, cpu_addr_mode_imp, 2},
    [0x99] = {" sta", cpu_instr_sta, cpu_addr_mode_aby, 5},
    [0x9a] = {" txs", cpu_instr_txs, cpu_addr_mode_imp, 2},
    [0x9b] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x9c] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 5},
    [0x9d] = {" sta", cpu_instr_sta, cpu_addr_mode_abx, 5},
    [0x9e] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0x9f] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0xa0] = {" ldy", cpu_instr_ldy, cpu_addr_mode_imm, 2},
    [0xa1] = {" lda", cpu_instr_lda, cpu_addr_mode_izx, 6},
    [0xa2] = {" ldx", cpu_instr_ldx, cpu_addr_mode_imm, 2},
    [0xa3] = {"*lax", cpu_instr_lax, cpu_addr_mode_izx, 6},
    [0xa4] = {" ldy", cpu_instr_ldy, cpu_addr_mode_zp0, 3},
    [0xa5] = {" lda", cpu_instr_lda, cpu_addr_mode_zp0, 3},
    [0xa6] = {" ldx", cpu_instr_ldx, cpu_addr_mode_zp0, 3},
    [0xa7] = {"*lax", cpu_instr_lax, cpu_addr_mode_zp0, 3},
    [0xa8] = {" tay", cpu_instr_tay, cpu_addr_mode_imp, 2},
    [0xa9] = {" lda", cpu_instr_lda, cpu_addr_mode_imm, 2},
    [0xaa] = {" tax", cpu_instr_tax, cpu_addr_mode_imp, 2},
    [0xab] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0xac] = {" ldy", cpu_instr_ldy, cpu_addr_mode_abs, 4},
    [0xad] = {" lda", cpu_instr_lda, cpu_addr_mode_abs, 4},
    [0xae] = {" ldx", cpu_instr_ldx, cpu_addr_mode_abs, 4},
    [0xaf] = {"*lax", cpu_instr_lax, cpu_addr_mode_abs, 4},
    [0xb0] = {" bcs", cpu_instr_bcs, cpu_addr_mode_rel, 2},
    [0xb1] = {" lda", cpu_instr_lda, cpu_addr_mode_izy, 5},
    [0xb2] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0xb3] = {"*lax", cpu_instr_lax, cpu_addr_mode_izy, 5},
    [0xb4] = {" ldy", cpu_instr_ldy, cpu_addr_mode_zpx, 4},
    [0xb5] = {" lda", cpu_instr_lda, cpu_addr_mode_zpx, 4},
    [0xb6] = {" ldx", cpu_instr_ldx, cpu_addr_mode_zpy, 4},
    [0xb7] = {"*lax", cpu_instr_lax, cpu_addr_mode_zpy, 4},
    [0xb8] = {" clv", cpu_instr_clv, cpu_addr_mode_imp, 2},
    [0xb9] = {" lda", cpu_instr_lda, cpu_addr_mode_aby, 4},
    [0xba] = {" tsx", cpu_instr_tsx, cpu_addr_mode_imp, 2},
    [0xbb] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 4},
    [0xbc] = {" ldy", cpu_instr_ldy, cpu_addr_mode_abx, 4},
    [0xbd] = {" lda", cpu_instr_lda, cpu_addr_mode_abx, 4},
    [0xbe] = {" ldx", cpu_instr_ldx, cpu_addr_mode_aby, 4},
    [0xbf] = {"*lax", cpu_instr_lax, cpu_addr_mode_aby, 4},
    [0xc0] = {" cpy", cpu_instr_cpy, cpu_addr_mode_imm, 2},
    [0xc1] = {" cmp", cpu_instr_cmp, cpu_addr_mode_izx, 6},
    [0xc2] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0xc3] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_izx, 8},
    [0xc4] = {" cpy", cpu_instr_cpy, cpu_addr_mode_zp0, 3},
    [0xc5] = {" cmp", cpu_instr_cmp, cpu_addr_mode_zp0, 3},
    [0xc6] = {" dec", cpu_instr_dec, cpu_addr_mode_zp0, 5},
    [0xc7] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_zp0, 5},
    [0xc8] = {" iny", cpu_instr_iny, cpu_addr_mode_imp, 2},
    [0xc9] = {" cmp", cpu_instr_cmp, cpu_addr_mode_imm, 2},
    [0xca] = {" dex", cpu_instr_dex, cpu_addr_mode_imp, 2},
    [0xcb] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0xcc] = {" cpy", cpu_instr_cpy, cpu_addr_mode_abs, 4},
    [0xcd] = {" cmp", cpu_instr_cmp, cpu_addr_mode_abs, 4},
    [0xce] = {" dec", cpu_instr_dec, cpu_addr_mode_abs, 6},
    [0xcf] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_abs, 6},
    [0xd0] = {" bne", cpu_instr_bne, cpu_addr_mode_rel, 2},
    [0xd1] = {" cmp", cpu_instr_cmp, cpu_addr_mode_izy, 5},
    [0xd2] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0xd3] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_izy, 8},
    [0xd4] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0xd5] = {" cmp", cpu_instr_cmp, cpu_addr_mode_zpx, 4},
    [0xd6] = {" dec", cpu_instr_dec, cpu_addr_mode_zpx, 6},
    [0xd7] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_zpx, 6},
    [0xd8] = {" cld", cpu_instr_cld, cpu_addr_mode_imp, 2},
    [0xd9] = {" cmp", cpu_instr_cmp, cpu_addr_mode_aby, 4},
    [0xda] = {" nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0xdb] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_aby, 7},
    [0xdc] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0xdd] = {" cmp", cpu_instr_cmp, cpu_addr_mode_abx, 4},
    [0xde] = {" dec", cpu_instr_dec, cpu_addr_mode_abx, 7},
    [0xdf] = {"*dcm", cpu_instr_dcm, cpu_addr_mode_abx, 7},
    [0xe0] = {" cpx", cpu_instr_cpx, cpu_addr_mode_imm, 2},
    [0xe1] = {" sbc", cpu_instr_sbc, cpu_addr_mode_izx, 6},
    [0xe2] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0xe3] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0xe4] = {" cpx", cpu_instr_cpx, cpu_addr_mode_zp0, 3},
    [0xe5] = {" sbc", cpu_instr_sbc, cpu_addr_mode_zp0, 3},
    [0xe6] = {" inc", cpu_instr_inc, cpu_addr_mode_zp0, 5},
    [0xe7] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 5},
    [0xe8] = {" inx", cpu_instr_inx, cpu_addr_mode_imp, 2},
    [0xe9] = {" sbc", cpu_instr_sbc, cpu_addr_mode_imm, 2},
    [0xea] = {" nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0xeb] = {"*sbc", cpu_instr_sbc, cpu_addr_mode_imm, 2},
    [0xec] = {" cpx", cpu_instr_cpx, cpu_addr_mode_abs, 4},
    [0xed] = {" sbc", cpu_instr_sbc, cpu_addr_mode_abs, 4},
    [0xee] = {" inc", cpu_instr_inc, cpu_addr_mode_abs, 6},
    [0xef] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0xf0] = {" beq", cpu_instr_beq, cpu_addr_mode_rel, 2},
    [0xf1] = {" sbc", cpu_instr_sbc, cpu_addr_mode_izy, 5},
    [0xf2] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 2},
    [0xf3] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 8},
    [0xf4] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0xf5] = {" sbc", cpu_instr_sbc, cpu_addr_mode_zpx, 4},
    [0xf6] = {" inc", cpu_instr_inc, cpu_addr_mode_zpx, 6},
    [0xf7] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 6},
    [0xf8] = {" sed", cpu_instr_sed, cpu_addr_mode_imp, 2},
    [0xf9] = {" sbc", cpu_instr_sbc, cpu_addr_mode_aby, 4},
    [0xfa] = {" nop", cpu_instr_nop, cpu_addr_mode_imp, 2},
    [0xfb] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
    [0xfc] = {"*nop", cpu_instr_nop, cpu_addr_mode_imp, 4},
    [0xfd] = {" sbc", cpu_instr_sbc, cpu_addr_mode_abx, 4},
    [0xfe] = {" inc", cpu_instr_inc, cpu_addr_mode_abx, 7},
    [0xff] = {" ???", cpu_instr_xxx, cpu_addr_mode_imp, 7},
};
