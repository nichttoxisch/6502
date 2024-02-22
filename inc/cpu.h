#ifndef CPU_H_
#define CPU_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#define MEMORY_BUFFER_CAPACITY (0xffff)
typedef struct memory {
  uint8_t buffer[MEMORY_BUFFER_CAPACITY];
} memory_t;

typedef struct memory memory_t;
void memory_init(memory_t *memory);
uint8_t memory_read_u8(const memory_t *memory, uint16_t addr);
void memory_write_u8(memory_t *memory, uint16_t addr, uint8_t value);

typedef struct cpu {
  uint16_t pc;
  uint16_t sp;

  uint8_t a, x, y;

  union {
    struct {
      uint8_t c : 1;  // Carry
      uint8_t z : 1;  // Zero
      uint8_t i : 1;  // IRQ Diable
      uint8_t d : 1;  // Decimal Mode
      uint8_t b : 1;  // Break Command
      uint8_t u : 1;  // Unused
      uint8_t v : 1;  // Overflow
      uint8_t n : 1;  // Negative
    } flag;
    uint8_t sr;
  };

  memory_t *memory;
  uint64_t cycles;
  uint8_t opc;
  uint16_t addr_abs;
} cpu_t;

void cpu_init(cpu_t *cpu, memory_t *memory);
uint8_t cpu_read_u8(cpu_t *cpu, uint16_t addr);
void cpu_write_u8(cpu_t *cpu, uint16_t addr, uint8_t value);

uint16_t cpu_read_u16(cpu_t *cpu, uint16_t addr);
void cpu_write_u16(cpu_t *cpu, uint16_t addr, uint16_t value);

uint8_t cpu_fetch_u8(cpu_t *cpu);
uint16_t cpu_fetch_u16(cpu_t *cpu);

void cpu_push_u8(cpu_t *cpu, uint8_t data);
void cpu_push_u16(cpu_t *cpu, uint16_t data);
uint8_t cpu_pop_u8(cpu_t *cpu);
uint16_t cpu_pop_u16(cpu_t *cpu);

void cpu_step(cpu_t *cpu);
void cpu_execute(cpu_t *cpu, uint64_t cycles);
void cpu_execute_irq(cpu_t *cpu);
void cpu_execute_nmi(cpu_t *cpu);

void cpu_log(cpu_t *cpu, FILE *f);
void cpu_state(cpu_t *cpu);
void cpu_disassemble(cpu_t *cpu, uint16_t from, uint16_t to, FILE *f);

typedef uint16_t (*cpu_addr_mode_f)(cpu_t *cpu);
typedef void (*cpu_instr_f)(cpu_t *cpu, uint16_t data);

typedef struct {
  const char *name;
  cpu_instr_f operate;
  cpu_addr_mode_f fetch_data;
  uint8_t cycles;
} cpu_instr_t;

typedef struct {
  struct {
    char name[4];
    uint8_t prg_rom_chunks;
    uint8_t chr_rom_chunks;
    uint8_t mapper1;
    uint8_t mapper2;
    uint8_t ptg_ram_size;
    uint8_t tv_system1;
    uint8_t tv_system2;
    char _[5];  // unused
  } header;

  uint8_t mapper_id;
  uint8_t prg_banks;
  uint8_t chr_banks;

  uint8_t *prg_memory;
  size_t prg_memory_size;
  uint8_t *chr_memory;
  size_t chr_memory_size;
} cartridge_t;

void cartridge_init(cartridge_t *card, const char *path);

// * MAPPER *

typedef struct {
  enum {
    m000,
  } type;

  uint8_t prg_banks;
  uint8_t chr_banks;
} cpu_mapper_t;

void cpu_mapper_init(cpu_mapper_t *mapper, uint8_t prg_banks,
                     uint8_t chr_banks);

uint8_t cpu_mapper_read_u8(cpu_mapper_t *mapper, uint16_t addr,
                           uint32_t *mapped_addr);
uint8_t cpu_mapper_write_u8(cpu_mapper_t *mapper, uint16_t addr,
                            uint32_t *mapped_addr);

// * INSTRUCTIONS *

extern const cpu_instr_t cpu_instr_lookup[0x100];

uint16_t cpu_addr_mode_imp(cpu_t *cpu);
uint16_t cpu_addr_mode_imm(cpu_t *cpu);
uint16_t cpu_addr_mode_zp0(cpu_t *cpu);
uint16_t cpu_addr_mode_zpx(cpu_t *cpu);
uint16_t cpu_addr_mode_zpy(cpu_t *cpu);
uint16_t cpu_addr_mode_rel(cpu_t *cpu);
uint16_t cpu_addr_mode_abs(cpu_t *cpu);
uint16_t cpu_addr_mode_abx(cpu_t *cpu);
uint16_t cpu_addr_mode_aby(cpu_t *cpu);
uint16_t cpu_addr_mode_ind(cpu_t *cpu);
uint16_t cpu_addr_mode_izx(cpu_t *cpu);
uint16_t cpu_addr_mode_izy(cpu_t *cpu);

void cpu_instr_adc(cpu_t *cpu, uint16_t data);
void cpu_instr_and(cpu_t *cpu, uint16_t data);
void cpu_instr_asl(cpu_t *cpu, uint16_t data);
void cpu_instr_bcc(cpu_t *cpu, uint16_t data);
void cpu_instr_bcs(cpu_t *cpu, uint16_t data);
void cpu_instr_beq(cpu_t *cpu, uint16_t data);
void cpu_instr_bit(cpu_t *cpu, uint16_t data);
void cpu_instr_bmi(cpu_t *cpu, uint16_t data);
void cpu_instr_bne(cpu_t *cpu, uint16_t data);
void cpu_instr_bpl(cpu_t *cpu, uint16_t data);
void cpu_instr_brk(cpu_t *cpu, uint16_t data);
void cpu_instr_bvc(cpu_t *cpu, uint16_t data);
void cpu_instr_bvs(cpu_t *cpu, uint16_t data);
void cpu_instr_clc(cpu_t *cpu, uint16_t data);
void cpu_instr_cld(cpu_t *cpu, uint16_t data);
void cpu_instr_cli(cpu_t *cpu, uint16_t data);
void cpu_instr_clv(cpu_t *cpu, uint16_t data);
void cpu_instr_cmp(cpu_t *cpu, uint16_t data);
void cpu_instr_sax(cpu_t *cpu, uint16_t data);
void cpu_instr_cpx(cpu_t *cpu, uint16_t data);
void cpu_instr_cpy(cpu_t *cpu, uint16_t data);
void cpu_instr_dec(cpu_t *cpu, uint16_t data);
void cpu_instr_dex(cpu_t *cpu, uint16_t data);
void cpu_instr_dey(cpu_t *cpu, uint16_t data);
void cpu_instr_dcm(cpu_t *cpu, uint16_t data);
void cpu_instr_eor(cpu_t *cpu, uint16_t data);
void cpu_instr_inc(cpu_t *cpu, uint16_t data);
void cpu_instr_inx(cpu_t *cpu, uint16_t data);
void cpu_instr_iny(cpu_t *cpu, uint16_t data);
void cpu_instr_isc(cpu_t *cpu, uint16_t data);
void cpu_instr_jmp(cpu_t *cpu, uint16_t data);
void cpu_instr_jsr(cpu_t *cpu, uint16_t data);
void cpu_instr_lda(cpu_t *cpu, uint16_t data);
void cpu_instr_ldx(cpu_t *cpu, uint16_t data);
void cpu_instr_ldy(cpu_t *cpu, uint16_t data);
void cpu_instr_lax(cpu_t *cpu, uint16_t data);
void cpu_instr_lsr(cpu_t *cpu, uint16_t data);
void cpu_instr_nop(cpu_t *cpu, uint16_t data);
void cpu_instr_ora(cpu_t *cpu, uint16_t data);
void cpu_instr_pha(cpu_t *cpu, uint16_t data);
void cpu_instr_php(cpu_t *cpu, uint16_t data);
void cpu_instr_pla(cpu_t *cpu, uint16_t data);
void cpu_instr_plp(cpu_t *cpu, uint16_t data);
void cpu_instr_rol(cpu_t *cpu, uint16_t data);
void cpu_instr_ror(cpu_t *cpu, uint16_t data);
void cpu_instr_rti(cpu_t *cpu, uint16_t data);
void cpu_instr_rts(cpu_t *cpu, uint16_t data);
void cpu_instr_sbc(cpu_t *cpu, uint16_t data);
void cpu_instr_sec(cpu_t *cpu, uint16_t data);
void cpu_instr_sed(cpu_t *cpu, uint16_t data);
void cpu_instr_sei(cpu_t *cpu, uint16_t data);
void cpu_instr_sta(cpu_t *cpu, uint16_t data);
void cpu_instr_stx(cpu_t *cpu, uint16_t data);
void cpu_instr_sty(cpu_t *cpu, uint16_t data);
void cpu_instr_tax(cpu_t *cpu, uint16_t data);
void cpu_instr_tay(cpu_t *cpu, uint16_t data);
void cpu_instr_tsx(cpu_t *cpu, uint16_t data);
void cpu_instr_txa(cpu_t *cpu, uint16_t data);
void cpu_instr_txs(cpu_t *cpu, uint16_t data);
void cpu_instr_tya(cpu_t *cpu, uint16_t data);

void cpu_instr_xxx(cpu_t *cpu, uint16_t data);

#endif  // CPU_H_
