
#include "tests/nestest.h"

#include <assert.h>
#include <string.h>

#include "inc/cpu.h"

int main(void) {
  const char* rom_path = "tests/nestest.nes";

  cpu_t cpu = {0};
  memory_t mem;
  cpu_init(&cpu, &mem);

  cartridge_t cart;
  cartridge_init(&cart, rom_path);

  memcpy(&mem.buffer[0xc000], cart.prg_memory, cart.prg_memory_size);

  cpu.pc = 0xc000;

  for (size_t i = 0; i < NESTEST_EXPECT_SIZE; i++) {
    cpu_state(&cpu);

    nestest_entry_t entry = nestest_expect[i];

    assert(cpu.pc == entry.pc);
    assert(cpu.a == entry.reg_a);
    assert(cpu.x == entry.reg_x);
    assert(cpu.y == entry.reg_y);
    assert(cpu.sr == entry.reg_p);
    assert(cpu.sp == entry.reg_sp);

    cpu_step(&cpu);
  }

  printf("[INFO] All %d tests pass.\r\n", NESTEST_EXPECT_SIZE);

  return 0;
}
