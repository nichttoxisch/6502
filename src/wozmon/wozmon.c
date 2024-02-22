
#include <ncurses.h>
#include <stdbool.h>

#include "inc/cpu.h"

#define OUT_DATA 0xD000
#define IN_DATA_READY 0xD001
#define IN_DATA 0xD002

int main(void) {
  cpu_t cpu = {0};
  memory_t mem;

  const char* path = "roms/wozmon.nes";
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "[ERROR] Could not open '%s'.", path);
  }

  fread(&mem.buffer[0x8000], 1, 0x8000, f);
  fclose(f);

  cpu_init(&cpu, &mem);

  initscr();
  cbreak();
  noecho();
  scrollok(stdscr, TRUE);
  nodelay(stdscr, TRUE);

  int last_key = -1;

  while (1) {
    // cpu_state(&cpu);
    cpu_step(&cpu);

    char ch = mem.buffer[OUT_DATA];
    if (ch != 0) {
      printf("%c", ch);
      fflush(stdout);
      mem.buffer[OUT_DATA] = 0;
    }

    if (mem.buffer[IN_DATA_READY] == 0) {
      int ich = getch();
      if (ich != -1) {
        if (ich != last_key) {
          mem.buffer[IN_DATA_READY] = 0x08;
          mem.buffer[IN_DATA] = ich;
          last_key = ich;
        }
      }
      last_key = ich;
    }

    napms(1);
  }

  return 0;
}
