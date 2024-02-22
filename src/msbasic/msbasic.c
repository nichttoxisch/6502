
#include <ncurses.h>
#include <stdbool.h>

#include "inc/cpu.h"

#define OUT_DATA 0x5000
#define IN_DATA_READY 0x5001
#define IN_DATA 0x5002

static cpu_t cpu = {0};
static memory_t mem;

static int last_key = -1;

void get_char(void) {
  if (mem.buffer[IN_DATA_READY] != 0) return;

  int ich = getch();
  if (ich == -1) {
    last_key = ich;
    return;
  };

  if (ich == last_key) return;

  mem.buffer[IN_DATA_READY] = 0x08;
  if (ich == '\n') ich = '\r';
  mem.buffer[IN_DATA] = ich;
  last_key = ich;
}

int main(void) {
  const char* path = "roms/msbasic.nes";
  FILE* f = fopen(path, "rb");
  if (!f) {
    fprintf(stderr, "[ERROR] Could not open '%s'.", path);
  }

  fread(&mem.buffer[0x8000], 1, 0x8000, f);
  fclose(f);

  const char* log_path = "log.txt";
  FILE* f_log = fopen(log_path, "wb");
  if (!f_log) {
    fprintf(stderr, "[ERROR] Could not open '%s'.", log_path);
  }

  cpu_init(&cpu, &mem);

  initscr();
  cbreak();
  noecho();
  scrollok(stdscr, TRUE);
  nodelay(stdscr, TRUE);

  while (1) {
    // cpu_log(&cpu, f_log);
    cpu_step(&cpu);

    char ch = mem.buffer[OUT_DATA];
    if (ch != 0) {
      printf("%c", ch);
      fflush(stdout);
      mem.buffer[OUT_DATA] = 0;
    }

    get_char();

    // napms(1);
  }

  fclose(f_log);

  return 0;
}
