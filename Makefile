CC = clang
CFLAGS = -Wunused-function -Wall -Wextra -pedantic -ggdb -O0
INC = -I.
TARGET_LIB = lib/libcpu.a

all: lib

always:
	mkdir -p build lib roms

msbasic: lib msbasic_rom always
	$(CC) $(CFLAGS) -o build/msbasic src/msbasic/msbasic.c -L./lib/ -lcpu -lncurses  $(INC)
	./build/msbasic

msbasic_rom: always
	ca65 -D msbasic src/msbasic/msbasic/msbasic.s -o build/msbasic.o
	ld65 -C src/msbasic/msbasic/msbasic.cfg build/msbasic.o -o roms/msbasic.nes -Ln build/msbasic.lbl

wozmon: lib wozmon_rom always
	$(CC) $(CFLAGS) -o build/wozmon src/wozmon/wozmon.c -L./lib/ -lcpu -lncurses  $(INC)
	./build/wozmon

wozmon_rom: always
	ca65 src/wozmon/wozmon.asm -o build/wozmon.o
	ld65 -C src/wozmon/wozmon.cfg -vm -m build/wozmon.map -o roms/wozmon.nes build/wozmon.o
	
nestest: lib always
	$(CC) $(CFLAGS)  -o build/nestest src/nestest.c -L./lib/ -lcpu $(INC)
	./build/nestest

test: lib always
	$(CC) $(CFLAGS)  -o build/test src/test.c $(TARGET_LIB) $(INC)
	./build/test

lib: always
	$(CC) $(CFLAGS) src/cpu.c -c -o build/cpu.o  $(INC)
	ar rv $(TARGET_LIB) build/cpu.o 
	ranlib $(TARGET_LIB)

clean: 
	rm -rf build lib roms 
