#!/bin/sh
# rebuild the program bench and run one program; prints the failing test's source
cd "$(dirname "$0")"
RTL=../rtl
if [ "${NOBUILD:-0}" != 1 ]; then
verilator --binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
  --output-split 20000 --output-split-cfuncs 500 -CFLAGS -O1 -I$RTL -I$RTL/core --top-module tb_ap030_program \
  --Mdir build/obj_prog -o tb_prog tb_ap030_program.sv $RTL/ap030_top.v $RTL/ap030_core.v $RTL/ap030_memsys.v \
  $RTL/ap030_mmu.v $RTL/ap030_cache.v $RTL/ap030_bus.v $RTL/ap030_alu.v $RTL/ap030_muldiv.v $RTL/ap030_regfile.v \
  > build/build_prog.log 2>&1 || { grep -E "^%Error" build/build_prog.log | head; exit 1; }
fi
t=$1; shift
vasmm68k_mot -Fbin -m68030 -m68851 -no-opt -o build/$t.bin asm/$t.s > build/$t.asm.log 2>&1 || { cat build/$t.asm.log; exit 1; }
python3 bin2hex.py build/$t.bin build/$t.hex
./build/obj_prog/tb_prog +prog=build/$t.hex "$@" > build/$t.log 2>&1
grep -E "program reports|TEST FAILED|ALL TESTS|FAIL:|halted" build/$t.log | head -5
n=$(grep -o "TEST FAILED number [0-9]*" build/$t.log | awk '{print $4}')
if [ -n "$n" ]; then echo "--- source of test $n:"; grep -n -B6 -E "[ ,]$n(\s|$)" asm/$t.s | grep -v "^--$" | tail -8; fi
