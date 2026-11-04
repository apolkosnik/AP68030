#!/bin/sh
# Verilate the 68030 cputest replay bench (tb_cputest030.sv) and assemble its
# monitor.  WORK= the object directory (default ../obj030cpt);
# CACHE030= FPU= COPROC= as run_tests.sh.
set -e
cd "$(dirname "$0")"
RTL=../rtl
WORK=${WORK:-$(cd .. && pwd)/obj030cpt}
mkdir -p "$WORK"
python3 ../tools/ucode.py > /dev/null
${VASM:-vasmm68k_mot} -Fbin -m68030 -m68851 -no-opt -o "$WORK/cputest_mon030.bin" asm/cputest_mon030.s > /dev/null
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
	-Werror-PINMISSING -Werror-IMPLICIT \
	--top-module tb_cputest --Mdir "$WORK/obj_cputest" -o tb_cputest \
	-I"$RTL" -I"$RTL/fpu" -CFLAGS "-O2 -march=native" \
	$RTL/ap68060_pkg.sv $RTL/gen/ap68060_upkg.sv $RTL/ap68060_alu.sv $RTL/ap68060_alu_slow.sv $RTL/ap68060_muldiv.sv \
	$RTL/ap030_bus.v $RTL/ap030_cache.v $RTL/ap030_mmu.v $RTL/ap68030_biu.sv $RTL/ap68030_cpif.sv \
	$RTL/ap68060_ram.sv $RTL/ap68060_atc.sv $RTL/ap68060_dmu.sv \
	$RTL/ap68060_predec.sv $RTL/ap68060_fetch.sv \
	$RTL/ap68060_decode.sv $RTL/ap68060_useq.sv $RTL/fpu/ap040_fpu.v $RTL/ap68060_fpif.sv $RTL/ap68060_backend.sv \
	$RTL/ap68060_snoop.sv $RTL/ap68030.sv \
	tb_cputest030.sv > "$WORK/build.log" 2>&1 || { grep -E "%Error" "$WORK/build.log" | head -20; exit 1; }
echo "built $WORK/obj_cputest/tb_cputest"
