#!/bin/sh
# AP68030 (070) regression: AP68030's MC68030 programs on the pipelined
# core, through AP68030's pin-level program bench (tb_ap68030_program.sv:
# 1 MB of RAM behind ports of several widths, the test registers, the
# coprocessor model).  Needs verilator (5.x), vasmm68k_mot and python3.
# Usage: run_tests.sh [workdir] (default $WORK, else ../obj030)
#        PROGS="t_integer ..." to run a subset; JOBS=n in parallel;
#        CACHE030=0 for the pipeline's 8 KB caches (default 1: the 68030's).
# (The clock-enable runs of AP68030's own regression (+ce, +ce_rand) and
# its pipeline-model programs (+lazy, t_lazy) do not apply: this core runs
# on every clock and has no prefetch model hooks.)
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl
WORK=${1:-${WORK:-$(cd .. && pwd)/obj030}}
JOBS=${JOBS:-8}
mkdir -p "$WORK"

DEFPROGS="t_integer t_exceptions t_bus t_cache t_mmu t_cp t_stack t_snoop t_extword t_pmmuenc t_held t_initstruct t_ipendmask"
DEFPROGS="$DEFPROGS $(cd asm && ls t_cpx_*.s 2>/dev/null | sed 's/\.s$//' | tr '\n' ' ')"
PROGS=${PROGS:-$DEFPROGS}
HALTPROGS=${HALTPROGS-t_dblfault}

echo "== assembling test programs =="
for t in $PROGS $HALTPROGS; do
	[ -f "asm/$t.s" ] || continue
	$VASM -Fbin -m68030 -m68851 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

VBCC=${VBCC:-/opt/amiga-cc/vbcc}
export VBCC
CPROGS=""
if [ -x "$VBCC/bin/vc" ] && [ -z "${NOC:-}" ]; then
	VC="$VBCC/bin/vc"; VLINK="$VBCC/bin/vlink"
	$VASM -quiet -Fhunk -m68030 -o "$WORK/start.o" c/start.s
	$VC +aos68k -c -O2 -speed -cpu=68030 -DTIME -o "$WORK/dhry_1.o" c/dhry_1.c > "$WORK/dhry.compile.log" 2>&1
	$VC +aos68k -c -O2 -speed -cpu=68030 -DTIME -o "$WORK/dhry_2.o" c/dhry_2.c >> "$WORK/dhry.compile.log" 2>&1
	$VLINK -brawbin1 -o "$WORK/dhry.bin" "$WORK/start.o" "$WORK/dhry_1.o" "$WORK/dhry_2.o"
	python3 bin2hex.py "$WORK/dhry.bin" "$WORK/dhry.hex"
	CPROGS="dhry"
fi

echo "== compiling =="
mkdir -p ../tb/build
python3 ../tools/ucode.py > /dev/null
verilator --binary --timing -j 16 --build-jobs 16 -O3 \
	-Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
	-Werror-PINMISSING -Werror-IMPLICIT \
	--top-module tb_ap030_program --Mdir "$WORK/obj_prog" -o tb_prog \
	-I"$RTL" -I"$RTL/fpu" -CFLAGS "-O2 -march=native" \
	"-GFAST_PORT=${FAST_PORT:-0}" "-GCACHE030=${CACHE030:-1}" "-GFPU=${FPU:-0}" "-GCOPROC=${COPROC:-1}" \
	$RTL/ap68060_pkg.sv $RTL/gen/ap68060_upkg.sv $RTL/ap68060_alu.sv $RTL/ap68060_alu_slow.sv $RTL/ap68060_muldiv.sv \
	$RTL/ap030_bus.v $RTL/ap030_cache.v $RTL/ap030_mmu.v $RTL/ap68030_biu.sv $RTL/ap68030_cpif.sv $RTL/ap68060_ram.sv $RTL/ap68060_atc.sv $RTL/ap68060_dmu.sv \
	$RTL/ap68060_predec.sv $RTL/ap68060_fetch.sv \
	$RTL/ap68060_decode.sv $RTL/ap68060_useq.sv $RTL/fpu/ap040_fpu.v $RTL/ap68060_fpif.sv $RTL/ap68060_backend.sv \
	$RTL/ap68060_snoop.sv $RTL/ap68030.sv \
	tb_ap68030_program.sv > "$WORK/build_prog.log" 2>&1 || { grep -E "%Error" "$WORK/build_prog.log" | head -20; exit 1; }

echo "== running =="
SIM="$WORK/obj_prog/tb_prog"
list() {
	for t in $PROGS; do
		[ -f "$WORK/$t.hex" ] || continue
		echo "$t|+prog=$WORK/$t.hex"
		echo "${t}_waits|+prog=$WORK/$t.hex +waits=2"
	done
	for t in $CPROGS; do
		echo "$t|+prog=$WORK/$t.hex +maxclk=20000000"
		echo "${t}_waits|+prog=$WORK/$t.hex +maxclk=40000000 +waits=2"
	done
	for t in $HALTPROGS; do
		[ -f "$WORK/$t.hex" ] && echo "$t|+prog=$WORK/$t.hex +expect_halt"
	done
	return 0
}
list | xargs -d '\n' -P "$JOBS" -I{} sh -c '
	n=${1%%|*}; a=${1#*|}
	# shellcheck disable=SC2086
	'"$SIM"' $a > '"$WORK"'/$n.log 2>&1 || true
	if grep -q "ALL TESTS PASSED" '"$WORK"'/$n.log && ! grep -Eq "TEST FAILED|FAIL:" '"$WORK"'/$n.log; then
		echo "  pass  $n"
	else
		echo "  FAIL  $n  ($(grep -E "TEST FAILED|FAIL" '"$WORK"'/$n.log | head -1))"
	fi' _ {} | sort > "$WORK/results.txt"
cat "$WORK/results.txt"
grep -h "^BENCH" "$WORK"/dhry*.log 2>/dev/null | sed 's/^/        /' || true
if grep -q "FAIL" "$WORK/results.txt"; then echo "AP68030 (070): FAILURES"; exit 1; fi
echo "AP68030 (070): ALL TESTS PASSED"
