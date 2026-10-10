#!/bin/sh
# AP68030 self-test suite under Verilator.
# Needs verilator (5.x), vasmm68k_mot (vbcc) and python3.
# Usage: [FAST_PORT=1] sh run_tests.sh [workdir]
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl
WORK=${1:-build}
mkdir -p "$WORK"

SRC="$RTL/ap030_top.v $RTL/ap030_core.v $RTL/ap030_memsys.v $RTL/ap030_mmu.v $RTL/ap030_cache.v \
     $RTL/ap030_bus.v $RTL/ap030_alu.v $RTL/ap030_muldiv.v $RTL/ap030_regfile.v"

PROGS="t_integer t_exceptions t_bus t_cache t_mmu t_cp t_stack t_snoop t_extword t_pmmuenc t_held"
HALTPROGS="t_dblfault"
LAZYPROGS="t_lazy"     # run with +lazy only: they drive the pipeline-model inputs
# programs on the coprocessor model's script (command $0030): asm/t_cpx_*.s
PROGS="$PROGS $(cd asm && ls t_cpx_*.s 2>/dev/null | sed 's/\.s$//' | tr '\n' ' ')"

echo "== assembling test programs =="
for t in $PROGS $HALTPROGS $LAZYPROGS; do
	[ -f "asm/$t.s" ] || continue
	$VASM -Fbin -m68030 -m68851 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

# compiled C programs (tb/c): vbcc for the 68030, linked flat behind
# tb/c/start.s (vectors at 0, code from $400) by vlink.  dhry is Dhrystone
# 2.1; it checks its final values and reports clocks, Dhrystones/s and DMIPS.
VBCC=${VBCC:-/opt/amiga-cc/vbcc}
export VBCC
CPROGS=""
if [ -x "$VBCC/bin/vc" ]; then
	VC="$VBCC/bin/vc"; VLINK="$VBCC/bin/vlink"
	$VASM -quiet -Fhunk -m68030 -o "$WORK/start.o" c/start.s
	$VC +aos68k -c -O2 -speed -cpu=68030 -DTIME -o "$WORK/dhry_1.o" c/dhry_1.c > "$WORK/dhry.compile.log" 2>&1
	$VC +aos68k -c -O2 -speed -cpu=68030 -DTIME -o "$WORK/dhry_2.o" c/dhry_2.c >> "$WORK/dhry.compile.log" 2>&1
	$VLINK -brawbin1 -o "$WORK/dhry.bin" "$WORK/start.o" "$WORK/dhry_1.o" "$WORK/dhry_2.o"
	python3 bin2hex.py "$WORK/dhry.bin" "$WORK/dhry.hex"
	CPROGS="dhry"
else
	echo "  (vbcc not found at $VBCC: compiled C programs skipped)"
fi

VFLAGS="--binary --timing -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE \
        --output-split 20000 --output-split-cfuncs 500 -CFLAGS -O1 -I$RTL -I$RTL/core"

echo "== compiling =="
build() {
	name=$1; top=$2; shift 2
	# shellcheck disable=SC2086
	verilator $VFLAGS --top-module "$top" --Mdir "$WORK/obj_$name" -o "tb_$name" "$@" \
	    > "$WORK/build_$name.log" 2>&1 || { echo "  BUILD FAIL  $name  (see $WORK/build_$name.log)"; exit 1; }
}
build bus  tb_ap030_bus     tb_ap030_bus.sv $RTL/ap030_bus.v &
pid_bus=$!
build prog tb_ap030_program "-GFAST_PORT=${FAST_PORT:-0}" tb_ap030_program.sv $SRC &
pid_prog=$!
wait $pid_bus || exit 1
wait $pid_prog || exit 1

echo "== running =="
fail=0
run() {
	name=$1; shift
	rc=0
	"$@" > "$WORK/$name.log" 2>&1 || rc=$?
	if [ "$rc" -eq 0 ] && grep -q "ALL TESTS PASSED" "$WORK/$name.log" && ! grep -Eq 'TEST FAILED|FAIL:' "$WORK/$name.log"; then
		echo "  pass  $name"
	else
		echo "  FAIL  $name  (see $WORK/$name.log)"
		fail=1
	fi
}
# the processor clocks and instructions a passing run reports
counts() { grep -o "ALL TESTS PASSED ([0-9]* clocks, [0-9]* instructions" "$WORK/$1.log" || true; }
same_clocks() {
	if [ -n "$(counts "$1")" ] && [ "$(counts "$1")" != "$(counts "$2")" ]; then
		echo "  FAIL  $1  takes other clocks than $2: $(counts "$1" | cut -c19-) / $(counts "$2" | cut -c19-)"
		fail=1
	fi
}
run bus "$WORK/obj_bus/tb_bus"
for t in $PROGS; do
	[ -f "$WORK/$t.hex" ] || continue
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex"
	run "${t}_waits" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +waits=2
	# with the processor clock enable: half and quarter rate, and a random
	# pattern; the bench runs on the processor clock, so each must take
	# exactly the processor clocks of the run without the enable
	run "${t}_ce2" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +ce=2
	same_clocks "${t}_ce2" "$t"
	run "${t}_ce4_waits" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +ce=4 +waits=2
	same_clocks "${t}_ce4_waits" "${t}_waits"
	run "${t}_cerand" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +ce_rand
	same_clocks "${t}_cerand" "$t"
	# Hatari-style instruction prefetch (fetch_lazy), at half rate with wait states
	run "${t}_lazy" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +lazy +ce=2 +waits=2
done
for t in $LAZYPROGS; do
	[ -f "$WORK/$t.hex" ] || continue
	# a hang (the guard against wrong model inputs failing) ends at maxclk
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +lazy +maxclk=400000
	run "${t}_ce2_waits" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +lazy +ce=2 +waits=2 +maxclk=800000
	run "${t}_cerand" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +lazy +ce_rand +maxclk=1600000
	same_clocks "${t}_cerand" "$t"
done
for t in $CPROGS; do
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +maxclk=20000000
	run "${t}_waits" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +maxclk=40000000 +waits=2
	grep -h "^BENCH" "$WORK/$t.log" "$WORK/${t}_waits.log" | sed 's/^/        /'
done
for t in $HALTPROGS; do
	[ -f "$WORK/$t.hex" ] || continue
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +expect_halt
done

if [ $fail -eq 0 ]; then echo "AP68030: ALL TESTS PASSED"; else echo "AP68030: FAILURES"; exit 1; fi
