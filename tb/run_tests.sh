#!/bin/sh
# AP68030 self-test suite under Verilator.
# Needs verilator (5.x), vasmm68k_mot (vbcc) and python3.
# Usage: sh run_tests.sh [workdir]
set -eu
cd "$(dirname "$0")"

VASM=${VASM:-vasmm68k_mot}
RTL=../rtl
WORK=${1:-build}
mkdir -p "$WORK"

SRC="$RTL/ap030_top.v $RTL/ap030_core.v $RTL/ap030_memsys.v $RTL/ap030_mmu.v $RTL/ap030_cache.v \
     $RTL/ap030_bus.v $RTL/ap030_alu.v $RTL/ap030_muldiv.v $RTL/ap030_regfile.v"

PROGS="t_integer t_exceptions t_bus t_cache t_mmu t_cp"
HALTPROGS="t_dblfault"

echo "== assembling test programs =="
for t in $PROGS $HALTPROGS; do
	[ -f "asm/$t.s" ] || continue
	$VASM -Fbin -m68030 -m68851 -no-opt -o "$WORK/$t.bin" "asm/$t.s" >/dev/null
	python3 bin2hex.py "$WORK/$t.bin" "$WORK/$t.hex"
done

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
build prog tb_ap030_program tb_ap030_program.sv $SRC &
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
run bus "$WORK/obj_bus/tb_bus"
for t in $PROGS; do
	[ -f "$WORK/$t.hex" ] || continue
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex"
	run "${t}_waits" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +waits=2
done
for t in $HALTPROGS; do
	[ -f "$WORK/$t.hex" ] || continue
	run "$t" "$WORK/obj_prog/tb_prog" "+prog=$WORK/$t.hex" +expect_halt
done

if [ $fail -eq 0 ]; then echo "AP68030: ALL TESTS PASSED"; else echo "AP68030: FAILURES"; exit 1; fi
