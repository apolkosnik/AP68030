#!/bin/sh
# ys030.sh <name> [PARAM=VALUE ...]: a Yosys estimate of ap68030's ALMs on
# Cyclone V (synth_intel_alm), hierarchy kept, per-module statistics.
# sv2v converts the SystemVerilog; the numbers are an estimate to compare
# variants, not Quartus' count (syn/fitcore030.sh is that).
set -e
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../../rtl}
name=$1; shift
out=$here/$name; rm -rf "$out"; mkdir -p "$out"
FILES="ap030_defs.svh ap68060_pkg.sv gen/ap68060_upkg.sv ap68060_alu.sv ap68060_alu_slow.sv ap68060_muldiv.sv ap030_bus.v ap030_cache.v ap030_mmu.v ap68030_biu.sv ap68030_cpif.sv ap68060_ram.sv ap68060_atc.sv ap68060_dmu.sv ap68060_predec.sv ap68060_fetch.sv ap68060_decode.sv ap68060_useq.sv fpu/ap040_fpu.v ap68060_fpif.sv ap68060_backend.sv ap68060_snoop.sv ap68030.sv"
# a copy of the sources without the simulation-only regions (translate_off ..
# translate_on), which Yosys cannot read
src=$out/src; mkdir -p $src/gen $src/fpu
for f in $FILES $(cd $R && ls *.svh gen/*.svh fpu/*.svh 2>/dev/null); do
  python3 $here/strip_sim.py $R/$f > $src/$f
done
R=$src
paths=""; for f in $FILES; do paths="$paths $R/$f"; done
sv2v -I $R -I $R/gen -I $R/fpu --top ap68030 $paths 2> $out/sv2v.log | sed 's/\$finish;/;/' > $out/ap68030.v
set -- "$@"
chp=""; for p in "$@"; do chp="$chp; chparam -set ${p%%=*} ${p#*=} ap68030"; done
yosys -q -l $out/yosys.log -p "read_verilog -sv -nodisplay $out/ap68030.v; hierarchy -top ap68030 $chp; synth_intel_alm -family cyclonev -top ap68030; tee -o $out/stat.txt stat" > /dev/null 2>&1 || { tail -20 $out/yosys.log; exit 1; }
