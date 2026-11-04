#!/bin/sh
# Out-of-context fit of the 68030 core (ap68030) on the MiSTer part at
# 70 MHz: syn/fitcore030.sh <work dir name> [mkooc script]
#   R=<rtl dir>  PERIOD=<ns> (default 14.286)  PARAMS="CACHE030=0 FPU=1"
here=$(cd "$(dirname "$0")" && pwd)
R=${R:-$here/../rtl}
S=${2:-$here/mkooc.sh}
cd "$here"
exec $S $1 ap68030 ${PERIOD:-14.286} $R/ap030_defs.svh $R/ap68060_pkg.sv $R/gen/ap68060_upkg.sv $R/ap68060_alu.sv $R/ap68060_alu_slow.sv $R/ap68060_muldiv.sv $R/ap030_bus.v $R/ap030_cache.v $R/ap030_mmu.v $R/ap68030_biu.sv $R/ap68030_cpif.sv $R/ap68060_ram.sv $R/ap68060_atc.sv $R/ap68060_dmu.sv $R/ap68060_predec.sv $R/ap68060_fetch.sv $R/ap68060_decode.sv $R/ap68060_useq.sv $R/fpu/ap040_fpu.v $R/ap68060_fpif.sv $R/ap68060_backend.sv $R/ap68060_snoop.sv $R/ap68030.sv
