#!/bin/sh
# Synthesise, fit and time the core for a Cyclone V (timing check only).
# Usage: sh run_syn.sh   (writes syn/output/*.rpt; the PID is in syn/quartus.pid)
# Requires Python 3; fails if any reported timing corner misses its constraints.
set -eu
cd "$(dirname "$0")"
Q=${QUARTUS_ROOTDIR:-/opt/intelFPGA_lite/17.0/quartus}/bin
echo $$ > quartus.pid
$Q/quartus_map ap68030 > map.log 2>&1
$Q/quartus_fit ap68030 > fit.log 2>&1
$Q/quartus_sta ap68030 > sta.log 2>&1
grep -A5 '^; Slow .* Model Fmax Summary' output/ap68030.sta.rpt
python3 check_timing.py output/ap68030.sta.summary
