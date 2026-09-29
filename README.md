# AP68030

A synthesisable MC68030 in Verilog: the complete 68020/030 integer
instruction set, the on-chip instruction and data caches, the paged memory
management unit, the coprocessor interface and the MC68030 bus with its
timing, all validated against the *MC68030 User's Manual* (MC68030UM) and
the *M68000 Family Programmer's Reference Manual* with Verilator.

The design targets 50 MHz in a Cyclone V and behaves like the real part at
the pins: half-clock bus states S0-S5, asynchronous (DSACKx, 8/16/32-bit
dynamic bus sizing) and synchronous (STERM) cycles, burst fills with
CBREQ/CBACK, ECS/OCS, DBEN, RMC, CIOUT/CIIN, BERR/HALT retry and rerun,
bus arbitration, interrupt, breakpoint and coprocessor acknowledge cycles
in CPU space, and the RESET instruction driving the RESET pin for 512
clocks.

## Layout

```
rtl/ap030_top.v        pin-level top: 68030 signal set, reset synchroniser
rtl/ap030_core.v       instruction unit: prefetch pipe, decoder, sequencer, exceptions
rtl/core/*.vh          sequencer states (exec_a/b/c), decoder, frame layout, tasks
rtl/ap030_memsys.v     data/instruction units: cache lookup, translation, write buffer,
                       bus arbitration between walker, writes, data and prefetch
rtl/ap030_cache.v      256-byte direct mapped cache (16 lines x 4 entries), logical tags
rtl/ap030_mmu.v        ATC (22 entries), TT0/TT1, table search engine, PTEST/PLOAD/PFLUSH
rtl/ap030_bus.v        bus controller: S0-S5 states, sizing, bursts, retry, arbitration
rtl/ap030_alu.v        ALU, barrel shifter, BCD
rtl/ap030_muldiv.v     32x32 multiply, 64/32 divide
rtl/ap030_regfile.v    D0-D7, A0-A6, USP/ISP/MSP
rtl/ap030_defs.svh     constants
tb/tb_ap030_bus.sv     pin-level bench of the bus controller with slave models
tb/tb_ap030_program.sv program bench: 1 MB RAM behind several port types, test registers
tb/tb_cp_model.svh     scripted coprocessor at CpID 1
tb/asm/*.s             self-test programs (vasm syntax)
tb/run_tests.sh        the regression
syn/                   Quartus project for the timing check
doc/ARCHITECTURE.md    how the core is built
```

## Running the tests

Requirements: Verilator 5.x, `vasmm68k_mot` (vasm with the Motorola syntax
module), Python 3.

```
cd tb
sh run_tests.sh            # everything: bus bench, every program, with and without wait states
./run_prog.sh t_mmu +trace # one program, with the instruction trace (+bustrace, +ctrace, +rftrace, +itrace, +strace)
```

Every passing program reports its clock count, instruction count and the
resulting clocks per instruction; `+strace` prints the sequencer state and
the memory unit's handshakes every clock for profiling.

The programs report through memory-mapped test registers (documented at
the top of `tb/tb_ap030_program.sv`) and print the source of a failing
check.

| program        | covers                                                                       |
|----------------|------------------------------------------------------------------------------|
| `t_integer`    | the integer instruction set, addressing modes, condition codes, bit fields, CAS, MOVEM, MOVEP, BCD |
| `t_exceptions` | traps and frames ($0/$1/$2/$9/$A/$B), illegal/A-line/F-line, CHK/CHK2, privilege, MOVEC/MOVES, trace T1/T0, interrupts (levels, autovector, vectored, spurious, level 7 edge), STOP, master stack and throwaway frames, RTE format errors, BKPT, bus error rerun and software completion on reads, posted writes and instruction fetches, address errors, RESET |
| `t_bus`        | dynamic bus sizing through 32/16/8-bit ports, misaligned operands, line and page crossings, cache hits, MOVEP, TAS/CAS RMW cycles, CIIN, code from narrow ports |
| `t_cache`      | CACR (enable, freeze, clear all/entry, write allocate, burst enable), write-through, the instruction cache and self-modifying code, hit/miss timing |
| `t_mmu`        | MMU registers and configuration exceptions, two- and three-level trees, short and long descriptors, early termination, indirect descriptors, limits, U/M history updates, WP/supervisor faults with RTE rerun, page-crossing operands, instruction fetch faults, harmless prefetches into unmapped pages, PTEST (all levels, An result), PLOAD, PFLUSH variants, PMOVEFD, FCL, SRE, TT0 (FC, R/W, CI), MMUDIS, ATC replacement, cache inhibit |
| `t_cp`         | the coprocessor protocol: every response primitive, cpGEN with all EA forms, cpBcc/cpDBcc/cpScc/cpTRAPcc, cpSAVE/cpRESTORE, busy, exceptions requested by the coprocessor |
| `t_dblfault`   | a bus error while stacking a bus error frame halts the processor |

## Timing and performance

`syn/run_syn.sh` runs Quartus Prime (Cyclone V 5CSEBA6U23I7, 50 MHz
constraint, ports as virtual pins with a 2 ns budget) and prints the Fmax
summary. Quartus Prime 17.0 reports Fmax 53.0 MHz (slow 1100 mV 100 C
corner, slack +0.57 ns at 50 MHz), 17.0k ALMs and 9.5k registers.

On the self-test programs (cache hits, synchronous memory, no wait states)
the core runs at 5.5 clocks per instruction on the integer suite and 6.1
to 6.6 on the exception and MMU suites, which are dominated by long
instructions (absolute long operands, MOVEM, exception frames, table
searches); instruction cache hits stream at one longword per clock, a data
cache hit takes two clocks and consecutive stores run four clocks apart.

## Licence

GPL, see `LICENSE`.
