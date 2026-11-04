# AP68030 (030-70mhz): the pipelined core as an MC68030

A Motorola **MC68030** in SystemVerilog, built from the pipelined core of
the AP68060 (branch `060-70mhz`): its fetch unit, decoder, microcode
sequencer, AG-DC1-DC2-EX-WB back end and data memory unit, remade to the
68030's architecture and put on the 68030's bus. It targets the MiSTer's
Cyclone V (5CSEBA6U23I7) at **70 MHz**. It is validated in Verilator
against AP68030's self-checking programs, at the pins, through AP68030's
program bench.

The 68030 pieces the pipeline did not have come from AP68030, the
sequential MC68030 core this repository started with: the bus controller
(`ap030_bus.v`), the on-chip caches (`ap030_cache.v`), the PMMU
(`ap030_mmu.v`) and the coprocessor protocol (ported into
`ap68030_cpif.sv`).

## Status at a glance

Measured at commit `8361a50` (default build); the 8 KB / FPU build at `8157742`.

| | |
|---|---|
| Target | Cyclone V 5CSEBA6U23I7 (MiSTer), out-of-context fit at 70 MHz (14.286 ns) |
| Area, default build (68030 caches, coprocessor interface, no FPU) | 21,899 ALMs, 14,546 registers |
| Area, 8 KB caches and on-chip FPU | 24,432 ALMs, 17,401 registers, 339,968 block memory bits, 15 DSP blocks |
| Timing | Default build: +0.562 ns at 70 MHz, Fmax 72.9 MHz, no failing endpoint. 8 KB caches and on-chip FPU: +0.006 ns, Fmax 70.0 MHz |
| Regression | AP68030's 18 programs, each plain and with wait states, plus Dhrystone: all 37 runs pass (default build) |
| Dhrystone 2.1 | 0.133 DMIPS/MHz with the 68030 caches; 0.375 DMIPS/MHz with the 8 KB caches (AP68030 itself: 0.235) |
| Corpus | The WinUAE 68030 `cputest` corpus has not been run on this core yet |

**Open**:

- **FPU = 1** is the on-chip FPU with the 68060's rules: the transcendental
  instructions, packed decimal, FScc/FDBcc/FTRAPcc and denormal operands
  trap as on the 68060. A 68881/68882's behaviour would need them in
  hardware.
- **Speed with the 68030 caches**: every cache hit goes through the bus
  interface (grant, then the lookup the next clock), so the default build is
  slower per clock than AP68030 itself.
  A fast path for the 256-byte caches in the data memory unit and the
  fetch unit is the next performance step.

## Configuration

Top level: [rtl/ap68030.sv](rtl/ap68030.sv), with AP68030's `ap030_top`
ports (the ones this core has no use for are kept and tied off).

| Parameter | Effect |
|---|---|
| `CACHE030` | 1 (default): the MC68030's 256-byte instruction and data caches (UM section 6: 16 lines of four long words, a valid bit per long word, logical tags with the function code, CACR EI/FI/IBE/ED/FD/DBE/WA, CI/CEI/CD/CED with CAAR), at the bus interface; the pipeline's own caches are off. 0: the pipeline's 8 KB four-way caches under the 68030's CACR (faster; see below). |
| `FPU` | 0 (default): no FPU (none is synthesized); F-line instructions with CpID 1-7 run the coprocessor interface (UM section 10) on the bus, so an external 68881/68882 works and without one they take the F-line exception. 1: the on-chip FPU (68060 rules, see Open). |

With `CACHE030 = 0` the caches are bigger but not the 68030's: aliases of
one physical line do not evict each other, a snoop invalidates by address
rather than by cache index, and the caches cannot keep function codes
apart, so they are off while the PMMU translates (TC.E). AP68030's checks
that depend on the 256-byte geometry fail in this mode by design:
`t_bus` 35, `t_cache` 6, `t_held` 37, `t_mmu` 140, `t_snoop` 17-18.

## What is implemented

- **Programming model**: the 68030's ISA (CHK2/CMP2, CAS/CAS2 with no write
  on a mismatch, MOVEP, 64-bit MULL/DIVL, BKPT with the acknowledge and
  its replacement opcode), MSP and ISP with SR.M, T1 and T0 tracing, the
  control registers SFC, DFC, CACR, USP, VBR, CAAR, MSP, ISP.
- **Exceptions**: frames $0, $1, $2, $9, $A and $B and their RTE.
  - Bus and address errors stack $B (the SSW, the failed cycle's address
    and data); a write whose failed cycle ends its instruction stacks $A.
  - RTE reruns a faulted write (SSW.DF set), completes a faulted read from
    the data input buffer (DF clear), or resumes after a read-modify-write
    the handler emulated.
- **Posted writes**: one write pending, as the 68030's write buffer. Its
  bus error is taken at the next boundary ($A), or, when a later access
  waits for it, there ($B, the waiting instruction restarts).
- **Bus**: the 68030's pins through `ap030_bus.v`: S0-S5 cycles, dynamic
  bus sizing, STERM synchronous cycles, CBREQ/CBACK bursts, CIIN, RMC,
  BERR/HALT retry, bus arbitration, RESET.
- **PMMU**: TC, CRP, SRP, TT0, TT1, MMUSR; the table search (short and long
  descriptors, early termination, indirect descriptors, limits, the
  function code level, U and M updates); PMOVE, PTEST, PLOAD, PFLUSH and
  the MMU configuration exception; MMUDIS. A denied access is a bus error.
- **Coprocessor interface** (`FPU = 0`): cpGEN with every response
  primitive, cpBcc, cpDBcc, cpScc, cpTRAPcc, cpSAVE, cpRESTORE; interrupts
  in a come-again dialogue with a format $9 frame, resumed by RTE.

## Architecture

```
            +-----------+   +----------+   +----------+
            |  fetch    |-->| D1       |-->| D2       |--+
            |  I-queue  |   | decode   |   | micro-   |  |  uops
            +-----------+   +----------+   | sequencer|  |
                 ^                         +----------+  v
                 |           +-----+   +-----+   +-----+   +----+   +----+
                 |           | AG  |-->| DC1 |-->| DC2 |-->| EX |-->| WB |--+
                 |           +-----+   +-----+   +-----+   +----+   +----+  |
                 |                    \______ DMU _______/                   |
                 |                     store buffer (1)          coprocessor |
                 v                               v               interface   v
            +-----------------------------------------------------------------+
            |  ap68030_biu: PMMU (ap030_mmu), 68030 I/D caches (ap030_cache), |
            |  table searches, DIB completion   ->   ap030_bus   -> 68030 pins |
            +-----------------------------------------------------------------+
```

- [ap68060_fetch.sv](rtl/ap68060_fetch.sv), [ap68060_decode.sv](rtl/ap68060_decode.sv),
  [ap68060_useq.sv](rtl/ap68060_useq.sv), [ap68060_backend.sv](rtl/ap68060_backend.sv),
  [ap68060_dmu.sv](rtl/ap68060_dmu.sv): the pipeline (file names kept from the AP68060).
- [ap68030_biu.sv](rtl/ap68030_biu.sv): the requesters' interface on
  `ap030_bus`, with the PMMU and the 68030 caches. Clients: 0 the DMU,
  1 the fetch unit, 2 the coprocessor interface.
- [ap68030_cpif.sv](rtl/ap68030_cpif.sv): the coprocessor interface, a
  sequential engine the back end starts when the instruction reaches WB.
- [tools/ucode.py](tools/ucode.py): the microcode (68030 routines: the
  frames, RTE of $A/$B/$9, the MMU instructions).

## Verification

```sh
cd tb030
./run_tests.sh                          # default build: all programs, plain and +waits=2
CACHE030=0 ./run_tests.sh /tmp/obj8k    # the 8 KB cache build
PROGS="t_mmu t_cp" NOC=1 ./run_tests.sh # a subset, without the C programs
```

Needs Verilator 5.x, `vasmm68k_mot` and Python 3 (vbcc for Dhrystone).
The programs are AP68030's ([tb030/asm/](tb030/asm/)): `t_integer`,
`t_exceptions`, `t_stack`, `t_bus`, `t_cache`, `t_snoop`, `t_mmu`,
`t_pmmuenc`, `t_extword`, `t_held`, `t_initstruct`, `t_ipendmask`,
`t_dblfault`, `t_cp` and the `t_cpx_*` coprocessor scripts.

## Synthesis

```sh
cd syn
./fitcore030.sh f1                               # default build, 70 MHz
PARAMS="CACHE030=0 FPU=1" ./fitcore030.sh f2     # 8 KB caches, on-chip FPU
```

An out-of-context fit: every port is a virtual pin, so the result is the
core's register-to-register timing.

## Documentation

- [doc/PLAN-M68030.md](doc/PLAN-M68030.md): the remake's plan, decisions and status.
- [doc/STATUS.md](doc/STATUS.md): the AP68060 core this was built from.

## License

GNU General Public License, version 2 or later: see [LICENSE](LICENSE).
