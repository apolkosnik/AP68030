# AP68030 architecture

References are to the MC68030 User's Manual (UM) sections.

## Overview

```
                 +-----------------------------------------------------------+
                 |  ap030_core: instruction unit                              |
                 |  prefetch pipe (6 words) -> decoder -> sequencer -> ALU   |
                 |  exception frames, RTE rerun, coprocessor dialogue         |
                 +-----------+----------------------------+------------------+
                   i_stb/addr/fc  i_ack/data/fault   d_stb/addr/size/rw/fc  d_ack/rdata/fault
                 +-----------+----------------------------+------------------+
                 |  ap030_memsys: instruction and data units                 |
                 |  icache (logical, FC2)  dcache (logical, FC2-0)  MMU/ATC  |
                 |  write buffer (posted writes)   table walker              |
                 |  bus arbitration: walker > writes > data > prefetch       |
                 +-----------------------------+-----------------------------+
                                               | request / done / fill
                 +-----------------------------+-----------------------------+
                 |  ap030_bus: bus controller (S0-S5 half-clock states)      |
                 +-----------------------------------------------------------+
```

Everything runs from one clock. The bus controller uses both clock edges
because the 68030's bus states are half clocks long: address and control
outputs change on the falling edge where the real part changes them, and
the asynchronous inputs (DSACKx, BERR, HALT, AVEC, CIIN, CBACK, BR, BGACK)
are sampled on the falling edge as UM 7 requires.

## Instruction unit (ap030_core.v, rtl/core/*.vh)

**Prefetch pipe.** Six words are held in `pq[0..5]` with valid and fault
bits; `pq[0]` is stage C (the word at `scan_pc`), `pq[1]` stage B. The
instruction port fetches longwords with up to two fetches outstanding
(`fetch_out`, returned in order; `fetch_disc` counts those a flush made
stale); the instruction unit keeps one request pending while it looks up
another, signals room with `i_ready`, and delivers a cache hit in the
clock its data is valid, so hits stream at one longword per clock. A fetch
that faults marks its words: the exception is taken only when a faulted
word is needed (UM 8.1.2), so a prefetch past the end of a page into an
unmapped one is harmless.

**Decoder.** `ap030_decode.vh` decodes `dw` (stage C at dispatch, else the
latched `ir`) into the first sequencer state, operand size, ALU operation,
source/destination kinds (register, immediate, EA, quick) and whether the
second word belongs to the operation (`dc_needs_ext`); an instruction that
takes an exception at its dispatch (illegal, A-line, F-line, privilege
violation) needs only its operation word, so it neither waits for the next
word nor takes the bus error of its prefetch (UM 8.1.2, 8.1.5). At an instruction
boundary the dispatcher pops one or two words, latches the decoded controls
into `g_*` registers and jumps to the immediate, EA or execute state.
Simple register instructions dispatch out of `S_GEN_EXEC`, overlapping the
previous instruction's last clock. The register operands of the instruction
being dispatched are read through two dedicated register-file ports
(`rf_d`, `rf_e`) so that the decoder sits in no other path; the states that
follow use the latched `g_*` fields, and the EA engine takes its fields
from `ir`. The ALU's barrel shifter works from registered copies of its
operands (it is the deepest logic of the ALU), which costs shift and rotate
instructions one clock; CAS/CAS2 register their compare flags before
deciding on them for the same reason.

**Sequencer.** A state machine (`S_*` in `ap030_states.vh`) with one data
access at a time: `dreq` posts a request to the data port and parks the
sequencer in `S_DWAIT` with a destination (`dw_dst`) and a return state
(`dw_ret`). The EA engine (`S_EA*`) handles every 68020 addressing mode
including memory indirect with base and outer displacements.

**Undefined flags** follow the 68020/030 behaviour observed on silicon
(the WinUAE 68020 model): BCD N from the result and V from the correction,
DIVx overflow leaves V=1 N=1 Z=0 C=0, CHK sets Z from the operand, MULx.L
with equal registers keeps the high half, MOVEM -(An) stores the initial
An minus the size, MOVE An,-(An) stores the decremented value.

## Exceptions (ap030_exec_b.vh, ap030_frame.vh, ap030_tasks.vh)

Frames $0, $1 (throwaway, S set, on the interrupt stack when M is set),
$2, $9, $A and $B are pushed with the layout of UM 8.4. The SSW carries
FC/FB/RC/RB/DF/RM/RW/SIZE/FC as in Figure 8-9. The internal words of the
$A/$B frames hold this core's resume state:

| words | content                                                             |
|-------|---------------------------------------------------------------------|
| 4     | resume kind (boundary, stream, read, write, RMW) and state          |
| 6-7   | stage C and B images                                                |
| 10-11 | `ea`, 14-17 `src`/`dst`, 20-21 `imm`, 24-25 `tmp`, 28-29 `tmp2`     |
| 22-23 | data input buffer (software completion)                             |
| 26-27 | `ir`, version 0 with `dw_dst`/`dw_ret`                              |
| 32-36 | loop counters, MOVEM mask, EA/immediate return states               |
| 37-38 | the bytes already read of a misaligned operand (UM 8.2.1)           |
| 39-44 | extension word, trace and coprocessor dialogue state                |

The values are captured when the fault is recognised (not while the frame
is being pushed, which reuses the counters). RTE reloads them and, with
DF set, reruns the faulted cycle from the SSW, fault address and DOB; a
cleared DF delivers the DIB (reads) or drops the cycle (writes); an RMW
fault reruns the whole instruction with DF set and counts as emulated
with DF clear (UM 8.2.2). A version mismatch in word 27 is a format error
(UM 8.1.8). Misaligned operands whose later portion faults keep the
portions already transferred in words 37-38 and merge them on the rerun;
for writes the validated first portion is still written, as the real
part's bus would have done.

Interrupts: the IPL lines are synchronised, level 7 is edge sensitive
(a transition to 7, or the mask dropping below 7 with the request held),
IPEND reflects a pending interrupt, STOP wakes on a request above the
mask. Trace T1/T0 follow UM 8.1.7: T0 traces changes of flow, instruction
traps included.  The trace of an instruction trap (TRAP, TRAPcc/TRAPV,
cpTRAPcc, CHK/CHK2, divide by zero) or a coprocessor post-instruction
exception is taken after that exception's processing (UM 8.1.12), with
the trapping instruction's address in its frame (UM Table 8-6); this is
decided by the instruction, not the vector, so an interrupt or a
coprocessor exception with any vector gets no extra trace, and a
mid-instruction coprocessor exception is traced when the instruction
completes after its RTE.  With a trace pending, a general coprocessor
instruction's dialog runs until a null CA=0 primitive (UM 10.5.2.5).

Posted writes that fail on the bus are reported at the next instruction
boundary with a format $A frame (UM 8.1.2), and RTE reruns the write.  A
bus or address error first waits for the writes posted before it; if one
of them fails, that fault is taken right after as a bus error of its own,
its frame on top, so the handler sees the faults and the reruns happen in
program order.  Only a fault of a bus/address error's own frame write
halts the processor (double bus fault, UM 7.5.4).

Known difference: a write of the next instruction that was waiting for the
write buffer goes to the bus once the failed write has left it.  The
MC68030 instead begins exception processing at once and suspends that
instruction before its write (a format $B frame, UM 8.1.2 and Table 8-6).
When both writes fail for the same reason (one page, one address), the
MC68030 takes one bus error and this core two: a handler that repairs the
cause is entered a second time, finds nothing to do and its RTE reruns
the second write; memory ends the same and no write is lost or reordered.
A handler that counts bus errors sees one more.

## Memory subsystem (ap030_memsys.v)

The data unit looks up the data cache and the ATC in the same clock (the
"translation in parallel with the cache lookup" of UM 9.2). A read hit
needs no translation unless the ATC entry says the page is bad (UM 9.2.1);
everything else is validated by the MMU before it reaches the bus. Writes
update the data cache (write-through, write-allocate for aligned longwords
when WA is set, invalidation otherwise, UM 6.1.2.1) and enter a one-entry
write buffer that completes on the bus while the sequencer continues; a
write whose buffer slot and bus are free goes to the bus controller in the
clock it is posted, and the next one may post in the clock the previous
completes, so consecutive stores run four clocks apart on a synchronous
port. Read-modify-write and CPU-space writes are not posted so that their
bus errors are seen by the instruction. A data cache hit is acknowledged
in the clock its data is valid (two-clock read, UM 11.2). Operands crossing a longword, cache
line or page are split as UM 7.2.2 describes, and the first portion of a
line-crossing read is not burst.

Table searches wait for the write buffer to drain (a descriptor may just
have been written) and hold RMC for their duration.  A read-modify-write
operation (TAS, CAS, CAS2) holds RMC from its first transfer until its last
write, the CAS/CAS2 compare mismatch that ends it without a write, or a
fault, and no instruction prefetch runs in between (UM 7.3.3), so RMC is
negated before the next cycle (UM 7.1.1).

**System options** (inputs of `ap030_top`, tied to 0 for a plain MC68030):
`snoop_we`/`snoop_addr` invalidate the data cache entry for an address
another bus master has written. The MC68030 does not snoop; CIIN keeps
DMA-written memory out of the caches on reads, but CIIN is ignored on
writes, so with write allocation an aligned longword store still creates
an entry (UM 6.1.2), and a system whose DMA writes such memory can report
the writes here. `nmi_vec_nocache` makes the level 7 autovector fetch
bypass the data cache, so an external overlay of that vector (a freezer
cartridge) is always seen. The Minimig integration uses both.

**Caches** (`ap030_cache.v`, UM 6): 256 bytes each, direct mapped, 16
lines of four longword entries with individual valid bits, tagged by the
logical address and FC2 (instruction) or FC2-0 (data). Bursts fill the
missing entries of a line; on a narrow port the entry is completed from
the extra cycles (UM 6.1.3.1). CACR implements WA, DBE, CD, CED, FD, ED,
IBE, CI, CEI, FI, EI; CAAR selects the entry cleared by CED/CEI; CDIS
disables both caches. CIIN inhibits fills and is ignored on writes; CIOUT
(the CI bit of the ATC entry or TT register) inhibits the caches for the
access, which leaves an entry stale if a cached location is later written
with CIOUT asserted, exactly as UM 6.1 warns.

**MMU** (`ap030_mmu.v`, UM 9): a 22-entry fully associative ATC with the
V/FC/LA/B/CI/WP/M/PA entry of UM 9.4 and history-bit replacement, TT0/TT1
with base/mask, FC base/mask, R/W and RWM, and the table search engine of
UM 9.5: function code lookup, up to four index levels of any width, page
sizes 256 bytes to 32 KB, short and long descriptors, upper and lower
limits (on the root pointer, on long table descriptors and on long early
termination descriptors), early termination with contiguous mapping,
indirect descriptors, supervisor and write protection (RMC cycles count as
writes), U and M updates written back under RMC. Limit violations, invalid
descriptors, supervisor violations and bus errors during the search create
an entry with B set, so the access faults until the entry is flushed.
PTEST levels 0-7 set the MMUSR of Table 9-3 and return the address of the
last descriptor fetched completely; PLOAD, PFLUSHA, PFLUSH by FC and by
FC and address, PMOVE and PMOVEFD, and the configuration exception (vector
56, format $2, PC after the PMOVE) are implemented. A CpID 0 encoding the
MC68030 does not support (the 68851-only types and registers, reserved
bits of the UM 3.3.3 formats, a PC relative or immediate EA field) takes
the F-line exception in supervisor mode and a privilege violation in user
mode (UM 9.8). MMUDIS disables
translation; RESET clears the E bits and leaves the ATC alone.

## Bus controller (ap030_bus.v, UM 7)

Requests carry an address, byte count, total operand size (for SIZ),
direction, function code and the RMC, CIOUT, CBREQ and OCS attributes.
The controller runs S0-S5 with the timing of UM Figures 7-7 to 7-61:

- asynchronous cycles terminate on DSACKx (three clocks plus wait states),
  with the port size read from DSACK1/0 and the operand assembled or the
  write lanes driven per Tables 7-4 and 7-5;
- synchronous cycles terminate on STERM (two clocks), bursts of four
  longwords with modulo-4 address wrap continue while CBACK is asserted;
- ECS and OCS pulse for the first half clock, DBEN follows UM 7.3;
- BERR, BERR with HALT (retry), HALT (single cycle) and late BERR after
  DSACK are handled as UM 7.5; retry restarts the cycle from S0 after both
  negate;
- BR/BG/BGACK arbitration follows the state machine of Figure 7-61: the
  bus is granted between cycles (never inside an RMC sequence) and the
  outputs are released;
- CPU space cycles (FC=7) serve the interrupt acknowledge (vector on the
  low byte, AVEC, spurious on BERR), breakpoint acknowledge (BERR: illegal
  instruction; DSACK: the opcode replaces the BKPT) and coprocessor CIR
  accesses (type 2, CpID on A15-A13).

## Coprocessor interface (ap030_exec_c.vh, UM 10)

cpGEN, cpBcc, cpDBcc, cpScc, cpTRAPcc, cpSAVE and cpRESTORE talk to CpID
0-7 through the CIRs; CpID 0 is the on-chip MMU (PMOVE/PTEST/PLOAD/PFLUSH
decode from the same F-line space). Every response primitive of Table 10-2
is served: busy, null (with CA, IA, PF, TF), supervisor check, transfer
operation word, transfer from instruction stream, evaluate and transfer
effective address, evaluate effective address and transfer data (every EA
category check of Table 10-4), write to previously evaluated EA, take
address and transfer data, transfer to/from top of stack, transfer single
and multiple main processor registers, transfer main processor control
register, transfer multiple coprocessor registers, transfer status register
and scanPC, and the three take-exception primitives. Protocol violations,
F-line aborts (the control CIR written with $0001), format errors and the
absence of a coprocessor (BERR on the first CIR access) raise the
exceptions of UM 10.5.

## Verification

`tb/tb_ap030_bus.sv` drives the bus controller alone against slave models
of every termination (asynchronous 8/16/32-bit ports, synchronous with and
without bursts, CIIN, BERR, late BERR, retry, HALT, CBACK negated early)
and checks the data, the fills and the cycle lengths in half clocks.

`tb/tb_ap030_program.sv` wraps the whole processor with 1 MB of RAM
reachable through different ports (synchronous burst, 16-bit, 8-bit,
32-bit with CIIN, plain asynchronous), a bus error region, an interrupt
controller, the breakpoint and coprocessor acknowledge cycles and a
scripted coprocessor; the self-test programs in `tb/asm` run on it.
`+waits=n` adds wait states to every port, which reorders the interaction
between the write buffer, the walker and the prefetcher and is run for
every program.
