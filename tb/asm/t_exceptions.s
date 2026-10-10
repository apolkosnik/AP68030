; AP68030 exception processing self test
; assembled with vasmm68k_mot -Fbin -m68030
;
; Covers UM Section 8: instruction traps and their frames (formats $0 and
; $2), privilege violation, illegal/A-line/F-line, trace T1 and T0,
; autovectored, vectored and spurious interrupts with IPL masking and the
; level 7 transition, STOP, the master/interrupt stack switch with the
; throwaway frame, MOVEC and MOVES, RTE format errors, BKPT acknowledge,
; bus faults on data reads (rerun and software completion), posted writes
; and instruction fetches (formats $A/$B, SSW, RTE continuation), address
; errors, and the RESET instruction.
;
; testbench registers:
;   $F00100 word  failing test number       $F00102 word  $600D / $BAD0
;   $F00110 word  interrupt request level   $F00112 word  IACK vector (0 AVEC, $FF BERR)
;   $F00114 word  delay in clocks           $F00116 word  level applied after the delay
;   $F00118 word  BKPT replacement opcode (0 = BERR)
;   $F00120 byte  must be written with FC=1
;   $F00130 long  bus error trigger address (longword granularity, 0 = off)
;   $F00160 long  length of the last RESET instruction pulse
;   $F001FC long  bus cycles that overlapped a RESET instruction pulse

FAILREG	equ	$F00100
DONEREG	equ	$F00102
IPLREG	equ	$F00110
VECREG	equ	$F00112
DLYREG	equ	$F00114
DLVREG	equ	$F00116
BKPTREG	equ	$F00118
FCREG	equ	$F00120
BERRREG	equ	$F00130
WCI	equ	$400000		; RAM alias with CIIN: a data read always reaches the bus
W16	equ	$200000		; RAM alias on the 16-bit port (another logical address)
RSTLEN	equ	$F00160
RSTBUS	equ	$F001FC

; scratch variables
lastvec	equ	$3800		; word: vector number of the last exception
lastfmt	equ	$3802		; word: format of the last exception frame
lastpc	equ	$3804		; long: stacked PC
lastsr	equ	$3808		; word: stacked SR
lastia	equ	$380C		; long: instruction address (formats 2/9/A/B)
lastssw	equ	$3810		; word: SSW of a bus fault frame
lastfa	equ	$3814		; long: fault address
exccnt	equ	$3818		; word: exceptions taken
tracepc	equ	$381C		; long: last traced instruction address
mode	equ	$3820		; word: handler behaviour selector
irqhold	equ	$3822		; word: nonzero = the interrupt handler leaves the request asserted

; handler modes
M_RTE	equ	0		; plain RTE
M_SKIP2	equ	1		; add 2 to the stacked PC (skip the faulting word)
M_SUPER	equ	2		; set S in the stacked SR and skip 2
M_CLRT	equ	3		; clear T1/T0 in the stacked SR (trace stops)
M_FIXBERR equ	4		; clear the bus error trigger, RTE (rerun)
M_SWDATA equ	5		; software completion: DIB, clear DF
M_ADDRFIX equ	6		; address error: PC field += 2 (skip the jump)
M_FIXPC	equ	7		; set the PC field to fixpc
M_BERRDROP equ	8		; clear the trigger (rerun); a fault in the bus error
				; region $FFxxxx is completed in software (DF cleared)

fixpc	equ	$3824		; long

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

chkl	macro
	cmp.l	#\2,\1
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; record the frame at a6 (a register was saved below it)
record2	macro
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	move.w	6(a6),d6
	lsr.w	#8,d6
	lsr.w	#4,d6
	move.w	d6,lastfmt
	move.l	2(a6),lastpc
	move.w	(a6),lastsr
	addq.w	#1,exccnt
	endm

chkw	macro
	move.w	\1,d6
	and.l	#$FFFF,d6
	cmp.l	#\2,d6
	beq.s	ok\@
	failt	\3
ok\@:
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	dc.l	h_buserr	; 2
	dc.l	h_addrerr	; 3
	dc.l	h_illegal	; 4
	dc.l	h_generic	; 5 zero divide
	dc.l	h_generic	; 6 CHK
	dc.l	h_generic	; 7 TRAPcc
	dc.l	h_priv		; 8
	dc.l	h_trace		; 9
	dc.l	h_aline		; 10
	dc.l	h_fline		; 11
	dc.l	unexp		; 12
	dc.l	h_generic	; 13 coprocessor protocol
	dc.l	h_generic	; 14 format error
	dc.l	h_generic	; 15 uninitialized interrupt
	rept	8
	dc.l	unexp		; 16-23
	endr
	dc.l	h_irq		; 24 spurious
	dc.l	h_irq		; 25 level 1
	dc.l	h_irq		; 26
	dc.l	h_irq		; 27
	dc.l	h_irq		; 28
	dc.l	h_irq		; 29
	dc.l	h_irq		; 30
	dc.l	h_irq		; 31
	rept	16
	dc.l	h_generic	; 32-47 TRAP #n
	endr
	rept	16
	dc.l	h_generic	; 48-63
	endr
	dc.l	h_irq		; 64 vectored interrupt
	rept	191
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$00003111,d0
	movec	d0,cacr
	clr.w	mode
	clr.w	exccnt
	clr.w	irqhold

;---------------------------------------------------------------- TRAP #n: format $0, PC = next
	move.w	#$2700,sr
	trap	#0
t1:	chkw	lastvec,32,1
	chkw	lastfmt,0,2
	move.l	lastpc,d0
	chkl	d0,t1,3
	chkw	lastsr,$2700,4
	trap	#15
	chkw	lastvec,47,5

;---------------------------------------------------------------- ILLEGAL, A-line, F-line: PC = instruction
	move.w	#M_SKIP2,mode
i1:	illegal
	chkw	lastvec,4,6
	move.l	lastpc,d0
	chkl	d0,i1,7
	; static BTST #n,#imm is illegal (no immediate destination for a
	; static bit number; the dynamic form BTST Dn,#imm is legal)
i1b:	dc.w	$083C,$4E71,$4E71	; the handler skips the opcode, the two NOPs run
	chkw	lastvec,4,107
	move.l	lastpc,d0
	chkl	d0,i1b,108
i2:	dc.w	$A123
	chkw	lastvec,10,8
	move.l	lastpc,d0
	chkl	d0,i2,9
i3:	dc.w	$FFC0		; bits 8:6 = 111: F-line without coprocessor dialogue
	chkw	lastvec,11,10
	move.l	lastpc,d0
	chkl	d0,i3,11
	; cpGEN to an absent coprocessor: the CIR access ends in BERR -> F-line
	move.w	#M_SKIP2,mode
i4:	dc.w	$F400,$0000	; cpGEN CpID 2
	chkw	lastvec,11,12
	move.l	lastpc,d0
	chkl	d0,i4,13
	bra.s	i4done
	nop
i4done:

;---------------------------------------------------------------- CHK, DIVU by zero, TRAPV, TRAPcc: format $2
	move.w	#M_RTE,mode
	move.l	#5,d0
c1:	chk.w	#3,d0
c1n:	chkw	lastvec,6,14
	chkw	lastfmt,2,15
	move.l	lastpc,d0
	chkl	d0,c1n,16
	move.l	lastia,d0
	chkl	d0,c1,17
	moveq	#0,d1
	move.l	#10,d0
d1:	divu.w	d1,d0
d1n:	chkw	lastvec,5,18
	chkw	lastfmt,2,19
	move.l	lastia,d0
	chkl	d0,d1,20
	move.l	lastpc,d0
	chkl	d0,d1n,21
	move.w	#$02,ccr
v1:	trapv
v1n:	chkw	lastvec,7,22
	move.l	lastpc,d0
	chkl	d0,v1n,23
	move.w	#$04,ccr
v2:	trapeq.w	#$1234
v2n:	chkw	lastvec,7,24
	move.l	lastpc,d0
	chkl	d0,v2n,25
	move.l	lastia,d0
	chkl	d0,v2,26
	clr.w	lastvec
	move.w	#$00,ccr
	trapeq.l	#$12345678	; not taken, operand skipped
	chkw	lastvec,0,27
	trapv
	chkw	lastvec,0,28

;---------------------------------------------------------------- CHK2 trap and no trap
	move.w	#M_RTE,mode
	lea	bounds,a0
	move.l	#50,d2
	chk2.l	(a0),d2		; 10..40: out of bounds
	chkw	lastvec,6,29
	clr.w	lastvec
	move.l	#20,d2
	chk2.l	(a0),d2
	chkw	lastvec,0,30

;---------------------------------------------------------------- privilege violation from user mode
	move.w	#M_SUPER,mode
	move.l	a7,a5
	move.l	#$2000,a1
	move.l	a1,usp
	andi.w	#$DFFF,sr	; user mode
p1:	move.w	sr,d0		; privileged
	; back in supervisor mode (the handler set S and skipped)
	chkw	lastvec,8,31
	move.l	lastpc,d0
	chkl	d0,p1,32
	chkw	lastsr,$0700,33	; the stacked SR shows user mode
	cmpa.l	a5,a7
	beq.s	p1ok
	failt	34
p1ok:

;---------------------------------------------------------------- MOVEC: the MC68030 control registers
	moveq	#5,d0
	movec	d0,sfc
	moveq	#6,d0
	movec	d0,dfc
	movec	sfc,d1
	chkl	d1,5,35
	movec	dfc,d1
	chkl	d1,6,36
	move.l	#$3111,d0
	movec	d0,cacr
	movec	cacr,d1
	chkl	d1,$3111,37
	move.l	#$3919,d0	; with CI and CD (clear) bits: they read as zero
	movec	d0,cacr
	movec	cacr,d1
	chkl	d1,$3111,38
	move.l	#$12345670,d0
	movec	d0,caar
	movec	caar,d1
	chkl	d1,$12345670,39
	move.l	#$00001000,d0
	movec	d0,vbr
	movec	vbr,d1
	chkl	d1,$1000,40
	moveq	#0,d0
	movec	d0,vbr
	move.l	#$3000,d0
	movec	d0,msp
	movec	msp,d1
	chkl	d1,$3000,41
	movec	isp,d1
	cmpa.l	d1,a7
	beq.s	m1ok
	failt	42
m1ok:
	move.l	#$2100,d0
	movec	d0,usp
	move.l	usp,a2
	chkl	a2,$2100,43
	; an undefined control register (TC, a 68040 code) is illegal
	move.w	#M_SKIP2,mode
mc1:	dc.w	$4E7B,$0003
	chkw	lastvec,4,44
	move.l	lastpc,d0
	chkl	d0,mc1,45
	bra.s	mc1done
	nop
mc1done:

;---------------------------------------------------------------- MOVES with SFC/DFC
	moveq	#1,d0
	movec	d0,dfc		; user data
	move.b	#$5A,d1
	moves.b	d1,(FCREG).l	; the testbench checks FC=1 on this write
	moveq	#5,d0
	movec	d0,sfc
	move.l	#$C0DEC0DE,($3900).l
	moves.l	($3900).l,d2
	chkl	d2,$C0DEC0DE,46
	moves.w	($3902).l,a3	; sign extended into an address register
	chkl	a3,$FFFFC0DE,47
	moveq	#5,d0
	movec	d0,dfc

;---------------------------------------------------------------- trace T1: every instruction
	move.w	#M_RTE,mode
	clr.w	exccnt
	move.w	#$A700,sr	; T1
	nop
	moveq	#1,d0
tr_last:
	move.w	#$2700,sr	; the SR write itself is traced, then off
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,3,48
	chkw	lastvec,9,49
	chkw	lastfmt,2,50
	; the traced instruction's address is in the frame
	move.l	tracepc,d0
	chkl	d0,tr_last,51

;---------------------------------------------------------------- trace T0: change of flow only
	clr.w	exccnt
	move.w	#$6700,sr	; T0
	nop
	moveq	#2,d0
	addq.l	#1,d0
	bra.s	t0_taken	; taken branch: traced
	nop
t0_taken:
	nop
	move.w	#$2700,sr	; SR change: traced (upper bits change)
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,2,52

;---------------------------------------------------------------- TRAP under T1: trap, then the trace of the TRAP,
; then the trace of the SR write (UM 8.1.12)
	clr.w	exccnt
	move.w	#$A700,sr
	trap	#1
	move.w	#$2700,sr
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,3,53

;---------------------------------------------------------------- interrupts: masking, autovector, vectored, spurious
	clr.w	exccnt
	move.w	#0,VECREG
	move.w	#3,IPLREG	; request level 3 while masked at 7
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,0,54
	move.w	#$2200,sr	; mask 2: taken
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,55
	chkw	lastvec,27,56	; autovector level 3
	chkw	lastfmt,0,57
	chkw	lastsr,$2200,58	; the interrupted SR
	move.w	sr,d1
	and.w	#$FF00,d1
	chkw	d1,$2200,59	; back to the old mask after RTE
	; the handler released the request; a second one at level 2 is masked
	move.w	#2,IPLREG
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,60
	move.w	#0,IPLREG
	; vectored interrupt
	move.w	#$40,VECREG
	move.w	#5,IPLREG
	nop
	nop
	chkw	lastvec,64,61
	; spurious: BERR during the acknowledge
	move.w	#$FF,VECREG
	move.w	#4,IPLREG
	nop
	nop
	chkw	lastvec,24,62
	move.w	#0,VECREG
	; level 7 is edge sensitive: a second request needs a transition
	; (the handler keeps the request asserted during this block)
	clr.w	exccnt
	move.w	#1,irqhold
	move.w	#$2700,sr
	move.w	#7,IPLREG	; 0 -> 7 while masked at 7: taken
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,63
	chkw	lastvec,31,64
	move.w	#7,IPLREG	; still level 7 after the RTE: no new interrupt
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,65
	move.w	#$2600,sr	; lowering the mask below 7 with the request held: taken
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,2,66
	move.w	#$2700,sr
	move.w	#0,IPLREG
	nop
	move.w	#7,IPLREG	; transition: taken again
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,3,67
	move.w	#0,IPLREG
	clr.w	irqhold

;---------------------------------------------------------------- STOP: wakes on an interrupt
	clr.w	exccnt
	move.w	#40,DLYREG
	move.w	#4,DLVREG	; level 4 in 40 clocks
	stop	#$2300		; mask 3: level 4 wakes it
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,68
	chkw	lastvec,28,69
	move.w	#$2700,sr

;---------------------------------------------------------------- master stack: throwaway frame
	clr.w	exccnt
	move.l	#$3300,d0
	movec	d0,msp
	move.w	#$3000,sr	; S, M, mask 0
	cmpa.l	#$3300,a7	; the master stack is active
	beq.s	ms1ok
	failt	70
ms1ok:
	move.w	#6,IPLREG
	nop
	nop
	move.w	#0,IPLREG
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,1,71
	chkw	lastfmt,1,72	; the handler saw the throwaway frame on the ISP
	chkw	lastsr,$3000,73	; ... with the master-mode SR (S set, M set)
	move.w	sr,d1
	and.w	#$FF00,d1
	chkw	d1,$3000,74	; M restored by the two RTEs
	cmpa.l	#$3300,a7
	beq.s	ms2ok
	failt	75
ms2ok:
	move.w	#$2700,sr

;---------------------------------------------------------------- RTE format error
	move.w	#M_SKIP2,mode
	; build a frame with an illegal format ($5) below the stack and RTE it
	move.l	a7,a5
	move.w	#$5000,-(a7)	; format 5
	move.l	#rte_ok,-(a7)
	move.w	#$2700,-(a7)
rte_bad:
	rte
	; the format error handler (vector 14) skipped the RTE; a five-word...
	chkw	lastvec,14,76
	move.l	lastpc,d0
	chkl	d0,rte_bad,77
	move.l	a5,a7
	bra.s	rte_ok2
rte_ok:
	failt	78
rte_ok2:
	; the MC68030 clears T1/T0 before the format error: the frame's SR has
	; no trace bits, so the handler's RTE does not resume tracing
	move.w	#M_SKIP2,mode
	move.l	a7,a5
	move.w	#$5000,-(a7)
	move.l	#rte_ok,-(a7)
	move.w	#$2700,-(a7)
	move.w	#$6700,sr	; T0: the RTE is a change of flow
	rte
	chkw	lastsr,$2700,126
	andi.w	#$3FFF,sr
	chkw	lastvec,14,127
	move.l	a5,a7
	; a good six-word frame returns to its PC
	move.w	#M_RTE,mode
	move.l	a7,a5
	pea	rte6
	move.w	#$2000+14*4,-(a7)	; format 2, vector 14 (any)
	move.l	#rte6_target,-(a7)
	move.w	#$2704,-(a7)	; Z set on return
	rte
rte6:	failt	79
rte6_target:
	move.w	ccr,d1
	chkw	d1,$04,80
	move.l	a5,a7

;---------------------------------------------------------------- BKPT: acknowledge cycle
	move.w	#M_SKIP2,mode
	move.w	#0,BKPTREG	; BERR: illegal instruction
	moveq	#0,d0
bk1:	bkpt	#2
	chkw	lastvec,4,81
	move.l	lastpc,d0
	chkl	d0,bk1,82
	move.w	#$7007,BKPTREG	; moveq #7,d0 replaces the breakpoint
	moveq	#0,d0
	bkpt	#3
	chkl	d0,7,83
	move.w	#0,BKPTREG

;---------------------------------------------------------------- bus error on a data read: rerun by RTE
; The faulting reads go through the CIIN window so that they always reach the
; bus; CIIN is ignored on writes (a write-allocation would satisfy the read
; from the cache), so the data is prepared through another alias.
	move.w	#M_FIXBERR,mode
	move.l	#$0BADF00D,(W16+$3A00).l
	move.l	#WCI+$3A00,BERRREG
	move.l	(WCI+$3A00).l,d0
	chkl	d0,$0BADF00D,84
	chkw	lastvec,2,85
	chkw	lastfmt,$B,86
	move.w	lastssw,d1
	and.l	#$01C0,d1	; DF set, RM clear, RW set
	chkl	d1,$0140,87
	move.l	lastfa,d1
	chkl	d1,WCI+$3A00,88
	; a misaligned read faulting on its second portion reports that address
	move.l	#$11223344,(W16+$3A02).l
	nop
	move.l	#WCI+$3A04,BERRREG
	move.l	(WCI+$3A02).l,d0
	chkl	d0,$11223344,89
	move.l	lastfa,d1
	chkl	d1,WCI+$3A04,90

	; the rerun completes a split operand read into a register: the whole
	; longword is delivered, not only the rerun portion (MOVEM.L loads
	; sign-extend by the operand size)
	move.l	#$8123C456,(W16+$3A62).l
	move.l	#$9ABCDEF0,(W16+$3A66).l
	move.l	#WCI+$3A64,BERRREG
	movem.l	(WCI+$3A62).l,d2-d3
	chkl	d2,$8123C456,109
	chkl	d3,$9ABCDEF0,110
	; a memory-indirect operand whose pointer read faults resumes in the EA
	; calculation and still reads and adds the operand
	move.l	#$3A50,(W16+$3A40).l	; the pointer
	move.l	#5,(W16+$3A50).l	; the operand
	move.l	#WCI+$3A40,BERRREG
	moveq	#10,d1
	add.l	([WCI+$3A40]),d1
	chkl	d1,15,111

;---------------------------------------------------------------- software completion: the handler supplies the data
	move.w	#M_SWDATA,mode
	move.l	#WCI+$3A10,BERRREG
	move.l	(WCI+$3A10).l,d0
	chkl	d0,$CAFEBABE,91
	move.w	#M_FIXBERR,mode

;---------------------------------------------------------------- bus error on a posted write: format $A, rerun
	move.l	#WCI+$3A20,BERRREG
	move.l	#$DEADBEEF,d0
	move.l	d0,(WCI+$3A20).l
	nop			; the fault is taken at a boundary
	nop
	chkw	lastvec,2,92
	chkw	lastfmt,$A,93
	move.w	lastssw,d1
	and.l	#$01C0,d1	; DF set, RW clear
	chkl	d1,$0100,94
	move.l	lastfa,d1
	chkl	d1,WCI+$3A20,95
	move.l	(W16+$3A20).l,d1	; the rerun completed the write (memory, not the
	chkl	d1,$DEADBEEF,96		; entry the write allocated)

;---------------------------------------------------------------- two posted writes fault: no double bus fault
; A posted write faults while the next write is already waiting for the
; write buffer.  The second write's fault belongs to its instruction, not to
; the bus error frame being stacked, so it is no double bus fault (UM 7.5.4,
; 8.1.2).  The second write goes to the bus error region, so it faults on the
; MC68030 as well: exactly two bus errors, the first write's handled first
; (program order).  The handler repairs the trigger (RTE reruns that write)
; and completes the fault in the bus error region in software (DF cleared).
; KNOWN DEVIATION (not tested here): had the second write gone to the
; repaired longword, the MC68030 would report one bus error - it begins
; exception processing immediately after the faulted data cycle (UM 8.1.2),
; suspending the next instruction before its write reaches the bus (UM
; Table 8-6: the format $B PC "may not be the instruction that generated the
; faulted bus cycle").  This core has already handed that write to the bus;
; it faults too and is reported as a second bus error, in program order.
	move.w	#M_BERRDROP,mode
	clr.w	exccnt
	move.l	#WCI+$3AB0,BERRREG
	lea	(WCI+$3AB0).l,a0
	move.l	#$12345678,(a0)
	move.l	#$9ABCDEF0,($FF0000).l
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,2,135
	move.l	lastfa,d1	; the second write's frame is handled last
	chkl	d1,$FF0000,136
	move.w	lastssw,d1
	and.l	#$01C0,d1	; a data write: DF set, RW clear
	chkl	d1,$0100,137
	; the same with both writes in one instruction
	clr.w	exccnt
	move.l	#$FEFFFC,BERRREG
	lea	($FEFFFC).l,a0
	movem.l	d1-d2,(a0)	; $FEFFFC (trigger), then $FF0000 (bus error region)
	nop
	nop
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,2,138
	move.l	lastfa,d1
	chkl	d1,$FF0000,139
	move.w	#M_FIXBERR,mode

;---------------------------------------------------------------- bus error on an instruction fetch
	move.l	#fetchtarget,BERRREG
	jsr	fetchtarget
	chkl	d0,$F00D,97
	chkw	lastvec,2,98
	chkw	lastfmt,$B,99
	move.w	lastssw,d1
	and.l	#$A000,d1	; FC and RC: stage C faulted and is rerun
	chkl	d1,$A000,100

;---------------------------------------------------------------- address error: odd jump target
; the frame's PC is the JMP + 2 (MC68030 as modelled by WinUAE, checked by
; the v24 cputest corpus), so the handler resumes there unchanged
	move.w	#M_RTE,mode
	lea	oddtarget+1,a0
ae1:	jmp	(a0)
ae1n:	chkw	lastvec,3,101
	chkw	lastfmt,$B,102
	move.l	lastpc,d0
	chkl	d0,ae1n,103
	move.w	lastssw,d1
	and.l	#$F000,d1	; RC and RB set, no fault bits
	chkl	d1,$3000,104

;---------------------------------------------------------------- checked against the WinUAE 68030 corpus
; CHK and CHK2 trap frames hold the flags the instruction set
	move.w	#M_RTE,mode
	moveq	#-5,d0
	moveq	#10,d1
	move.w	#$00,ccr
	chk.l	d1,d0		; Dn < 0: N=1, C=1 (bound >= 0), V=0, Z=0
	chkw	lastvec,6,112
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$09,113
	lea	bounds,a0
	move.l	#50,d2
	move.w	#$00,ccr
	chk2.l	(a0),d2		; above 10..40: N=1, C=1
	chkw	lastvec,6,114
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$09,115

; divide by zero: the MC68030 flags (V set for DIVU, Z set for DIVS)
	move.l	#$80000000,d0
	moveq	#0,d1
	move.w	#$00,ccr
	divu.w	d1,d0		; N from dividend bit 31, Z from its high word, V=1
	chkw	lastvec,5,116
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$0A,117
	move.w	#$1B,ccr
	divs.w	d1,d0		; Z=1, N=V=C=0, X kept
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$14,118
	move.w	#$02,ccr
	divs.l	d1,d0		; Z=1, N=C=0, V not changed
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$06,119

; DBcc with an odd displacement faults even when the count expires: the
; MC68030 prefetches from the branch target whenever the condition is false
	move.w	#M_FIXPC,mode
	move.l	#dbo_n,fixpc
	clr.w	lastvec
	moveq	#0,d0
dbo:	dc.w	$51C8,$0003	; DBF D0,*+5
dbo_n:	chkw	lastvec,3,120
	chkl	d0,$0000FFFF,121

; RTE to an odd PC: the address error frame holds the SR from the RTE frame
	move.l	#rteo_n,fixpc
	move.w	#$2700,sr
	move.w	#$0000,-(sp)	; format $0
	pea	(oddtarget+1).l
	move.w	#$2715,-(sp)
	rte
rteo_n:	chkw	lastvec,3,122
	chkw	lastsr,$2715,123

; an exception taken at an overlapped dispatch stacks the flags of the
; instruction that just completed
	move.w	#M_SKIP2,mode
	moveq	#0,d0
	move.w	#$00,ccr
	ori.b	#$80,d0
	illegal
	chkw	lastvec,4,124
	move.w	lastsr,d6
	and.l	#$1F,d6
	chkl	d6,$08,125

;---------------------------------------------------------------- checked against the WinUAE 68030 corpus (v24)
; T0 traces ORI/ANDI/EORI to CCR and MOVE to SR even when nothing above the
; CCR changes; MOVE to CCR is not traced
	move.w	#M_RTE,mode
	clr.w	exccnt
	move.w	#$6700,sr	; T0
	ori.b	#$00,ccr	; traced
	move.w	#$04,ccr	; MOVE to CCR: not traced
	move.w	#$6700,sr	; MOVE to SR, no change above the CCR: traced
	move.w	#$2700,sr	; traced (clears T0)
	move.w	exccnt,d1
	and.l	#$FFFF,d1
	chkl	d1,3,128

; odd branch targets: the address error frame's PC (the frame is resumed
; at fixpc by the handler)
	move.w	#M_FIXPC,mode
	lea	oddtarget+1,a0
	move.l	#jsro_n,fixpc
	move.l	sp,a5
jsro:	jsr	(a0)		; JSR: the target
jsro_n:	move.l	a5,sp
	move.l	lastpc,d0
	chkl	d0,oddtarget+1,129
	move.l	#jmpx_n,fixpc
	moveq	#0,d1
jmpx:	jmp	0(a0,d1.w)	; JMP (d8,An,Xn): the end of the JMP + 2
jmpx_n:	move.l	lastpc,d0
	chkl	d0,jmpx_n+2,130
	move.l	#rtro_n,fixpc
	move.l	sp,a5
	pea	(oddtarget+1).l
	move.w	#$0000,-(sp)
rtro:	rtr			; RTR: the RTR + 2
rtro_n:	move.l	a5,sp
	move.l	lastpc,d0
	chkl	d0,rtro+2,131
	move.l	#dbo2_n,fixpc
	moveq	#1,d0
dbo2:	dc.w	$51C8,$0003	; DBF D0,*+5 taken: the target
dbo2_n:	move.l	lastpc,d0
	chkl	d0,dbo2+5,132

; an odd exception vector: the address error frame's PC is the vector
; offset of the exception being processed
	moveq	#0,d0
vcopy:	move.l	d0,a2
	move.l	(a2),($2000,a2)
	addq.l	#4,d0
	cmp.l	#$400,d0
	bne.s	vcopy
	move.l	#$00000123,($2010).l	; vector 4 (illegal): odd
	move.l	#ovec_n,fixpc
	move.l	#$2000,d0
	movec	d0,vbr
	illegal
ovec_n:	moveq	#0,d0
	movec	d0,vbr
	chkw	lastvec,3,133
	move.l	lastpc,d0
	chkl	d0,$10,134

;---------------------------------------------------------------- interrupts into user-mode stack code
; a level 3 request is swept over every clock of a user-mode loop that
; pushes, pops, calls and links on A7 (as tasks do under Kickstart); the
; interrupt runs on the ISP, and after each round USP, ISP and the
; loop's results must be exact.  The loop returns to supervisor mode
; through the privilege violation of an RTE.
	move.w	#M_RTE,mode
	move.w	#0,VECREG
	move.l	#1,d7			; delay in clocks
irs_round:
	move.w	#$2700,sr
	move.w	#0,IPLREG
	move.l	sp,a5			; ISP before the round
	lea	($2800).l,a0
	move.l	a0,usp
	clr.w	exccnt
	move.w	#3,DLVREG
	move.w	d7,DLYREG
	move.w	#$0000,sr		; user mode, mask 0
	moveq	#0,d0
	moveq	#7,d2
irs_loop:
	move.l	d0,-(sp)
	addq.l	#1,d0
	move.w	d0,-(sp)
	move.w	(sp)+,d1
	move.l	(sp)+,d3
	bsr	irs_sub
	link	a6,#-8
	move.l	d0,-4(a6)
	unlk	a6
	lea	-12(sp),sp
	lea	12(sp),sp
	dbf	d2,irs_loop
	rte				; privileged: back to supervisor
	move.w	#$2700,sr
	cmpa.l	a5,sp
	beq.s	irs_ok1
	move.w	#135,d6
	bra	irs_fail
irs_ok1:
	move.l	usp,a0
	cmpa.l	#$2800,a0
	beq.s	irs_ok2
	move.w	#136,d6
	bra	irs_fail
irs_ok2:
	cmp.l	#8,d0
	bne.s	irs_bad
	cmp.l	#8,d1
	bne.s	irs_bad
	cmp.l	#7,d3
	bne.s	irs_bad
	cmp.l	#8,d4
	beq.s	irs_ok3
irs_bad:
	move.w	#137,d6
	bra	irs_fail
irs_ok3:
	addq.l	#1,d7
	cmp.l	#400,d7
	bne	irs_round
	move.w	#0,IPLREG
	bra.s	irs_done
irs_sub:
	move.l	d0,d4			; d0 after the increment
	rts
irs_fail:
	move.w	d6,d7
	bra	fail_all
irs_done:

;---------------------------------------------------------------- RESET instruction: 512 clocks on the pin
	reset
	move.l	RSTLEN,d0
	chkl	d0,512,105
	movec	cacr,d1		; internal state untouched
	chkl	d1,$3111,106

	jmp	(resetbus).l		; continued at the end: the code below keeps its addresses
	nop
	nop
	nop

;---------------------------------------------------------------- data
bounds:	dc.l	10,40
oddtarget:
	nop
	rts
fetchtarget:
	move.l	#$F00D,d0
	rts

;---------------------------------------------------------------- handlers
; frame: 0(sp) SR, 2(sp) PC, 6(sp) format/vector, 8(sp) instruction address (2/9/A/B)
h_generic:
	movem.l	d6,-(sp)
	move.l	sp,a6
	addq.l	#4,a6		; a6 -> frame
	record2
	cmp.w	#2,lastfmt
	bne.s	hg1
	move.l	8(a6),lastia
hg1:
	move.w	mode,d6
	cmp.w	#M_SKIP2,d6
	bne.s	hg2
	addq.l	#2,2(a6)
hg2:
	movem.l	(sp)+,d6
	rte

h_illegal:
h_aline:
h_fline:
	movem.l	d6,-(sp)
	move.l	sp,a6
	addq.l	#4,a6
	record2
	move.w	mode,d6
	cmp.w	#M_SKIP2,d6
	bne.s	hi1
	addq.l	#2,2(a6)
	; a cpGEN (F-line with bits 8:6 = 000) or MOVEC has an extension word: skip it too
	move.l	lastpc,a6
	move.w	(a6),d6
	cmp.w	#$4E7B,d6
	beq.s	hi2
	and.w	#$F1C0,d6
	and.w	#$F000,d6
	cmp.w	#$F000,d6
	bne.s	hi1
	move.w	(a6),d6
	and.w	#$01C0,d6
	bne.s	hi1
hi2:
	move.l	sp,a6
	addq.l	#4,a6
	addq.l	#2,2(a6)
hi1:
	movem.l	(sp)+,d6
	rte

h_priv:
	movem.l	d6,-(sp)
	move.l	sp,a6
	addq.l	#4,a6
	record2
	ori.w	#$2000,(a6)	; back to supervisor mode
	addq.l	#2,2(a6)
	movem.l	(sp)+,d6
	rte

h_trace:
	movem.l	d6,-(sp)
	move.l	sp,a6
	addq.l	#4,a6
	record2
	move.l	8(a6),tracepc
	move.w	mode,d6
	cmp.w	#M_CLRT,d6
	bne.s	ht1
	andi.w	#$3FFF,(a6)
ht1:
	movem.l	(sp)+,d6
	rte

h_irq:
	movem.l	d6/a6,-(sp)	; the interrupted code's A6 is preserved
	move.l	sp,a6
	addq.l	#8,a6
	record2
	tst.w	irqhold
	beq.s	h_irq_rel
	or.w	#$0700,(a6)	; request held: return with the mask at 7
	bra.s	h_irq_out
h_irq_rel:
	move.w	#0,IPLREG	; release the request
h_irq_out:
	movem.l	(sp)+,d6/a6
	rte

h_buserr:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	move.w	$A(a6),lastssw
	move.l	$10(a6),lastfa
	move.w	mode,d6
	cmp.w	#M_FIXBERR,d6
	bne.s	hb1
	move.l	#0,BERRREG	; repaired: RTE reruns the cycle
	bra.s	hbx
hb1:
	cmp.w	#M_SWDATA,d6
	bne.s	hb2
	move.l	#0,BERRREG
	move.l	#$CAFEBABE,$2C(a6)	; data input buffer
	andi.w	#$FEFF,$A(a6)	; DF cleared: the read is not rerun
hb2:
	cmp.w	#M_BERRDROP,d6
	bne.s	hbx
	move.l	#0,BERRREG
	cmp.b	#$FF,$11(a6)	; fault address bits 23-16
	bne.s	hbx
	andi.w	#$FEFF,$A(a6)	; DF cleared: the write is not rerun
hbx:
	movem.l	(sp)+,d6/a6
	rte

h_addrerr:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	move.w	$A(a6),lastssw
	move.l	$10(a6),lastfa
	move.w	mode,d6
	cmp.w	#M_ADDRFIX,d6
	bne.s	ha0
	addq.l	#2,2(a6)	; skip the JMP (An); its pipe images are refetched
ha0:
	cmp.w	#M_FIXPC,d6
	bne.s	ha1
	move.l	fixpc,2(a6)	; resume at fixpc (the pipe is refetched)
ha1:
	movem.l	(sp)+,d6/a6
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2

;---------------------------------------------------------------- RESET instruction: the bus is idle for the pulse
; UM 7.8, Figure 7-65: a posted write still under way and the fetches of a
; RESET run from the 16-bit window complete before the pin is driven, and no
; bus cycle starts until the pulse is over
resetbus:
	move.l	RSTBUS,d5
	move.l	#$5A5AA5A5,(W16+$3B00).l	; posted: two cycles on the 16-bit port
	reset
	move.l	RSTBUS,d0
	sub.l	d5,d0
	chkl	d0,0,140
	move.l	(WCI+$3B00).l,d0		; the write reached memory
	chkl	d0,$5A5AA5A5,141
	jsr	W16+resetsub
	move.l	RSTBUS,d0
	sub.l	d5,d0
	chkl	d0,0,142
	move.l	RSTLEN,d0
	chkl	d0,512,143
	move.w	#$600D,(DONEREG).l
	stop	#$2700
resetsub:			; run from the 16-bit window
	reset
	rts
