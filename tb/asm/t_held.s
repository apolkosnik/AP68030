; AP68030 posted write fault self test
; assembled with vasmm68k_mot -Fbin -m68030
;
; Once a posted write fails, no later data access of the instruction in
; execution reaches the bus: the bus error is taken there, with a long frame
; ($B) whose PC is that instruction (UM 8.1.2; Table 8-6: "may not be the
; instruction that generated the faulted bus cycle") and whose DF, fault
; address and DOB describe the failed write (UM 8.2.1).  RTE reruns the
; write and the instruction goes on with the access it waited with: one bus
; error, a later access never issued before it.  When the instruction ends
; without another access, the fault is taken at the boundary ($A, PC =
; the next instruction).  The bus error handler repairs the trigger, so the
; RTE reruns the write.
;
; testbench registers:
;   $F00100 word  failing test number       $F00102 word  $600D / $BAD0
;   $F00118 word  BKPT replacement opcode (0 = BERR)
;   $F00120 byte  must be written with FC=1
;   $F00130 long  bus error trigger address (longword granularity, 0 = off)

FAILREG	equ	$F00100
DONEREG	equ	$F00102
BKPTREG	equ	$F00118
FCREG	equ	$F00120
BERRREG	equ	$F00130
WCI	equ	$400000		; RAM alias with CIIN: a data read always reaches the bus
W16	equ	$200000		; RAM alias on the 16-bit port (another logical address)

; scratch variables
lastvec	equ	$3800		; word: vector number of the last exception
lastfmt	equ	$3802		; word: format of the last exception frame
lastpc	equ	$3804		; long: stacked PC
lastsr	equ	$3808		; word: stacked SR
lastssw	equ	$3810		; word: SSW of a bus fault frame
lastfa	equ	$3814		; long: fault address
exccnt	equ	$3818		; word: exceptions taken
lastdob	equ	$381C		; long: data output buffer of a bus fault frame

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

chkw	macro
	move.w	\1,d6
	and.l	#$FFFF,d6
	cmp.l	#\2,d6
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; record the frame at a6
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

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	dc.l	h_buserr	; 2
	rept	5
	dc.l	unexp		; 3-7
	endr
	dc.l	h_priv		; 8
	rept	247
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$00003111,d0
	movec	d0,cacr
	move.w	#$2700,sr
	move.l	#$C0FFEE00,(W16+$3B40).l	; data for the held read
	move.l	#$00000000,(W16+$3B50).l	; TAS operand
	lea	(WCI+$3B10).l,a0		; the faulting longword

; arm: the next write to (a0) fails once (the handler repairs the trigger)
arm	macro
	clr.w	exccnt
	move.l	a0,BERRREG
	nop
	endm
; one bus error, a long frame, PC = the instruction at \1, its SSW (DF, RW,
; SIZE, FC masked with $01F7) = \2; failing numbers \3..\3+3 (uses D6/D7)
held	macro
	chkw	exccnt,1,\3
	chkw	lastfmt,$B,\3+1
	move.l	lastpc,d7
	chkl	d7,\1,\3+2
	move.w	lastssw,d7
	and.l	#$01F7,d7
	chkl	d7,\2,\3+3
	endm

;---------------------------------------------------------------- two instructions write the faulting longword
; the second waits behind the failed first one: it is in execution, the
; frame is the long one with its PC; the fault address and DOB are the
; first write's; the RTE reruns it, then the second write goes out
	arm
	move.l	#$12345678,(a0)
h1:	move.l	#$9ABCDEF0,(a0)
	nop
	held	h1,$0105,1		; DF, write, long, supervisor data
	move.l	lastfa,d1
	chkl	d1,WCI+$3B10,5
	move.l	lastdob,d1
	chkl	d1,$12345678,6
	move.l	(W16+$3B10).l,d1
	chkl	d1,$9ABCDEF0,7		; the later write last

;---------------------------------------------------------------- two writes of one instruction to the faulting longword
; the frame reports the first (UM 8.2.1); the second waits and completes
; after the RTE
	lea	(WCI+$3B20).l,a0
	move.l	#$1111AAAA,d1
	move.l	#$2222BBBB,d2
	arm
h2:	movem.w	d1-d2,(a0)
	nop
	held	h2,$0125,8		; DF, write, word, supervisor data
	move.l	lastfa,d1
	chkl	d1,WCI+$3B20,12
	move.l	lastdob,d1
	and.l	#$FFFF,d1		; right justified (UM 8.2.2)
	chkl	d1,$AAAA,13
	move.l	(W16+$3B20).l,d1
	chkl	d1,$AAAABBBB,14
	; MOVEP: bytes at +0 and +2 share the faulting longword
	lea	(WCI+$3B30).l,a0
	move.l	#$11223344,d0
	arm
h3:	movep.l	d0,0(a0)
	nop
	held	h3,$0115,15		; DF, write, byte, supervisor data
	move.l	lastfa,d1
	chkl	d1,WCI+$3B30,19		; the first byte
	move.l	(W16+$3B30).l,d1
	and.l	#$FF00FF00,d1
	chkl	d1,$11002200,20

;---------------------------------------------------------------- the held access, from various states
	lea	(WCI+$3B10).l,a0
	; a read with a postincrement
	lea	(WCI+$3B40).l,a1
	arm
	move.l	#$0BADCAFE,(a0)
h4:	move.l	(a1)+,d2
	nop
	held	h4,$0105,21
	chkl	d2,$C0FFEE00,25
	move.l	a1,d1
	chkl	d1,WCI+$3B44,26		; incremented once
	; a write with a predecrement
	lea	(WCI+$3B5C).l,a1
	arm
	move.l	#$0BADCAFE,(a0)
h5:	move.l	#$600DF00D,-(a1)
	nop
	held	h5,$0105,27
	move.l	a1,d1
	chkl	d1,WCI+$3B58,31		; decremented once
	move.l	(W16+$3B58).l,d1
	chkl	d1,$600DF00D,32
	; a read-modify-write (TAS): its locked read is issued after the RTE
	lea	(WCI+$3B50).l,a1
	arm
	move.l	#$0BADCAFE,(a0)
h6:	tas	(a1)
	nop
	held	h6,$0105,33
	move.l	(W16+$3B50).l,d1
	chkl	d1,$80000000,37
	; a coprocessor interface write (its F-line fault flag is kept)
	move.l	#$D3D3D3D3,d3
	moveq	#0,d4
	arm
	move.l	#$0BADCAFE,(a0)
h7:	dc.w	$F200,$0004	; coprocessor 1: D3 to the coprocessor
	dc.w	$F200,$0005	; operand CIR to D4
	held	h7,$0105,38
	chkl	d4,$D3D3D3D3,42
	; a breakpoint acknowledge (its illegal-instruction flag is kept)
	move.w	#$7007,BKPTREG	; MOVEQ #7,D0 replaces the breakpoint
	moveq	#0,d0
	arm
	move.l	#$0BADCAFE,(a0)
h8:	bkpt	#3
	held	h8,$0105,43
	chkl	d0,7,47
	move.w	#0,BKPTREG
	; a misaligned write (two portions)
	move.l	#$A1B2C3D4,d2
	arm
	move.l	#$0BADCAFE,(a0)
h9:	move.l	d2,(WCI+$3BC1).l
	nop
	held	h9,$0105,48
	move.l	(W16+$3BC0).l,d1
	and.l	#$00FFFFFF,d1
	chkl	d1,$00A1B2C3,52
	; a memory-indirect read
	move.l	#WCI+$3BD4,(W16+$3BD0).l
	move.l	#5,(W16+$3BD4).l
	moveq	#10,d1
	arm
	move.l	#$0BADCAFE,(a0)
h10:	add.l	([WCI+$3BD0]),d1
	held	h10,$0105,53
	chkl	d1,15,57
	; a MOVEM read
	move.l	#$11112222,(W16+$3BE0).l
	move.l	#$33334444,(W16+$3BE4).l
	arm
	move.l	#$0BADCAFE,(a0)
h11:	movem.l	(WCI+$3BE0).l,d4-d5
	held	h11,$0105,58
	chkl	d4,$11112222,62
	chkl	d5,$33334444,63
	; ADDX -(An),-(An) with X set (MOVE keeps X)
	move.l	#$00000001,(W16+$3BF0).l
	move.l	#$00000010,(W16+$3BF4).l
	lea	(WCI+$3BF4).l,a1
	lea	(WCI+$3BF8).l,a2
	arm
	ori.b	#$10,ccr
	move.l	#$0BADCAFE,(a0)
h12:	addx.l	-(a1),-(a2)	; $10 + $1 + X
	nop
	held	h12,$0105,64
	move.l	(W16+$3BF4).l,d1
	chkl	d1,$00000012,68
	; Scc to memory (MOVE cleared C: SCC is true)
	move.l	#0,(W16+$3C00).l
	arm
	move.l	#$0BADCAFE,(a0)
h13:	scc	(WCI+$3C00).l
	nop
	held	h13,$0105,69
	move.l	(W16+$3C00).l,d1
	chkl	d1,$FF000000,73

;---------------------------------------------------------------- user mode
; the held write keeps its function code (the bench checks that $F00120
; is written with FC=1)
	arm
	lea	($2C00).l,a1
	move.l	a1,usp
	move.w	#$0000,sr	; user mode
	move.l	#$0BADCAFE,(a0)
h14:	move.b	d0,(FCREG).l
	nop
	chkw	lastfmt,$B,76	; (user mode reads the records)
	move.l	lastpc,d1
	chkl	d1,h14,77
	rte			; privileged: back to supervisor
	move.w	#$2700,sr
	chkw	exccnt,2,74	; the bus error and the privilege violation
	move.w	lastssw,d1	; the bus error's SSW (still recorded)
	and.l	#$01F7,d1
	chkl	d1,$0101,75	; DF, write, long, user data
	move.l	#0,BERRREG

	move.w	#$600D,(DONEREG).l
done:	bra.s	done

;---------------------------------------------------------------- handlers
h_buserr:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	move.w	$A(a6),lastssw
	move.l	$10(a6),lastfa
	move.l	$18(a6),lastdob
	move.l	#0,BERRREG	; repaired: RTE reruns the write
	movem.l	(sp)+,d6/a6
	rte

h_priv:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	ori.w	#$2000,(a6)	; back to supervisor mode
	addq.l	#2,2(a6)
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
