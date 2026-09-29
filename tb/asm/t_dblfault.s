; AP68030 double bus fault self test (UM 8.1.2, 7.5.4)
; assembled with vasmm68k_mot -Fbin -m68030
;
; A bus error while the processor stacks a bus error frame halts it: the
; testbench runs this program with +expect_halt and passes when the HALT
; state is reached.  Before that, a bus error with the stack in good memory
; must be handled normally (the handler counts and repairs), so that the
; halt is known to come from the second fault and not from the first.

FAILREG	equ	$F00100
DONEREG	equ	$F00102
BERRREG	equ	$F00130
WCI	equ	$400000		; RAM alias with CIIN: reads always reach the bus

exccnt	equ	$3010

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	dc.l	h_buserr	; 2
	rept	253
	dc.l	h_unexp
	endr

	org	$400
start:
	move.l	#$3111,d0
	movec	d0,cacr
	clr.w	exccnt
	; a plain bus error is survivable
	move.l	#WCI+$5000,BERRREG
	move.l	(WCI+$5000).l,d1
	move.w	exccnt,d1
	cmp.w	#1,d1
	bne.s	fail1
	; the stack pointer moves into the bus error area: the frame push faults
	move.l	#WCI+$5010,BERRREG
	move.l	#$FF0100,sp
	move.l	(WCI+$5010).l,d1
	; not reached
	move.w	#2,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt0:	bra.s	halt0
fail1:
	move.w	#1,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

h_buserr:
	addq.w	#1,exccnt
	move.l	#0,BERRREG	; repaired: rerun
	rte

h_unexp:
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:	bra.s	halt2
