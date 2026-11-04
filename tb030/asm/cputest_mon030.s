; AP68030 (070) cputest replay monitor, run by tb_cputest030.sv.
; assembled with vasmm68k_mot -Fbin -m68030 -m68851, loaded at $42110000
;
; The AP68060 monitor (tb/asm/cputest_mon.s) for the 68030: the same
; mailbox and protocol.  The bench writes a round's input state into the
; mailbox and sets GO; the monitor loads it (with the MSP) and enters the
; tested instruction through a format $0 RTE whose frame the bench placed
; at the round's ISP - 8.  Every exception vector points at a stub that
; records its vector number and saves the registers with absolute
; addressing only -- the corpus stack holds the frame under test -- then
; tells the bench (a bus write of EVT) and waits: resume through RTE (a
; trace stacked on a primary exception) or stop.
;
; The test's CACR, TC, TT0 and TT1 are saved at the capture and put back on
; a resume; the monitor itself runs with the caches and the PMMU off, so
; the mailbox traffic reaches the bus.  The 68030 corpus' FPU groups are
; not run (no FPU): NOFP is always set.

MBOX	equ	$42120000
GO	equ	MBOX+$000	; word: bench -> monitor, start a round
IN_D	equ	MBOX+$004	; D0-D7/A0-A6, 60 bytes
IN_USP	equ	MBOX+$040
IN_FRM	equ	MBOX+$044	; ISP for the entry RTE (the frame)
IN_MSP	equ	MBOX+$048
IN_VBR	equ	MBOX+$04C
EVT	equ	MBOX+$100	; word: monitor -> bench, an exception entry
EVT_VEC	equ	MBOX+$102	; word: its vector number
CAP_D	equ	MBOX+$104	; D0-D7/A0-A6 at the handler entry
CAP_USP	equ	MBOX+$140
CAP_SP	equ	MBOX+$144	; the active stack pointer: the frame
CAP_MSP	equ	MBOX+$148
CAP_CR	equ	MBOX+$14C	; CACR TC TT0 TT1 of the test
CMD	equ	MBOX+$200	; word: bench -> monitor, 1 resume, 2 stop
RDY	equ	MBOX+$204	; word: monitor -> bench, idle at mask 7
ZERO	equ	MBOX+$210	; a zero long word (PMOVE source)
MSTACK	equ	$42130000

	org	$42110000

; $42110000 + 16 * n: the handler of vector n
n	set	0
	rept	256
	move.w	#n,(EVT_VEC).l
	jmp	(capture).l
	nop
n	set	n+1
	endr

; $42111000
capture:
	movem.l	d0-d7/a0-a6,(CAP_D).l
	movec	cacr,d0
	move.l	d0,(CAP_CR).l
	pmove	tc,(CAP_CR+4).l
	pmove	tt0,(CAP_CR+8).l
	pmove	tt1,(CAP_CR+12).l
	move.l	#$00000808,d0		; the caches cleared and off
	movec	d0,cacr
	pmove	(ZERO).l,tc
	pmove	(ZERO).l,tt0
	pmove	(ZERO).l,tt1
	pflusha
	move.l	usp,a0
	move.l	a0,(CAP_USP).l
	move.l	a7,(CAP_SP).l
	movec	msp,a0
	move.l	a0,(CAP_MSP).l
	move.w	#1,(EVT).l
cwait:
	move.w	(CMD).l,d0
	beq.s	cwait
	clr.w	(CMD).l
	cmp.w	#1,d0
	bne.s	stop
	pmove	(CAP_CR+8).l,tt0
	pmove	(CAP_CR+12).l,tt1
	pmove	(CAP_CR+4).l,tc
	move.l	(CAP_CR).l,d0
	movec	d0,cacr
	movem.l	(CAP_D).l,d0-d7/a0-a6
	rte
stop:
	move.w	#$2700,sr
	lea	(MSTACK).l,sp
	bra	irdy

; the reset entry, at a fixed address (tb_cputest030.sv IDLE)
	ds.b	$42111800-*
idle:
	move.w	#$2700,sr
	lea	(MSTACK).l,sp
	clr.l	(ZERO).l
irdy:
	move.w	#1,(RDY).l
iwait:
	tst.w	(GO).l
	beq.s	iwait
	clr.w	(GO).l
	moveq	#0,d0
	movec	d0,cacr
	pmove	(ZERO).l,tc
	pmove	(ZERO).l,tt0
	pmove	(ZERO).l,tt1
	movec	d0,sfc
	movec	d0,dfc
	pflusha
	move.l	(IN_USP).l,a0
	move.l	a0,usp
	move.l	(IN_MSP).l,a0
	movec	a0,msp
	move.l	(IN_VBR).l,a0
	movec	a0,vbr
	movem.l	(IN_D).l,d0-d7/a0-a6
	move.l	(IN_FRM).l,a7
	rte
