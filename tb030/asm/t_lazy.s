; AP68030 fetch_lazy self test (UM 11.2.2 prefetch; ap030_top "fetch_lazy")
; assembled with vasmm68k_mot -Fbin -m68030; run with +lazy only
;
; The pipeline-model inputs of fetch_lazy are set by the program through the
; testbench registers $F1E0-$F1E9 (tb_ap030_program.sv).
;   1. a wrong stop, below the code still to run: the processor waits for
;      instruction words it needs and must not hang (immediate, absolute
;      long, two-word operation word, memory indirect extension words)
;   2. a scan that never advances: likewise
;   3. a correct stop on an RTS after MOVEM.L (SP)+ (many clocks taking no
;      words): no fetch past the RTS's longword before the RTS
;   4. a correct stop on an RTS whose stack read is slow (100 wait states)
;      while the queue is empty: no fetch past the RTS's longword either --
;      the guard is for a starved sequencer, not for a stop holding while an
;      instruction runs
;
; protocol with the testbench:
;   word write to $F00100 = failing test number
;   word write to $F00102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102
WAITREG	equ	$F00140		; word: wait states of the memory port
WATCH	equ	$F001C0		; long: watched address
WATCHFC	equ	$F001C4		; word: FC of its last bus read (0: none)
FMSTOP	equ	$F001E0		; long: fetch_stop
FMSCAN	equ	$F001E4		; long: fetch_scan_to
FMVALID	equ	$F001E8		; word: bit 0 fetch_stop_v, bit 1 fetch_scan_v

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

	org	0
	dc.l	$3800		; initial ISP
	dc.l	start		; initial PC
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	moveq	#0,d0
	movec	d0,cacr		; caches off: every instruction fetch is a bus cycle

;---------------------------------------------------------------- 1. wrong stop
	clr.l	FMSTOP		; every longword is at or past the stop
	move.w	#1,FMVALID
	bsr	words
	clr.w	FMVALID
	chkl	d0,$12345678,1
	chkl	d1,$47AE1479,2
	chkl	d2,$600DF00D,3

;---------------------------------------------------------------- 2. scan that never advances
	clr.l	FMSCAN		; nothing scanned
	move.w	#2,FMVALID
	bsr	words
	clr.w	FMVALID
	chkl	d0,$12345678,4
	chkl	d1,$47AE1479,5
	chkl	d2,$600DF00D,6

;---------------------------------------------------------------- 3. stop on an RTS after MOVEM
	move.l	#sub3r+4,WATCH	; the longword after the RTS's
	move.l	#sub3r,FMSTOP
	move.w	#1,FMVALID
	bsr	sub3
	clr.w	FMVALID
	move.w	WATCHFC,d0
	and.l	#7,d0
	chkl	d0,0,7

;---------------------------------------------------------------- 4. stop on an RTS with a slow stack read
	move.w	WAITREG,d6
	move.l	#sub4r+2,WATCH	; the longword after the RTS's
	move.l	#sub4r,FMSTOP
	move.w	#1,FMVALID
	move.w	#100,WAITREG
	bsr	sub4
	move.w	d6,WAITREG
	clr.w	FMVALID
	move.w	WATCHFC,d0
	and.l	#7,d0
	chkl	d0,0,8

	move.w	#$600D,(DONEREG).l
	stop	#$2700

fail_all:
	clr.w	FMVALID
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	clr.w	FMVALID
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2

;---------------------------------------------------------------- code for tests 1 and 2
words:
	moveq	#0,d0
	move.l	#$12345678,d0	; the immediate's two words
	move.l	(val).l,d1	; absolute long
	mulu.l	d0,d1		; operation word and second word
	add.l	val2(pc),d1	; displacement
	move.l	([vptr]),d2	; memory indirect: base displacement words
	rts
	cnop	0,4
val:	dc.l	3
val2:	dc.l	$11111111
vptr:	dc.l	vdat
vdat:	dc.l	$600DF00D

;---------------------------------------------------------------- subroutines for tests 3 and 4
	org	$1000
sub3:
	movem.l	d0-d7/a0-a6,-(sp)
	nop
	cnop	0,4
	movem.l	(sp)+,d0-d7/a0-a6
sub3r:	rts			; at a longword boundary
	dc.w	$4E71,$4E71,$4E71,$4E71

	org	$1100
sub4:	nop
sub4r:	rts			; the second word of its longword
	dc.w	$4E71,$4E71,$4E71,$4E71
