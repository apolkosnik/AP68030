; AP68030 coprocessor interface tests on the coprocessor model's script
; (tb_cp_model.svh, command $0030): common equates, macros, vectors and the
; exception handler.  Included by asm/t_cpx_*.s, which define "start" and end
; with "endtest".  vasmm68k_mot -Fbin -m68030; run_tests.sh finds them.
;
; The model answers CpID 1.  A program writes the response primitives to
; SCRIPT in turn (SCRCLR empties the list), the operand CIR value to SCROP and
; the operand address CIR value to SCRADDR, then runs a cpGEN with command
; $0030.  Every response read moves the script on; the last word repeats.
;
; The handler logs up to eight exceptions in the order their handlers run
; (vector, format, SR, PC, instruction address of formats $2/$9, EA field of
; format $9), clears the trace bits of the frame it returns through unless it
; is a mid-instruction frame (format $9: the suspended instruction resumes
; with its own trace state), releases the interrupt request of an autovector
; or of IRQVEC, and adds SKIP to the PC of a format $0 frame of anything but
; an interrupt.  A failed check prints "Fnn " (hex) on the console; the
; first failing number is reported at the end.

FAILREG	equ	$F00100
DONEREG	equ	$F00102
IPLREG	equ	$F00110		; word: interrupt request level
VECREG	equ	$F00112		; word: vector supplied by the next IACK (0 = autovector)
OPREG0	equ	$F00188		; long: first operand the model received
OPREG1	equ	$F0018C
CTRLREG	equ	$F00184		; word: last control CIR value
CONS	equ	$F00190
SCRIPT	equ	$F00194		; word: append a response; long read: CPU space type 2 cycles
SCRCLR	equ	$F00196
SCROP	equ	$F00198
SCRADDR	equ	$F0019C
IRQARM	equ	$F001BC
WATCH	equ	$F001C0		; long: watched address
WATCHFC	equ	$F001C4		; word: FC of its last bus read

exccnt	equ	$3000		; word: exceptions taken
skip	equ	$3004		; long
failcnt	equ	$3008		; word
firstf	equ	$300A		; word: first failing check
irqvec	equ	$300C		; word: a vector number the handler treats as an interrupt
logvec	equ	$3010		; 8 words
logfmt	equ	$3020		; 8 words
logsr	equ	$3030		; 8 words
logpc	equ	$3040		; 8 longs
logia	equ	$3060		; 8 longs
logea	equ	$3080		; 8 longs
buf	equ	$3100		; 64 bytes
buf2	equ	$3140		; 64 bytes

; check number \1 failed: report it and go on
failt	macro
	move.w	#\1,d7
	bsr	report
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

clrlog	macro
	clr.w	exccnt
	clr.l	logvec
	clr.l	logvec+4
	clr.l	logvec+8
	clr.l	logfmt
	clr.l	logfmt+4
	endm

; the script: up to four response words (\1..\4; omitted ones are not appended)
script	macro
	move.w	#0,SCRCLR
	move.w	#\1,SCRIPT
	ifnb	\2
	move.w	#\2,SCRIPT
	endif
	ifnb	\3
	move.w	#\3,SCRIPT
	endif
	ifnb	\4
	move.w	#\4,SCRIPT
	endif
	endm

endtest	macro
	tst.w	failcnt
	bne	fail_all
	move.w	#$600D,(DONEREG).l
	stop	#$2700
	endm

	org	0
	dc.l	$3800		; initial ISP
	dc.l	start
	rept	254
	dc.l	h_exc
	endr

	org	$400
h_exc:
	movem.l	d6-d7/a5/a6,-(sp)
	lea	16(sp),a6
	move.w	exccnt,d7
	and.w	#7,d7
	lsl.w	#1,d7			; word index
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	lea	logvec,a5
	move.w	d6,0(a5,d7.w)
	move.w	6(a6),d6
	lsr.w	#8,d6
	lsr.w	#4,d6
	lea	logfmt,a5
	move.w	d6,0(a5,d7.w)
	lea	logsr,a5
	move.w	(a6),0(a5,d7.w)
	lsl.w	#1,d7			; long index
	lea	logpc,a5
	move.l	2(a6),0(a5,d7.w)
	lea	logia,a5
	move.l	8(a6),0(a5,d7.w)
	lea	logea,a5
	move.l	$10(a6),0(a5,d7.w)
	addq.w	#1,exccnt
	cmp.w	#9,d6
	beq.s	h_keept
	and.w	#$3FFF,(a6)		; no more tracing after the handler
h_keept:
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	cmp.w	irqvec,d6
	beq.s	h_irq
	cmp.w	#24,d6
	blo.s	h_noirq
	cmp.w	#31,d6
	bhi.s	h_noirq
h_irq:	move.w	#0,IPLREG		; an interrupt: release the request
	bra.s	h_out
h_noirq:
	move.w	6(a6),d6
	and.w	#$F000,d6
	bne.s	h_out
	move.l	skip,d6
	add.l	d6,2(a6)
h_out:
	movem.l	(sp)+,d6-d7/a5/a6
	rte

report:
	movem.l	d0/a0,-(sp)
	tst.w	failcnt
	bne.s	rep1
	move.w	d7,firstf
rep1:	addq.w	#1,failcnt
	lea	hexdig(pc),a0
	move.b	#'F',CONS
	move.w	d7,d0
	lsr.w	#4,d0
	and.w	#$F,d0
	move.b	0(a0,d0.w),CONS
	move.w	d7,d0
	and.w	#$F,d0
	move.b	0(a0,d0.w),CONS
	move.b	#' ',CONS
	movem.l	(sp)+,d0/a0
	rts
hexdig:	dc.b	'0123456789ABCDEF'
	even

fail_all:
	move.w	firstf,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

; the common prologue: caches on, nothing logged
init:
	move.l	#$3111,d0
	movec	d0,cacr
	clr.l	skip
	clr.w	failcnt
	clr.w	firstf
	clr.w	irqvec
	rts
