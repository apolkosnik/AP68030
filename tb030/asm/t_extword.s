; AP68030 exception priority of instructions that would take a second word
; assembled with vasmm68k_mot -Fbin -m68030 -m68851
;
; An illegal instruction, or a privileged one in user mode, is identified
; from its operation word (UM 8.1.5, 8.1.6, 9.8): when the prefetch of the
; word after it ends in a bus error, the illegal instruction or privilege
; violation exception is taken, not the bus error, because "if the aborted
; bus cycle is an instruction prefetch, the processor may delay taking the
; exception until it attempts to use the prefetched information" (UM 8.1.2)
; and these instructions never use it (WinUAE: op_illg reads no extension
; word; STOP and the MMU instructions check the privilege first).  An
; instruction that does use the word takes the bus error.
; Each opcode is the second word of a longword whose successor bus-errors
; (the bench's bus error trigger), with the caches off.
;
; protocol with the testbench:
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102
BERRREG	equ	$F00130		; long: bus error trigger address (0 = off)

; scratch variables
lastvec	equ	$3800		; word: vector number of the last exception
resume	equ	$3804		; long: the handlers return here
stk1	equ	$3400		; supervisor stack
USTK	equ	$3000		; user stack

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

; compare the word at \1 against \2, fail with number \3
chkw	macro
	moveq	#0,d7
	move.w	\1,d7
	cmp.l	#\2,d7
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; run the opcode \2 (and the word \3 after it, in the faulting longword) in
; supervisor mode, or user mode when \5 = user; expect vector \4; test \1
xw	macro
	lea	stk1,sp
	move.l	#r\@,resume
	clr.w	lastvec
	move.l	#w\@+2,(BERRREG).l
	ifc	"\5","user"
	move.w	#$0700,sr
	endif
	bra	w\@
	cnop	0,4
	nop
w\@:	dc.w	\2		; the longword after this one bus-errors
	dc.w	\3,$4E71,$4E71
r\@:	clr.l	(BERRREG).l
	chkw	lastvec,\4,\1
	endm

	org	0
	dc.l	stk1		; initial ISP
	dc.l	start
	dc.l	h_berr		; 2
	rept	253
	dc.l	h_any
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,cacr
	lea	USTK,a1
	move.l	a1,usp

	xw	1,$00C0,$0000,4		; CHK2.B with EA mode Dn: illegal
	xw	2,$4C08,$0000,4		; MULx.L with EA mode An: illegal
	xw	3,$0EC0,$0000,4		; CAS.L with EA mode Dn: illegal
	xw	4,$4E72,$2700,8,user	; STOP in user mode
	xw	5,$4E7A,$0801,8,user	; MOVEC VBR,D0 in user mode
	xw	6,$0E50,$9000,8,user	; MOVES.W (A0),A1 in user mode
	xw	7,$F010,$4200,8,user	; PMOVE TC,(A0) in user mode (UM 9.8)
	; an instruction that uses its second word takes the bus error
	xw	8,$4C00,$1000,2		; MULU.L D0,D1
	xw	9,$4E7A,$0801,2		; MOVEC VBR,D0 in supervisor mode

	move.w	#$600D,(DONEREG).l
done:	bra.s	done

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

;---------------------------------------------------------------- handlers
; bus error: the faulted stream cannot be resumed; record it and continue
; at resume on a fresh supervisor stack
h_berr:
	clr.l	(BERRREG).l
	move.w	#2,lastvec
	lea	stk1,sp
	move.w	#$2700,sr
	move.l	resume,-(sp)
	rts
; any other exception: record the vector, return to resume in supervisor
; mode with IPL 7 (formats $0 and $2 only occur here)
h_any:
	move.l	d6,-(sp)
	move.w	10(sp),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	move.l	resume,6(sp)
	move.w	#$2700,4(sp)
	move.l	(sp)+,d6
	rte
