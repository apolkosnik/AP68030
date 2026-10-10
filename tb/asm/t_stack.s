; AP68030 active stack pointer self test
; assembled with vasmm68k_mot -Fbin -m68030
;
; In supervisor mode A7 is the ISP (M=0) or the MSP (M=1) (UM Section 1,
; supervisor programming model).  A value written to A7 must be the stack
; pointer of whatever follows at once:
;  - an exception taken at the dispatch of the next instruction (illegal,
;    A-line, F-line) stacks its frame below the new A7 (UM 8.1), and the
;    write survives the exception (A7 after the RTE);
;  - RTS/RTD/RTR/RTE that pop the stack and then take an address error on
;    an odd target stack the format $B frame below the popped A7 (WinUAE
;    gencpu i_RTS/i_RTD/i_RTR/i_RTE at 68030 level: the pop is kept);
;  - MOVEC ISP/MSP reads the A7 just written, and an instruction reading A7
;    right after MOVEC to the active ISP/MSP sees the MOVEC value (PRM MOVEC).
; The instruction pairs are placed so that the second one is usually
; already in the pipe (a DIVU before them lets the prefetch catch up);
; the results must not depend on that.
;
; protocol with the testbench:
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102

; scratch variables
lastvec	equ	$3800		; word: vector number of the last exception
hsp	equ	$3804		; long: A7 at the handler's entry (the frame address)
resume	equ	$3808		; long: the handler returns here

stk1	equ	$3400
stk2	equ	$3000
stk3	equ	$2C00
oddt	equ	$1001
FRAME0	equ	8		; format $0 frame
FRAMEB	equ	92		; format $B frame (address error)

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

; compare the long \1 against the immediate \2, fail with number \3
chkl	macro
	move.l	\1,d7
	cmp.l	#\2,d7
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; the next exception returns to \1 in supervisor mode, IPL 7
arm	macro
	move.l	#\1,resume
	clr.w	lastvec
	endm

	org	0
	dc.l	stk1		; initial ISP
	dc.l	start
	rept	254
	dc.l	h_any
	endr

	org	$400
start:
	move.w	#$2700,sr

;---------------------------------------------------------------- A7 write, then an exception at dispatch
	; MOVEA.L Dn,A7 then ILLEGAL
	lea	stk1,sp
	arm	r1
	move.l	#stk2,d0
	move.l	d0,sp
	illegal
r1:	chkl	hsp,stk2-FRAME0,1
	chkl	sp,stk2,2
	moveq	#0,d0
	move.w	lastvec,d0
	chkl	d0,4,3
	; ADDQ.L #8,A7 then A-line
	lea	stk1-$100,sp
	arm	r4
	addq.l	#8,sp
	dc.w	$A000
r4:	chkl	hsp,stk1-$100+8-FRAME0,4
	chkl	sp,stk1-$100+8,5
	; SUBA.W #$100,A7 then ILLEGAL, the ILLEGAL prefetched
	lea	stk1,sp
	arm	r6
	moveq	#1,d2
	divu.w	#1,d2
	suba.w	#$100,sp
	illegal
r6:	chkl	hsp,stk1-$100-FRAME0,6
	chkl	sp,stk1-$100,7
	; SUBQ.L #4,A7 then F-line (no coprocessor dialogue), other alignment
	lea	stk1,sp
	arm	r8
	moveq	#1,d2
	divu.w	#1,d2
	nop
	subq.l	#4,sp
	dc.w	$FFC0
r8:	chkl	hsp,stk1-4-FRAME0,8
	chkl	sp,stk1-4,9
	; master stack: MOVEA.L Dn,A7 with M=1 then ILLEGAL; the frame goes on the MSP
	move.l	#stk3,d0
	movec	d0,msp
	move.w	#$3700,sr
	arm	r10
	moveq	#1,d2
	divu.w	#1,d2
	move.l	#stk3-$100,d0
	move.l	d0,sp
	illegal
r10:	move.w	#$3700,sr
	movec	msp,d1
	move.w	#$2700,sr
	lea	stk1,sp
	chkl	hsp,stk3-$100-FRAME0,10
	chkl	d1,stk3-$100,11

;---------------------------------------------------------------- MOVEC and the active stack pointer
	; MOVEC Dn,ISP then MOVE.L A7,Dn, two alignments
	lea	stk1,sp
	move.l	#stk2,d0
	moveq	#1,d2
	divu.w	#1,d2
	movec	d0,isp
	move.l	sp,d1
	lea	stk1,sp
	chkl	d1,stk2,20
	moveq	#1,d2
	divu.w	#1,d2
	nop
	movec	d0,isp
	move.l	sp,d1
	lea	stk1,sp
	chkl	d1,stk2,21
	; MOVEC Dn,ISP then ADDQ.L #4,A7
	moveq	#1,d2
	divu.w	#1,d2
	movec	d0,isp
	addq.l	#4,sp
	move.l	sp,d1
	lea	stk1,sp
	chkl	d1,stk2+4,22
	; MOVEA.L Dn,A7 then MOVEC ISP,Dn, prefetched, two alignments
	move.l	#stk2,d0
	moveq	#1,d2
	divu.w	#1,d2
	move.l	d0,sp
	movec	isp,d1
	lea	stk1,sp
	chkl	d1,stk2,23
	move.l	#stk2,d0
	moveq	#1,d2
	divu.w	#1,d2
	nop
	move.l	d0,sp
	movec	isp,d1
	lea	stk1,sp
	chkl	d1,stk2,24
	; master stack: MOVEC Dn,MSP with M=1 then MOVE.L A7,Dn
	move.l	#stk3,d0
	movec	d0,msp
	move.w	#$3700,sr
	move.l	#stk3-$40,d0
	movec	d0,msp
	move.l	sp,d1
	move.w	#$2700,sr
	chkl	d1,stk3-$40,25

;---------------------------------------------------------------- pop, then an address error on the odd target
	; RTS
	lea	stk1,sp
	arm	r30
	move.l	#oddt,-(sp)
	rts
r30:	moveq	#0,d0
	move.w	lastvec,d0
	chkl	d0,3,30
	chkl	hsp,stk1-FRAMEB,31
	chkl	sp,stk1,32
	; RTD #8
	lea	stk1,sp
	arm	r33
	move.l	#oddt,-(sp)
	rtd	#8
r33:	chkl	hsp,stk1+8-FRAMEB,33
	chkl	sp,stk1+8,34
	; RTR
	lea	stk1,sp
	arm	r35
	move.l	#oddt,-(sp)
	move.w	#0,-(sp)
	rtr
r35:	chkl	hsp,stk1-FRAMEB,35
	chkl	sp,stk1,36
	; RTE, format $0 frame with a supervisor SR
	lea	stk1,sp
	arm	r37
	move.w	#$0000,-(sp)
	move.l	#oddt,-(sp)
	move.w	#$2700,-(sp)
	rte
r37:	chkl	hsp,stk1-FRAMEB,37
	chkl	sp,stk1,38

	lea	stk1,sp
	move.w	#$600D,(DONEREG).l
done:	bra.s	done

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

;---------------------------------------------------------------- handler
; every exception: record A7 and the vector, return to resume with S set
; and IPL 7 (the RTE of an address error frame resumes at its PC field)
h_any:
	move.l	sp,hsp
	move.l	d6,-(sp)
	move.w	10(sp),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	move.l	resume,6(sp)
	move.w	#$2700,4(sp)
	move.l	(sp)+,d6
	rte
