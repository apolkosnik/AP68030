; AP68030 user mode under a split address space (UNIX System V, Atari ASV)
; assembled with vasmm68k_mot -Fbin -m68030 -m68851
;
; ASV runs the kernel untranslated through TT0/TT1, which match only the
; supervisor function codes, and every process through the CRP (SRE set).
; A user access issued with a supervisor function code is then not
; translated at all.  Here user space is logical $80FFxxxx: through the CRP
; it is RAM at $20000-$23FFF, but untranslated it is the testbench's bus
; error area ($FFxxxx, A31-A24 ignored), so any such access faults in
; supervisor mode and fails the test.
;
; protocol with the testbench:
;   word write to $F00100 = failing test number
;   word write to $F00102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102

TBLA	equ	$10000		; A table (16 x 4), CRP and SRP
TBLB	equ	$11000		; B table of region F (256 x 4)
UCODE	equ	$20000		; physical user code page   (logical $80FF0000)
USTK	equ	$21000		; physical user stack page  (logical $80FF1000)
UCODE2	equ	$22000		; physical second code page (logical $80FF2000), mapped late
UDATA	equ	$23000		; physical data page        (logical $80FF3000), mapped late
UWP	equ	$24000		; physical data page        (logical $80FF4000), write protected

LCODE	equ	$80FF0000
LSTK	equ	$80FF2000	; user stack top: the end of the stack page
LCODE2	equ	$80FF2000
LDATA	equ	$80FF3000
LWP	equ	$80FF4000

TC_OFF	equ	$02C84800
TC_ON	equ	$82C84800	; E, SRE, PS = 4K, IS = 8, TIA = 4, TIB = 8
TT0_V	equ	$003F8143	; ASV: $00000000-$3FFFFFFF, supervisor, R/W ignored
TT1_V	equ	$807F8543	; ASV: $80000000-$FFFFFFFF, supervisor, cache inhibited

lastvec	equ	$3000
lastssw	equ	$3002
lastfa	equ	$3004
exccnt	equ	$3008		; bus errors taken
mode	equ	$300A
fixaddr	equ	$300C
fixval	equ	$3010
trapcnt	equ	$3014		; trap #0 taken from user mode
retpc	equ	$3018		; where trap #1 returns to
irqcnt	equ	$301C		; level 5 interrupts taken
sweep	equ	$301E		; clocks to the interrupt
scr	equ	$3020

IRQLVL	equ	$F00111		; byte: interrupt level now
IRQDLY	equ	$F00114		; word: clocks until IRQDLV applies
IRQDLV	equ	$F00117		; byte: level applied after the delay
SPHIREG	equ	$F001E0		; long: supervisor program reads with A31 set

M_FAIL	equ	0
M_FIX	equ	1

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

setl	macro
	move.l	#\2,(\1).l
	endm

cclr	macro
	move.l	#$3111+$808,d0
	movec	d0,cacr
	move.l	#$3111,d0
	movec	d0,cacr
	endm

; enter user code at logical \1 with the user stack at LSTK; trap #1 returns to \2
touser	macro
	move.l	#\2,retpc
	move.l	#LSTK,a0
	move.l	a0,usp
	clr.w	-(sp)			; format 0
	pea	(\1).l
	move.w	#$0000,-(sp)
	rte
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	dc.l	h_buserr	; 2
	rept	26		; 3-28
	dc.l	h_unexp
	endr
	dc.l	h_irq		; 29: level 5 autovector
	dc.l	h_unexp		; 30
	dc.l	h_unexp		; 31
	dc.l	h_trap0		; 32: count, back to user
	dc.l	h_trap1		; 33: back to supervisor, after the trap
	dc.l	h_trap2		; 34: a signal: frame on the user stack, to the user handler
	rept	221		; 35-255
	dc.l	h_unexp
	endr

	org	$400
start:
	move.l	#$3111,d0	; caches on, bursts, write allocate
	movec	d0,cacr
	clr.w	mode
	clr.w	exccnt
	clr.w	trapcnt
	moveq	#1,d0
	movec	d0,sfc
	movec	d0,dfc

	; translation tables: region 0 identity (code, tables), region F a B table
	lea	TBLA,a0
	move.w	#(TBLB+1024-TBLA)/4-1,d0
clrt:	clr.l	(a0)+
	dbf	d0,clrt
	setl	TBLA+0,$00000001
	setl	TBLA+60,TBLB+2
	setl	TBLB+$F0*4,UCODE+1
	setl	TBLB+$F1*4,USTK+1
	setl	TBLB+$F4*4,UWP+5	; write protected
					; $F2 and $F3 invalid until the faults map them

	; the user programs into their pages
	lea	ucode_src(pc),a0
	lea	UCODE,a1
	move.w	#(ucode_end-ucode_src)/2-1,d0
cpu1:	move.w	(a0)+,(a1)+
	dbf	d0,cpu1
	lea	ucode2_src(pc),a0
	lea	UCODE2,a1
	move.w	#(ucode2_end-ucode2_src)/2-1,d0
cpu2:	move.w	(a0)+,(a1)+
	dbf	d0,cpu2
	setl	UDATA,$5EED0003

	; ASV's configuration
	move.l	#TT0_V,(scr).l
	pmove	(scr).l,tt0
	move.l	#TT1_V,(scr).l
	pmove	(scr).l,tt1
	move.l	#$7FFF0002,(scr).l
	move.l	#TBLA,(scr+4).l
	pmove	(scr).l,crp
	pmove	(scr).l,srp
	move.l	#TC_ON,(scr).l
	pmove	(scr).l,tc
	cclr

;================================================================ 1. RTE to user, user stack, traps
	; the user program: stack reads and writes, trap #0 twice, trap #1 home
	touser	LCODE,t1_back
t1_back:
	chkl	d0,$12345678,1		; read back from the user stack
	chkl	d1,$80FF1FFC,2		; its stack pointer before the trap
	chkw	trapcnt,2,3
	chkw	exccnt,0,4
	move.l	(USTK+$FFC).l,d1	; what it pushed, physically
	chkl	d1,$12345678,5

;================================================================ 2. the same, many times
	; trap entry and return, back to back, with an RTE to user mode each time
	move.w	#199,d5
t2_loop:
	clr.w	trapcnt
	touser	LCODE,t2_back
t2_back:
	chkw	trapcnt,2,6
	dbf	d5,t2_loop
	chkw	exccnt,0,7

;================================================================ 3. a fault on a user code page
	; the user program jumps to $80FF2000, not yet mapped; the handler maps it
	move.w	#M_FIX,mode
	move.l	#TBLB+$F2*4,fixaddr
	move.l	#UCODE2+1,fixval
	clr.w	exccnt
	touser	LCODE+(ujump-ucode_src),t3_back
t3_back:
	chkl	d0,$C0DE0002,8		; the second page ran
	chkw	exccnt,1,9		; one fault, not a repeat
	move.w	lastssw,d1
	and.l	#$0007,d1
	chkl	d1,2,10			; user program

;================================================================ 4. a fault on a user data page
	move.l	#TBLB+$F3*4,fixaddr
	move.l	#UDATA+1,fixval
	clr.w	exccnt
	touser	LCODE+(uread-ucode_src),t4_back
t4_back:
	chkl	d0,$5EED0003,12		; the rerun read
	chkw	exccnt,1,13
	move.w	lastssw,d1
	and.l	#$0147,d1
	chkl	d1,$0141,14		; DF, read, user data
	move.l	lastfa,d1
	chkl	d1,LDATA,15

;================================================================ 5. a signal
	; trap #2 builds a frame on the user stack with MOVES and returns to a
	; user handler that reads it, several times nested
	move.w	#M_FAIL,mode
	clr.w	exccnt
	touser	LCODE+(usig-ucode_src),t5_back
t5_back:
	chkl	d0,$516A0005,16		; the handler saw its argument
	chkl	d1,LSTK-3*12,17		; three nested frames
	chkw	exccnt,0,18

;================================================================ 6. copy on write
	; the user program writes a write protected page; the handler unprotects it
	move.w	#M_FIX,mode
	move.l	#TBLB+$F4*4,fixaddr
	move.l	#UWP+1,fixval
	clr.w	exccnt
	touser	LCODE+(uwrite-ucode_src),t6_back
t6_back:
	chkw	exccnt,1,19
	move.l	(UWP).l,d1
	chkl	d1,$C0DE0006,20
	move.w	lastssw,d1
	and.l	#$0147,d1
	chkl	d1,$0101,21		; DF, write, user data

;================================================================ 7-10. the same with an interrupt at every clock
	; a level 5 interrupt arrives n clocks into each run, n = 1..360: at the
	; RTE to user mode, in a trap's entry or return, in a fault and its rerun
	clr.w	irqcnt
	move.b	#5,(IRQDLV).l		; the level first: a delay of 1 ends at once
	move.w	#1,sweep
t7_loop:
	clr.w	trapcnt
	clr.w	exccnt
	move.w	#M_FAIL,mode
	move.w	sweep,(IRQDLY).l
	touser	LCODE,t7_back
t7_back:
	bsr	irq_drain
	chkw	trapcnt,2,22
	chkl	d0,$12345678,23
	chkw	exccnt,0,24

	; a fault on the user code page
	setl	TBLB+$F2*4,0
	pflusha
	cclr
	move.w	#M_FIX,mode
	move.l	#TBLB+$F2*4,fixaddr
	move.l	#UCODE2+1,fixval
	clr.w	exccnt
	move.w	sweep,(IRQDLY).l
	touser	LCODE+(ujump-ucode_src),t8_back
t8_back:
	bsr	irq_drain
	chkl	d0,$C0DE0002,25
	chkw	exccnt,1,26

	; a fault on the user data page
	setl	TBLB+$F3*4,0
	pflusha
	cclr
	move.l	#TBLB+$F3*4,fixaddr
	move.l	#UDATA+1,fixval
	clr.w	exccnt
	move.w	sweep,(IRQDLY).l
	touser	LCODE+(uread-ucode_src),t9_back
t9_back:
	bsr	irq_drain
	chkl	d0,$5EED0003,27
	chkw	exccnt,1,28

	; copy on write
	setl	TBLB+$F4*4,UWP+5
	setl	UWP,0
	pflusha
	cclr
	move.l	#TBLB+$F4*4,fixaddr
	move.l	#UWP+1,fixval
	clr.w	exccnt
	move.w	sweep,(IRQDLY).l
	touser	LCODE+(uwrite-ucode_src),t10_back
t10_back:
	bsr	irq_drain
	chkw	exccnt,1,29
	move.l	(UWP).l,d1
	chkl	d1,$C0DE0006,30

	addq.w	#1,sweep
	cmp.w	#361,sweep
	bne	t7_loop
	chkw	irqcnt,4*360,31		; every one taken, once
	; no instruction fetch of the user half went out as supervisor program:
	; prefetch must not continue the user stream once an exception sets S
	; (such a fetch bus errors and is discarded, so only the bus shows it)
	move.l	(SPHIREG).l,d1
	chkl	d1,0,32

	; 33: RTE of a throwaway frame whose SR is user (ASV's signal return):
	; the next frame is on the user stack and read as user data, through the
	; CRP (read as supervisor data, TT1 passes it untranslated: bus error)
	move.w	#M_FAIL,mode
	clr.w	exccnt
	move.l	#t33_back,retpc
	move.w	#$0000,(USTK+$FF8).l	; logical LSTK-8: SR, PC, format 0
	move.l	#LCODE+(uh_done-ucode_src),(USTK+$FFA).l
	move.w	#$0000,(USTK+$FFE).l
	move.l	#LSTK-8,a0
	move.l	a0,usp
	move.w	#$1000,-(sp)		; format 1, throwaway
	pea	($DEADBEE0).l		; not used
	move.w	#$0000,-(sp)		; SR: user
	rte
t33_back:
	chkw	exccnt,0,33
	move.l	usp,a0
	chkl	a0,LSTK,34		; the user frame popped

	move.l	#TC_OFF,(scr).l
	pmove	(scr).l,tc
	move.w	#$600D,(DONEREG).l
	stop	#$2700

;---------------------------------------------------------------- user programs (position independent)
ucode_src:
	move.l	#$12345678,-(sp)
	move.l	(sp),d0
	move.l	sp,d1
	trap	#0
	add.l	#0,d0
	trap	#0
	move.l	(sp)+,d2
	trap	#1
ujump:	jmp	(LCODE2).l		; on an unmapped page
uread:	move.l	(LDATA).l,d0		; from an unmapped page
	trap	#1
uwrite:	move.l	#$C0DE0006,(LWP).l	; to a write protected page
	nop
	trap	#1
usig:	moveq	#3,d4
	trap	#2			; "signal": to ucode_h with an argument on the stack
	trap	#1
ucode_h:				; the user handler: the argument at 4(sp)
	move.l	4(sp),d0
	move.l	sp,d1
	subq.l	#1,d4
	beq.s	uh_done
	trap	#2			; nested, as a handler that faults again would be
uh_done:
	trap	#1
ucode_end:

ucode2_src:
	move.l	#$C0DE0002,d0
	trap	#1
	nop
ucode2_end:

;---------------------------------------------------------------- handlers
h_buserr:
	movem.l	d6/a5/a6,-(sp)
	lea	12(sp),a6
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	move.w	$A(a6),lastssw
	move.l	$10(a6),lastfa
	addq.w	#1,exccnt
	cmp.w	#3,exccnt		; a fault that repeats
	bge.s	hb_rep
	move.w	mode,d6
	cmp.w	#M_FIX,d6
	bne.s	hb_fail
	move.l	fixaddr,a5
	move.l	fixval,(a5)
	pflusha
	movem.l	(sp)+,d6/a5/a6
	rte
hb_rep:	move.w	#$F003,d7
	bra	fail_all
hb_fail:
	move.w	(a6),d7			; stacked SR: supervisor = $F2xx, user = $F0xx
	and.w	#$2000,d7
	lsr.w	#4,d7
	or.w	#$F002,d7
	bra	fail_all

; an interrupt still to come arrives with the mask lowered, then none is pending
irq_drain:
	move.w	#$2000,sr
	move.w	#400,d6
idr:	dbf	d6,idr
	move.w	#$2700,sr
	clr.w	(IRQDLY).l
	clr.b	(IRQLVL).l
	rts

h_irq:
	clr.b	(IRQLVL).l
	addq.w	#1,irqcnt
	rte

h_trap0:
	addq.w	#1,trapcnt
	rte

h_trap1:				; to the supervisor address in retpc
	move.w	#$2700,(sp)
	move.l	retpc,2(sp)
	rte

h_trap2:				; frame: d4 and the return PC, under an argument
	move.l	a0,-(sp)
	move.l	usp,a0
	move.l	6(sp),d6		; the PC after the trap
	moves.l	d6,-(a0)
	move.l	#$516A0005,d6
	moves.l	d6,-(a0)
	moves.l	d4,-(a0)		; 12 bytes a frame
	move.l	a0,usp
	move.l	#LCODE+(ucode_h-ucode_src),6(sp)
	move.l	(sp)+,a0
	rte

h_unexp:
	move.w	6(sp),d7
	and.w	#$0FFF,d7
	lsr.w	#2,d7
	or.w	#$FF00,d7
	bra	fail_all

fail_all:
	move.l	#TC_OFF,(scr).l
	pmove	(scr).l,tc
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1
