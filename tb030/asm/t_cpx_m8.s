; AP68030 tracing with coprocessor instructions and instruction traps.
;  - A pending trace keeps the cpGEN dialog going until null CA=0 PF=1 or a
;    take post-instruction exception (UM 10.5.2.5); a transfer of SR and
;    scanPC into the processor makes a trace on change of flow pending (UM
;    10.4.17).
;  - The exception of an instruction trap (TRAP #n, TRAPcc, CHK, divide by
;    zero) and a coprocessor post-instruction exception are processed before
;    the trace (UM 8.1.12 Table 8-5): the trace handler runs first, its frame
;    holds the handler's address and, as instruction address, the traced
;    instruction (UM Table 8-6; WinUAE trace_pc).  UM 8.1.7 traces instruction
;    traps in both trace modes (WinUAE only with T1; the UM is followed).
;  - A pre-instruction exception (not executed, UM 8.1.7) has no trace; a
;    mid-instruction one suspends the instruction, which is traced when it
;    completes after the RTE.  An interrupt is never traced.
	include	"asm/t_cpx.i"

start:
	bsr	init
;---------------------------------------------------------------- 1. T1: the dialog goes on after a CA=0 primitive
	move.l	#$D3D3D3D3,d3
	moveq	#0,d4
	move.l	#$44444444,SCROP
	clrlog
	script	$0C03,$2C04,$0902	; D3 to cp (CA=0); operand to D4 (CA=0); null PF=1
	move.w	#$A700,sr
c1:	dc.w	$F200,$0030
c1n:	move.w	#$2700,sr
	chkl	d4,$44444444,1
	chkw	exccnt,1,2
	chkw	logvec,9,3
	move.l	logpc,d1
	chkl	d1,c1n,4
	move.l	logia,d1
	chkl	d1,c1,5
;---------------------------------------------------------------- 2. T0: SR from the coprocessor is a change of flow
	move.l	#$27002700,SCROP
	clrlog
	script	$A200,$0902		; SR <- $2700 (CA); null PF=1
	move.w	#$6700,sr
	dc.w	$F200,$0030
	move.w	#$2700,sr
	chkw	exccnt,1,6
	chkw	logvec,9,7
;---------------------------------------------------------------- 3. ... also across a mid-instruction frame (interrupt during come again)
	move.l	#$20002000,SCROP
	clrlog
	script	$A200,$8900,$0902	; SR <- $2000 (CA); null CA IA; null PF=1
	move.w	#3,IRQARM		; level 3 at the command write
	move.w	#$6000,sr		; T0, interrupts on
	dc.w	$F200,$0030
	move.w	#$2700,sr
	chkw	exccnt,2,8
	chkw	logvec,27,9
	chkw	logfmt,9,10
	chkw	logvec+2,9,11
;---------------------------------------------------------------- 4. T1 and a post-instruction exception: that exception, then the trace
	clrlog
	script	$1E31			; take post-instruction exception, vector 49
	move.w	#$A700,sr
c4:	dc.w	$F200,$0030
c4n:	move.w	#$2700,sr
	chkw	exccnt,2,12
	chkw	logvec,9,13		; the trace handler runs first ...
	move.l	logpc,d1
	chkl	d1,h_exc,14		; ... in front of the post-instruction handler
	move.l	logia,d1
	chkl	d1,c4,15
	chkw	logvec+2,49,16
	move.l	logpc+4,d1
	chkl	d1,c4n,17
;---------------------------------------------------------------- 5. T1 and a pre-instruction exception with vector 7: no trace
	move.l	#4,skip
	clrlog
	script	$1C07			; take pre-instruction exception, vector 7
	move.w	#$A700,sr
	dc.w	$F200,$0030
	move.w	#$2700,sr
	chkw	exccnt,1,18
	chkw	logvec,7,19
	clr.l	skip
;---------------------------------------------------------------- 6. T1 and a mid-instruction exception with vector 33: traced when the instruction completes
	clrlog
	script	$1D21,$0902		; take mid-instruction exception, vector 33; null PF=1
	move.w	#$A700,sr
c6:	dc.w	$F200,$0030
c6n:	move.w	#$2700,sr
	chkw	exccnt,2,20
	chkw	logvec,33,21
	chkw	logfmt,9,22
	chkw	logvec+2,9,23
	move.l	logpc+4,d1
	chkl	d1,c6n,24
	move.l	logia+4,d1
	chkl	d1,c6,25
;---------------------------------------------------------------- 7. T1 and TRAP #0: the trace's instruction address is the TRAP
	clrlog
	move.w	#$A700,sr
c7:	trap	#0
c7n:	move.w	#$2700,sr
	chkw	exccnt,2,26
	chkw	logvec,9,27
	move.l	logpc,d1
	chkl	d1,h_exc,28
	move.l	logia,d1
	chkl	d1,c7,29
	chkw	logvec+2,32,30
	move.l	logpc+4,d1
	chkl	d1,c7n,31
;---------------------------------------------------------------- 8. T1 and a CHK trap
	moveq	#-1,d0
	moveq	#5,d1
	clrlog
	move.w	#$A700,sr
c8:	chk.w	d1,d0
	move.w	#$2700,sr
	chkw	exccnt,2,32
	chkw	logvec,9,33
	move.l	logia,d1
	chkl	d1,c8,34
	chkw	logvec+2,6,35
	move.l	logia+4,d1
	chkl	d1,c8,36
;---------------------------------------------------------------- 9. T1 and a divide by zero
	moveq	#0,d1
	clrlog
	move.w	#$A700,sr
c9:	divu.w	d1,d0
	move.w	#$2700,sr
	chkw	exccnt,2,37
	chkw	logvec,9,38
	move.l	logia,d1
	chkl	d1,c9,39
	chkw	logvec+2,5,40
;---------------------------------------------------------------- 10. T0 and TRAP #0: traced (UM 8.1.7: instruction traps)
	clrlog
	move.w	#$6700,sr
c10:	trap	#0
	move.w	#$2700,sr
	chkw	exccnt,2,41
	chkw	logvec,9,42
	move.l	logia,d1
	chkl	d1,c10,43
	chkw	logvec+2,32,44
;---------------------------------------------------------------- 11. an interrupt with vector 40 after a traced instruction: one trace
	move.w	#40,irqvec
	move.w	#40,VECREG
	clrlog
	move.w	#$A000,sr		; T1, IPL 0
c11:	move.w	#3,IPLREG		; traced; level 3 pending at its end
	nop
	move.w	#$2700,sr
	move.w	#0,VECREG
	clr.w	irqvec
	chkw	exccnt,2,45
	chkw	logvec,40,46		; the interrupt handler runs first ...
	chkw	logvec+2,9,47		; ... and returns to the trace handler
	move.l	logia+4,d1
	chkl	d1,c11,48
	endtest
