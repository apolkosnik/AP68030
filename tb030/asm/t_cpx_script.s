; AP68030 coprocessor interface: the model's script itself (UM 10.4)
; Primitives the processor handles as the UM describes, to check the
; script mechanism: register transfers, an evaluated EA, a come again.
	include	"asm/t_cpx.i"

start:
	bsr	init
	clrlog
;---------------------------------------------------------------- 1. transfer single main processor register D3 to the coprocessor
	move.l	#$D3D3D3D3,d3
	script	$8C03,$0902
	dc.w	$F200,$0030
	move.l	OPREG0,d1
	chkl	d1,$D3D3D3D3,1
;---------------------------------------------------------------- 2. operand CIR to D4, after a null come again
	move.l	#$13579BDF,SCROP
	moveq	#0,d4
	script	$8800,$AC04,$0902
	dc.w	$F200,$0030
	chkl	d4,$13579BDF,2
;---------------------------------------------------------------- 3. evaluate EA (a0) and transfer 4 bytes, then write it back to the EA
	lea	buf,a0
	move.l	#$600DF00D,(a0)
	move.l	#$12121212,SCROP
	script	$9704,$A004,$0902
	dc.w	$F210,$0030
	move.l	OPREG0,d1
	chkl	d1,$600DF00D,3
	move.l	buf,d1
	chkl	d1,$12121212,4
;---------------------------------------------------------------- 4. the CPU space cycle counter
	move.l	SCRIPT,d5
	script	$0902
	dc.w	$F200,$0030		; command write, response read
	move.l	SCRIPT,d1
	sub.l	d5,d1
	chkl	d1,2,5
	chkw	exccnt,0,6
	endtest
