; AP68030 coprocessor interface: the take address and transfer data and the
; transfer to/from top of stack primitives (UM 10.4.11, 10.4.12) keep the
; evaluated effective address (UM 10.4.10: only evaluate and transfer EA,
; evaluate EA and transfer data and transfer multiple coprocessor registers
; set it) and move their operands in data space.
	include	"asm/t_cpx.i"

WCI	equ	$400000		; RAM alias with CIIN: every read reaches the bus

start:
	bsr	init
	clrlog
;---------------------------------------------------------------- 1. take address: the write to previously evaluated EA goes to the EA
	move.l	#$11111111,buf
	move.l	#$22222222,buf2
	move.l	#buf2,SCRADDR
	move.l	#$5A5A5A5A,SCROP
	script	$9704,$8504,$A004,$0902	; EA (buf) to cp; take address buf2 to cp; write prev EA
	dc.w	$F239,$0030
	dc.l	buf
	move.l	buf,d1
	chkl	d1,$5A5A5A5A,1
	move.l	buf2,d1
	chkl	d1,$22222222,2
	move.l	OPREG1,d1
	chkl	d1,$22222222,3		; the take address operand reached the coprocessor
;---------------------------------------------------------------- 2. -(A7) from the coprocessor: likewise
	move.l	#$11111111,buf
	move.l	sp,a5
	script	$9704,$AE04,$A004,$0902	; EA (buf) to cp; -(A7) from cp; write prev EA
	dc.w	$F239,$0030
	dc.l	buf
	move.l	a5,d1
	sub.l	sp,d1
	chkl	d1,4,4
	move.l	(sp)+,d1
	chkl	d1,$5A5A5A5A,5
	move.l	buf,d1
	chkl	d1,$5A5A5A5A,6
;---------------------------------------------------------------- 3. the take address operand is read in data space after a PC-relative EA
	move.l	#WCI+buf2,SCRADDR
	move.l	#WCI+buf2,WATCH
	script	$9704,$8504,$0902	; EA (d16,PC) to cp; take address to cp
	dc.w	$F23A,$0030
	dc.w	pcdat-*
	move.w	WATCHFC,d1
	and.w	#7,d1
	chkw	d1,5,7			; supervisor data
	bra.s	c3
	cnop	0,4
pcdat:	dc.l	$77777777
c3:
;---------------------------------------------------------------- 4. (A7)+ to the coprocessor is read in data space after a PC-relative EA
	move.l	#$00000001,d0
	movec	d0,cacr			; data cache off: the stack read reaches the bus
	move.l	#$33333333,-(sp)
	move.l	sp,WATCH
	script	$9704,$8E04,$0902	; EA (d16,PC) to cp; (A7)+ to cp
	dc.w	$F23A,$0030
	dc.w	pcdat-*
	move.w	WATCHFC,d1
	and.w	#7,d1
	chkw	d1,5,8
	move.l	OPREG1,d1
	chkl	d1,$33333333,9
	move.l	#$3111,d0
	movec	d0,cacr
;---------------------------------------------------------------- 5. a mid-instruction frame after a take address holds the evaluated EA
	clrlog
	move.l	#buf2,SCRADDR
	script	$9704,$8504,$1D30,$0902	; EA (buf) to cp; take address; mid-instruction exception 48
	dc.w	$F239,$0030
	dc.l	buf
	chkw	exccnt,1,10
	chkw	logfmt,9,11
	move.l	logea,d1
	chkl	d1,buf,12
	endtest
