FAILREG	equ	$F00100
DONEREG	equ	$F00102
	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr
	org	$400
start:
	move.l	#$00003111,d0
	movec	d0,cacr
	moveq	#3,d6			; the first passes fill the instruction cache
loop:
	move.l	a7,a5
	lea	($3200).l,a7
	move.l	#$11111111,(a7)
	move.l	#$22222222,-8(a7)
	ori.w	#$2000,sr		; as Kickstart's Supervisor()
	subq.l	#8,a7
	move.w	sr,(a7)
	move.l	#$00F80CC0,2(a7)
	move.w	#$20,6(a7)
	move.w	(a7),d0
	move.l	($3200).l,d1
	move.l	a5,a7
	dbf	d6,loop
	move.w	sr,d2
	cmp.w	d2,d0
	bne.s	fl1
	cmp.l	#$11111111,d1		; the old A7 slot must be untouched
	bne.s	fl2
	move.w	#$600D,(DONEREG).l
	stop	#$2700
fl1:	move.w	#1,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	stop	#$2700
fl2:	move.w	#2,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	stop	#$2700
unexp:	move.w	#$FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
	stop	#$2700
