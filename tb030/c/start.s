; AP68030 startup for compiled C programs: vectors, stack, caches, main,
; verdict through the testbench registers
	section	"CODE",code
	xref	_main
	dc.l	$000F0000		; initial ISP (top of the RAM window, below $100000)
	dc.l	_start
	rept	254
	dc.l	_unexp
	endr
_start:
	move.l	#$00003111,d0		; both caches on, bursts, write allocate
	movec	d0,cacr
	jsr	_main
	tst.l	d0
	bne.s	_fail
	move.w	#$600D,($F00102).l
	stop	#$2700
_fail:
	move.w	d0,($F00100).l
	move.w	#$BAD0,($F00102).l
_h1:	bra.s	_h1
_unexp:
	move.w	#$00FF,($F00100).l
	move.w	#$BAD0,($F00102).l
_h2:	bra.s	_h2
