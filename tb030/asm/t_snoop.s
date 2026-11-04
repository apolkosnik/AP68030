; AP68030 snoop port self test (ap030_top: snoop_we/snoop_addr)
; assembled with vasmm68k_mot -Fbin -m68030
;
; Another bus master writes a longword the processor has just read on the
; bus and reports the write on the snoop port; the data cache entry for it
; must not survive, whenever the snoop comes: also while the fill of that
; read is still on its way to the cache (the bytes are latched, the entry is
; reported and registered, a narrow port or a burst reads the line over
; several cycles).  The testbench's DMA trigger ($F1D0) runs the write
; 0..15 processor clocks after the read's first bus cycle ends (or after the
; native port read the longword); a later read must see the new data.
;   part 1  32-bit synchronous port (the native port in FAST_PORT builds)
;   part 2  16-bit port (two DSACK cycles per longword)
;   part 3  burst fill: the write hits the line's third longword
;   part 4  8-bit port (four DSACK cycles per longword)
; failing test number = 16 * (part - 1) + delay + 1

FAILREG	equ	$F00100
DONEREG	equ	$F00102
DMAADR	equ	$F001B0
DMADAT	equ	$F001B4
TRIGA	equ	$F001D0
TRIGD	equ	$F001D4
W16	equ	$200000
W8	equ	$300000

	org	0
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	moveq	#0,d0
	movec	d0,cacr
	moveq	#0,d7			; test number base
;---------------------------------------------------------------- part 1
	move.l	#$0101+$800,d0		; caches on, data cache cleared, no burst
	movec	d0,cacr
	move.l	#$0101,d0
	movec	d0,cacr
	lea	$4400,a0		; written by the DMA
	lea	$4400,a1		; read by the processor
	moveq	#0,d6			; offset into the line read (first longword)
	bsr	sweep
;---------------------------------------------------------------- part 2
	moveq	#16,d7
	lea	$4500,a0
	lea	W16+$4500,a1
	bsr	sweep
;---------------------------------------------------------------- part 3
	moveq	#32,d7
	move.l	#$1101,d0		; data burst enabled
	movec	d0,cacr
	lea	$4608,a0		; the third longword of the line
	lea	$4600,a1		; the burst starts at the first
	moveq	#8,d6
	bsr	sweep
;---------------------------------------------------------------- part 4
	moveq	#48,d7
	move.l	#$0101,d0
	movec	d0,cacr
	lea	$4700,a0
	lea	W8+$4700,a1
	moveq	#0,d6
	bsr	sweep

	move.w	#$600D,(DONEREG).l
	stop	#$2700

; one line per delay: a0 the DMA address, a1 the processor's first read,
; d6 the offset of the DMA longword from a1
sweep:
	moveq	#0,d5			; delay
sw_loop:
	move.l	#$11110000,d1
	add.l	d5,d1
	move.l	d1,(a0)			; old value (no write allocation)
	move.l	a0,DMAADR
	move.l	#$22220000,d2
	add.l	d5,d2
	move.l	d2,DMADAT
	move.w	d5,TRIGD
	move.l	a1,TRIGA
	move.l	(a1),d3			; miss: the line or entry is filled, the DMA follows
	move.w	#40,d4
sw_wait:
	dbra	d4,sw_wait		; the write and its snoop are over
	move.l	(a1,d6.l),d3
	cmp.l	d2,d3			; the other master's data
	beq.s	sw_ok
	add.w	d5,d7
	addq.w	#1,d7
	bra	fail_all
sw_ok:
	lea	16(a0),a0
	lea	16(a1),a1
	addq.l	#1,d5
	cmp.l	#16,d5
	bne	sw_loop
	rts

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1

unexp:
	move.w	#$00FF,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt2:
	bra.s	halt2
