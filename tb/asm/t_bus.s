; AP68030 dynamic bus sizing and misalignment self test
; assembled with vasmm68k_mot -Fbin -m68030
;
; The testbench maps the same RAM at four windows with different ports:
;   $000000  32-bit synchronous, burst
;   $200000  16-bit asynchronous
;   $300000  8-bit asynchronous
;   $400000  32-bit asynchronous, CIIN asserted (never cached)
; Every test writes through one window and reads back through another, so
; both the write lanes (UM Table 7-5) and the read assembly (UM Table 7-4)
; of the port pairs are checked.  Addresses are not reused between tests
; because the caches are logical: an alias must never see stale data.
;
; protocol with the testbench:
;   word write to $F00100 = failing test number
;   word write to $F00102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102
W32	equ	$000000
W16	equ	$200000
W8	equ	$300000
WCI	equ	$400000

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

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start		; initial PC
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
	move.l	#$00003111,d0	; caches on, burst on, write allocate
	movec	d0,cacr

;---------------------------------------------------------------- longs, every
; alignment, written through each port and read through the others
	lea	($5000).l,a0		; base of this test's area (offset only)

; write long at offset 0..3 via the 32-bit port, read via 16 and 8
	move.l	#$11223344,W32+$5000
	move.l	#$55667788,W32+$5005
	move.l	#$99AABBCC,W32+$500A
	move.l	#$DDEEFF01,W32+$500F
	move.l	W16+$5000,d0
	chkl	d0,$11223344,1
	move.l	W16+$5005,d0
	chkl	d0,$55667788,2
	move.l	W8+$500A,d0
	chkl	d0,$99AABBCC,3
	move.l	W8+$500F,d0
	chkl	d0,$DDEEFF01,4
	move.l	WCI+$5005,d0
	chkl	d0,$55667788,5

; write via the 16-bit port, read via 32 and 8
	move.l	#$21222324,W16+$5020
	move.l	#$25262728,W16+$5025
	move.l	#$292A2B2C,W16+$502A
	move.l	#$2D2E2F20,W16+$502F
	move.l	W32+$5020,d0
	chkl	d0,$21222324,6
	move.l	W8+$5025,d0
	chkl	d0,$25262728,7
	move.l	W32+$502A,d0
	chkl	d0,$292A2B2C,8
	move.l	W8+$502F,d0
	chkl	d0,$2D2E2F20,9

; write via the 8-bit port, read via 32 and 16
	move.l	#$31323334,W8+$5040
	move.l	#$35363738,W8+$5045
	move.l	#$393A3B3C,W8+$504A
	move.l	#$3D3E3F30,W8+$504F
	move.l	W32+$5040,d0
	chkl	d0,$31323334,10
	move.l	W16+$5045,d0
	chkl	d0,$35363738,11
	move.l	W32+$504A,d0
	chkl	d0,$393A3B3C,12
	move.l	W16+$504F,d0
	chkl	d0,$3D3E3F30,13

;---------------------------------------------------------------- words and bytes
	move.w	#$4142,W16+$5061	; odd word via 16-bit port
	move.w	#$4344,W8+$5063
	move.w	#$4546,W32+$5065
	move.b	#$47,W16+$5067
	move.b	#$48,W8+$5068
	move.b	#$49,W32+$5069
	move.w	W32+$5061,d0
	and.l	#$FFFF,d0
	chkl	d0,$4142,14
	move.w	W32+$5063,d0
	and.l	#$FFFF,d0
	chkl	d0,$4344,15
	move.w	W8+$5065,d0
	and.l	#$FFFF,d0
	chkl	d0,$4546,16
	move.b	W32+$5067,d0
	and.l	#$FF,d0
	chkl	d0,$47,17
	move.b	W16+$5068,d0
	and.l	#$FF,d0
	chkl	d0,$48,18
	move.b	W8+$5069,d0
	and.l	#$FF,d0
	chkl	d0,$49,19
	; the neighbouring bytes are untouched
	move.l	W32+$5060,d0
	and.l	#$FF0000FF,d0
	chkl	d0,$00000043,20

;---------------------------------------------------------------- crossing a
; cache line (16 bytes) and a page: the first portion cannot burst
	move.l	#$5152535E,W32+$507E
	move.l	W16+$507E,d0
	chkl	d0,$5152535E,21
	move.l	#$6162636E,W16+$50FE
	move.l	W32+$50FE,d0
	chkl	d0,$6162636E,22
	move.l	#$7172737E,W8+$51FE	; page boundary for 512-byte pages too
	move.l	W32+$51FE,d0
	chkl	d0,$7172737E,23

;---------------------------------------------------------------- read hits
; after writes through the same window (write-through, allocate on aligned
; longs, invalidate otherwise)
	lea	W32+$5200,a1
	move.l	#$81828384,(a1)		; allocates the entry
	move.l	(a1),d0			; hit
	chkl	d0,$81828384,24
	move.w	#$8586,2(a1)		; hit: entry updated
	move.l	(a1),d0
	chkl	d0,$81828586,25
	move.b	#$87,1(a1)
	move.l	(a1),d0
	chkl	d0,$81878586,26
	move.l	W16+$5200,d0		; memory agrees
	chkl	d0,$81878586,27

;---------------------------------------------------------------- MOVEP through the 8-bit port
	lea	W8+$5300,a2
	move.l	#$A1A2A3A4,d1
	movep.l	d1,1(a2)
	move.b	W32+$5301,d0
	and.l	#$FF,d0
	chkl	d0,$A1,28
	move.b	W32+$5307,d0
	and.l	#$FF,d0
	chkl	d0,$A4,29
	moveq	#0,d2
	movep.w	1(a2),d2
	and.l	#$FFFF,d2
	chkl	d2,$A1A2,30

;---------------------------------------------------------------- TAS and CAS (RMW) on each port
	move.b	#$05,W32+$5400
	tas	W16+$5400
	move.b	W8+$5400,d0
	and.l	#$FF,d0
	chkl	d0,$85,31
	move.w	#$1234,W8+$5402
	move.w	#$1234,d3		; compare
	move.w	#$5678,d4		; update
	cas.w	d3,d4,W16+$5402
	bne.s	casfail
	move.w	W32+$5402,d0
	and.l	#$FFFF,d0
	chkl	d0,$5678,32
	bra.s	casok
casfail:
	failt	33
casok:
	move.l	#$01020304,W16+$5404
	move.l	#$FFFFFFFF,d3
	move.l	#$AAAAAAAA,d4
	cas.l	d3,d4,W8+$5404		; mismatch: d3 loaded, memory unchanged
	beq.s	casfail2
	chkl	d3,$01020304,34
	move.l	W32+$5404,d0
	chkl	d0,$01020304,35
	bra.s	casok2
casfail2:
	failt	36
casok2:

;---------------------------------------------------------------- the CIIN window is never cached:
; a write through another window is visible at once
	move.l	#$C0C1C2C3,WCI+$5500
	move.l	WCI+$5500,d0
	chkl	d0,$C0C1C2C3,37
	move.l	#$C4C5C6C7,W16+$5500
	move.l	WCI+$5500,d0
	chkl	d0,$C4C5C6C7,38

;---------------------------------------------------------------- instruction fetch through
; the narrow ports: run code from the 16-bit and 8-bit windows
	jsr	W16+narrow
	chkl	d0,$12345678,39
	jsr	W8+narrow
	chkl	d0,$12345678,40

;---------------------------------------------------------------- MOVEM misaligned through the 16-bit port
	lea	W16+$5602,a3
	move.l	#$D1D2D3D4,d5
	move.l	#$D5D6D7D8,d6
	movem.l	d5-d6,(a3)
	move.l	W32+$5602,d0
	chkl	d0,$D1D2D3D4,41
	move.l	W8+$5606,d0
	chkl	d0,$D5D6D7D8,42
	moveq	#0,d5
	moveq	#0,d6
	movem.l	W8+$5602,d5-d6
	chkl	d5,$D1D2D3D4,43
	chkl	d6,$D5D6D7D8,44

	move.w	#$600D,(DONEREG).l
	stop	#$2700

;---------------------------------------------------------------- code executed from a narrow window
narrow:
	move.l	#$12345678,d0
	nop
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
