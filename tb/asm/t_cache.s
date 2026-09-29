; AP68030 on-chip cache self test (UM Section 6)
; assembled with vasmm68k_mot -Fbin -m68030
;
; The testbench's clock counter ($F00150) measures access costs, so hits,
; misses, bursts and freezes are observable; the RAM aliases at $200000
; (16-bit port) write memory behind the logical caches, so staleness rules
; can be checked directly:
;   - the instruction cache is not updated by data writes (self-modifying
;     code needs a CACR clear)
;   - a data read hit ignores memory (write-through keeps them equal only
;     for writes made through the same logical address)
;   - CD/CI clear all entries, CED/CEI clear the entry selected by CAAR
;   - FD/FI freeze: no new entries, write hits still update
;   - WA: an aligned longword write miss allocates, a byte write does not
;   - DBE/IBE: a line miss bursts four longwords (fewer clocks for the
;     neighbours)

FAILREG	equ	$F00100
DONEREG	equ	$F00102
CLKREG	equ	$F00150
W16	equ	$200000

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
	dc.l	$3400
	dc.l	start
	rept	254
	dc.l	unexp
	endr

	org	$400
start:
;---------------------------------------------------------------- data cache off: every read goes to the bus
	moveq	#0,d0
	movec	d0,cacr
	move.l	#$11111111,($4000).l
	move.l	#$22222222,W16+$4000	; behind the cache: visible, no cache
	move.l	($4000).l,d1
	chkl	d1,$22222222,1

;---------------------------------------------------------------- data cache on, no burst: a read allocates, a hit ignores memory
	move.l	#$0101,d0		; EI, ED
	movec	d0,cacr
	move.l	#$33333333,($4010).l	; word/byte writes do not allocate; a long miss with WA=0 neither
	move.l	($4010).l,d1		; miss: read from memory, entry allocated
	chkl	d1,$33333333,2
	move.l	#$44444444,W16+$4010	; changes memory only
	move.l	($4010).l,d1		; hit: the cached value
	chkl	d1,$33333333,3
	move.l	W16+$4010,d1		; another logical address: from memory
	chkl	d1,$44444444,4
	; a write hit updates the entry (write-through)
	move.l	#$55555555,($4010).l
	move.l	($4010).l,d1
	chkl	d1,$55555555,5
	move.l	W16+$4010,d1
	chkl	d1,$55555555,6
	; a byte write hit updates only its byte
	move.b	#$AA,($4011).l
	move.l	($4010).l,d1
	chkl	d1,$55AA5555,7

;---------------------------------------------------------------- CD clears everything, CED one entry
	move.l	#$0101+$800,d0		; CD
	movec	d0,cacr
	move.l	#$66666666,W16+$4010
	move.l	($4010).l,d1		; miss again: memory
	chkl	d1,$66666666,8
	move.l	#$12345678,($4014).l	; memory (WA off: no allocation)
	move.l	($4014).l,d2		; allocate the neighbour
	move.l	#$77777777,W16+$4010
	move.l	#$88888888,W16+$4014
	move.l	#$4010,d0
	movec	d0,caar
	move.l	#$0101+$400,d0		; CED: entry of CAAR
	movec	d0,cacr
	move.l	($4010).l,d1
	chkl	d1,$77777777,9		; cleared: from memory
	move.l	($4014).l,d2
	chkl	d2,$12345678,10		; its neighbour kept its entry

;---------------------------------------------------------------- write allocate
; (an alias write with WA set would allocate or invalidate the same index,
; so the memory-behind writes are made with WA clear)
	move.l	#$0101+$2000,d0		; WA
	movec	d0,cacr
	move.l	#$99999999,($4020).l	; aligned long write miss: allocated
	move.l	#$0101,d0		; WA off: alias writes leave the cache alone
	movec	d0,cacr
	move.l	#$AAAAAAAA,W16+$4020
	move.l	($4020).l,d1
	chkl	d1,$99999999,11		; hit on the allocated entry
	move.l	W16+$4020,d1
	chkl	d1,$AAAAAAAA,12		; memory has the alias write
	move.l	#$0101+$2000,d0		; WA
	movec	d0,cacr
	move.l	($4024).l,d1		; allocate by reading
	move.w	#$BBBB,W16+$4024	; word write miss (alias, same index): the entry is invalidated
	move.l	#$CCCCCCCC,($4024).l	; long write, tag matches the invalid entry: validated
	move.l	#$0101,d0
	movec	d0,cacr
	move.l	#$DDDDDDDD,W16+$4024	; memory only
	move.l	($4024).l,d1
	chkl	d1,$CCCCCCCC,13		; hit
	move.l	#$EEEEEEEE,($4028).l	; write miss with WA=0: nothing allocated
	move.l	#$FFFFFFFF,W16+$4028
	move.l	($4028).l,d1
	chkl	d1,$FFFFFFFF,14

;---------------------------------------------------------------- freeze: misses do not allocate, hits update
	move.l	($4030).l,d1		; allocate $4030
	move.l	#$0101+$200,d0		; FD
	movec	d0,cacr
	move.l	#$0F0F0F0F,W16+$4034
	move.l	($4034).l,d1		; miss: memory, not allocated
	chkl	d1,$0F0F0F0F,30
	move.l	#$1F1F1F1F,W16+$4034
	move.l	($4034).l,d1
	chkl	d1,$1F1F1F1F,15		; still from memory
	move.l	#$2F2F2F2F,($4030).l	; hit while frozen: updated
	move.l	#$3F3F3F3F,W16+$4030
	move.l	($4030).l,d1
	chkl	d1,$2F2F2F2F,16

;---------------------------------------------------------------- burst: a line miss with DBE fetches the neighbours
	move.l	#$0101+$800,d0		; clear the data cache, no burst
	movec	d0,cacr
	move.l	#$0101,d0
	movec	d0,cacr
	move.l	#$41414141,($4100).l
	move.l	#$42424242,($4104).l
	move.l	#$43434343,($4108).l
	move.l	#$44444444,($410C).l
	move.l	#$0101+$800,d0
	movec	d0,cacr
	move.l	#$0101,d0
	movec	d0,cacr
	move.l	($4100).l,d1		; single entry fill
	move.l	#$52525252,W16+$4104
	move.l	($4104).l,d1		; miss: the neighbour was not filled
	chkl	d1,$52525252,17
	move.l	#$0101+$800,d0
	movec	d0,cacr
	move.l	#$1101,d0		; DBE
	movec	d0,cacr
	move.l	($4100).l,d1		; burst fills the line
	move.l	#$62626262,W16+$4104
	move.l	($4104).l,d1		; hit: the burst brought the old value
	chkl	d1,$52525252,18
	move.l	($410C).l,d1
	chkl	d1,$44444444,19

;---------------------------------------------------------------- the instruction cache is not snooped by data writes
; This block and its subroutines fit in one 256-byte window (all sixteen
; lines), so the callers never evict the subroutine lines.
	cnop	0,256
	move.l	#$0101+$800+$8,d0	; clear both caches
	movec	d0,cacr
	move.l	#$1101+$10,d0		; caches on, bursts on
	movec	d0,cacr
	moveq	#0,d3
	bsr	smc			; cached: nop; moveq #1,d3; rts
	chkl	d3,1,20
	move.w	#$7605,smc+2		; memory now says moveq #5,d3
	moveq	#0,d3
	bsr	smc			; the cached copy still executes
	chkl	d3,1,21
	move.l	#$1111+$8,d0		; CI: clear the instruction cache
	movec	d0,cacr
	moveq	#0,d3
	bsr.s	smc
	chkl	d3,5,22			; the new code runs
	; CEI clears only the entry named by CAAR
	move.w	#$7607,smc+2
	moveq	#0,d3
	bsr.s	smc
	chkl	d3,5,23			; stale again
	lea	smc+2,a0		; the longword holding the modified word
	move.l	a0,d0
	movec	d0,caar
	move.l	#$1111+$4,d0		; CEI
	movec	d0,cacr
	moveq	#0,d3
	bsr.s	smc
	chkl	d3,7,24
	bra.s	smc_fi
smc:
	nop
	moveq	#1,d3
	rts
smc_fi:
;---------------------------------------------------------------- freeze the instruction cache: new code is not cached
	move.l	#$1111+$8,d0		; CI: nothing resident
	movec	d0,cacr
	move.l	#$1111+$2,d0		; FI
	movec	d0,cacr
	moveq	#0,d3
	bsr	smc2			; not resident: fetched but not stored
	chkl	d3,2,25
	move.w	#$7609,smc2+2
	moveq	#0,d3
	bsr	smc2
	chkl	d3,9,26			; memory, since nothing was cached
	bra.s	smc2_end
	cnop	0,16
smc2:
	nop
	moveq	#2,d3
	rts
smc2_end:

;---------------------------------------------------------------- hit and miss timing
	move.l	#$1111,d0
	movec	d0,cacr
	move.l	($4200).l,d1		; miss (bursts the line)
	move.l	CLKREG,d4
	move.l	($4200).l,d1		; hit
	move.l	CLKREG,d5
	sub.l	d4,d5			; clocks for a hit sequence
	move.l	#$1111+$800,d0
	movec	d0,cacr
	move.l	#$1111,d0
	movec	d0,cacr
	move.l	CLKREG,d4
	move.l	($4200).l,d1		; miss
	move.l	CLKREG,d6
	sub.l	d4,d6
	cmp.l	d5,d6			; the miss took longer than the hit
	bhi.s	tmok
	failt	27
tmok:

	move.w	#$600D,(DONEREG).l
	stop	#$2700

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
