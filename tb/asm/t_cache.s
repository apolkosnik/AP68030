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
;   - a bus error on the first cycle of a burst leaves the whole line
;     invalid, in either cache

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


;---------------------------------------------------------------- snooped external writes
; A data cache entry created by write allocation (even in the CIIN window,
; since CIIN is ignored on writes) goes stale when another master writes the
; memory.  Without a snoop it is used (the MC68030 has no snooping); with the
; system's snoop port the entry is invalidated.
DMAADR	equ	$F001B0
DMADAT	equ	$F001B4
DMAGO	equ	$F001B8
NMIOPT	equ	$F001BA
WCI	equ	$400000
IPLREG	equ	$F00110
	move.l	#$3111+$800,d0		; clear; caches on, WA
	movec	d0,cacr
	move.l	#$3111,d0
	movec	d0,cacr
	move.l	#$AAAA0001,($4310).l	; aligned long write: allocated
	move.l	($4310).l,d1		; hit
	move.l	#$4310,DMAADR
	move.l	#$BBBB0002,DMADAT
	move.w	#2,DMAGO		; DMA write, no snoop
	nop
	move.l	($4310).l,d1
	chkl	d1,$AAAA0001,40		; the stale entry (plain 68030 behaviour)
	move.l	#$CCCC0003,DMADAT
	move.w	#1,DMAGO		; DMA write, snooped
	nop
	nop
	move.l	($4310).l,d1
	chkl	d1,$CCCC0003,41		; invalidated: memory
	move.l	#$DDDD0004,(WCI+$4320).l	; CIIN window: allocated all the same
	move.l	#WCI+$4320,DMAADR
	move.l	#$EEEE0005,DMADAT
	move.w	#1,DMAGO
	nop
	nop
	move.l	(WCI+$4320).l,d1
	chkl	d1,$EEEE0005,42

;---------------------------------------------------------------- the level 7 vector past the data cache
; nmi_vec_nocache off: a vector held in the data cache is used; on: the
; vector is read from the bus (where a freezer cartridge would overlay it)
	move.w	#$2700,sr
	move.l	#nmi1,($7C).l		; vector 31, cached by write allocation
	move.l	($7C).l,d1
	move.l	#$7C,DMAADR
	move.l	#nmi2,DMADAT
	move.w	#2,DMAGO		; memory changes behind the cache
	nop
	moveq	#0,d0
	move.w	#7,IPLREG		; level 7: taken even at mask 7
	nop
	nop
	nop
	chkl	d0,1,43			; the cached vector
	move.w	#1,NMIOPT		; now fetched from the bus
	move.l	#nmi1,($7C).l
	move.l	($7C).l,d1
	move.w	#2,DMAGO		; memory says nmi2 again, the cache nmi1
	nop
	moveq	#0,d0
	move.w	#7,IPLREG
	nop
	nop
	nop
	chkl	d0,2,44
	move.w	#0,NMIOPT

;---------------------------------------------------------------- read-modify-write reads and the data cache
; the read of a CAS/TAS is always a bus cycle; the entry it reads is filled
; from that data, so a stale entry does not survive it and a cold one is
; allocated (single entry, no burst)
	move.l	#$3111+$800,d0
	movec	d0,cacr
	move.l	#$3111,d0
	movec	d0,cacr
	move.l	#$11111111,($4330).l	; allocated
	move.l	($4330).l,d1
	move.l	#$4330,DMAADR
	move.l	#$22222222,DMADAT
	move.w	#2,DMAGO		; memory changes, the entry does not
	nop
	moveq	#0,d0			; compare fails: memory is $22222222
	cas.l	d0,d2,($4330).l
	chkl	d0,$22222222,45		; the RMW read went to memory
	move.l	($4330).l,d1
	chkl	d1,$22222222,46		; and the entry followed it
	move.b	#$05,($4340).l		; allocated (byte write, WA)
	move.b	($4340).l,d1
	move.l	#$4340,DMAADR
	move.l	#$06000000,DMADAT
	move.w	#2,DMAGO
	nop
	tas	($4340).l		; reads $06, writes $86
	move.b	($4340).l,d1
	and.l	#$FF,d1
	chkl	d1,$86,47
	move.l	#$3111+$800,d0		; cold cache: a CAS read allocates
	movec	d0,cacr
	move.l	#$3111,d0
	movec	d0,cacr
	move.l	#$4350,DMAADR
	move.l	#$33333333,DMADAT
	move.w	#2,DMAGO		; memory $33333333
	nop
	moveq	#0,d0
	cas.l	d0,d2,($4350).l		; fails, reads and allocates $33333333
	move.l	#$44444444,DMADAT
	move.w	#2,DMAGO		; memory changes behind the entry
	nop
	move.l	($4350).l,d1
	chkl	d1,$33333333,48		; the allocated entry is used

;---------------------------------------------------------------- bus error on the first cycle of a burst
; UM 6.1.3.2 (p. 6-19) and 7.5.1: "If the bus error occurs during the first
; cycle of a burst (i.e., before burst mode is entered), the data read from
; the bus is ignored, and the entire associated cache line is marked
; invalid."  The line read holds four valid entries of another tag; after
; the bus error they must miss.  The bus error trigger ($F130) asserts BERR
; with STERM (and CBACK) on the synchronous port.
BERRADR	equ	$F00130
BERRSP	equ	$6080			; data cache line 8: away from the lines used
	moveq	#0,d0
	movec	d0,cacr
	move.l	#$A0A0A0A0,($4560).l
	move.l	#$A1A1A1A1,($4564).l
	move.l	#$A2A2A2A2,($4568).l
	move.l	#$A3A3A3A3,($456C).l
	move.l	#$0101+$800+$8,d0	; clear both caches
	movec	d0,cacr
	move.l	#$1101,d0		; data burst, no write allocation
	movec	d0,cacr
	move.l	($4560).l,d1		; line 6: burst, four entries (tag $45)
	move.l	#$B1B1B1B1,W16+$4564	; memory behind the entries
	move.l	#$B3B3B3B3,W16+$456C
	move.l	($4564).l,d1
	chkl	d1,$A1A1A1A1,49		; the burst cached the neighbour
	move.l	#berr_h,($8).l		; bus error handler
	move.l	#$5560,BERRADR		; line 6, other tag
	move.l	sp,BERRSP
	lea	berr_c1,a1
	move.l	($5560).l,d1		; line miss: CBREQ; BERR on the first cycle
	failt	50			; (no bus error)
berr_c1:
	clr.l	BERRADR
	move.l	($456C).l,d1
	chkl	d1,$B3B3B3B3,51		; the whole line is invalid: memory
	bra	berr_blk
; the instruction cache: a routine's line, cached by a burst, then a jump
; to the last longword of the same line in another tag, which errs
; (this block fits in one 256-byte window, so the code around the routine
; never evicts its line)
	cnop	0,256
berr_blk:
	move.l	#$0101+$800+$8,d0
	movec	d0,cacr
	move.l	#$0111,d0		; instruction burst
	movec	d0,cacr
	moveq	#0,d3
	bsr.s	berr_fn			; its line: burst, four entries
	chkl	d3,1,52
	move.w	#$7605,berr_fn		; memory: moveq #5,d3
	moveq	#0,d3
	bsr.s	berr_fn
	chkl	d3,1,53			; the cached copy
	move.l	#berr_fn+$800C,BERRADR
	move.l	sp,BERRSP
	lea	berr_c2,a1
	jmp	berr_fn+$800C		; line miss: CBREQ; BERR on the first cycle
berr_c2:
	clr.l	BERRADR
	moveq	#0,d3
	bsr.s	berr_fn
	chkl	d3,5,54			; the whole line is invalid: memory
	move.l	#unexp,($8).l
	bra.s	berr_end
berr_h:	move.l	BERRSP,sp		; drop the bus error frame
	jmp	(a1)
	cnop	0,16
berr_fn:
	moveq	#1,d3
	rts
	cnop	0,16
berr_end:

	move.w	#$600D,(DONEREG).l
	stop	#$2700

nmi1:	moveq	#1,d0
	bra.s	nmi_x
nmi2:	moveq	#2,d0
nmi_x:	move.w	#0,IPLREG
	rte

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
