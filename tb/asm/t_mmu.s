; AP68030 memory management unit self test (UM Section 9)
; assembled with vasmm68k_mot -Fbin -m68030 -m68851
;
; The testbench's 1 MB RAM is aliased at every 1 MB region of the 24-bit
; physical space (A31-A24 are ignored); the region selects the port:
;   $0xxxxx 32-bit synchronous burst, $2xxxxx 16-bit, $3xxxxx 8-bit,
;   $4xxxxx 32-bit with CIIN, other regions 32-bit asynchronous,
;   $F0xxxx test registers, $FFxxxx bus error.
; The translation trees live in physical RAM below $20000, identity mapped
; by an early termination descriptor together with the code, the stack and
; the variables; the mapped data pages are at $20000-$3FFFF.  Descriptors
; are read back through logical region 5, which is mapped cache inhibited
; onto the same RAM, because the walker's history updates bypass the
; logical data cache exactly as on the real part.
;
; protocol with the testbench:
;   word write to $F00100 = failing test number
;   word write to $F00102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102
PINREG	equ	$F00170		; bit 0 asserts MMUDIS
CIOCNT	equ	$F00174		; bus cycles with CIOUT asserted

; physical layout
TBLA	equ	$10000		; short A table (16 x 4)
TBLAL	equ	$10080		; long A table (16 x 8)
TBLFC	equ	$10100		; function code table (8 x 4)
INDT	equ	$10140		; targets of the indirect descriptors
TBLA2	equ	$10200		; alternate A table (SRP tree, FCL supervisor tree)
TBLB	equ	$11000		; short B table of region 1 (256 x 4)
TBLBL	equ	$12000		; long B table of region 2 (256 x 8)
TBLB2	equ	$13000		; short B table of region 1, alternate tree
TBL3A	equ	$14000		; three-level tree: A (16 x 4)
TBL3B	equ	$14100		;                   B (64 x 4)
TBL3C	equ	$14200		;                   C (64 x 4)

; translation control values
TC_OFF	equ	$00C84800
TC_BASE	equ	$80C84800	; E, PS = 4K, IS = 8, TIA = 4, TIB = 8
TC_FCL	equ	$81C84800	; ... with function code lookup
TC_SRE	equ	$82C84800	; ... with the supervisor root pointer
TC_3L	equ	$80884660	; E, PS = 256, IS = 8, TIA = 4, TIB = 6, TIC = 6

; scratch variables
lastvec	equ	$3000		; word: vector number of the last exception
lastfmt	equ	$3002		; word: frame format
lastpc	equ	$3004		; long: stacked PC
lastsr	equ	$3008		; word: stacked SR
lastssw	equ	$300A		; word: special status word (bus fault frames)
lastfa	equ	$300C		; long: fault address
exccnt	equ	$3010		; word: exceptions taken
mode	equ	$3012		; word: bus error handler behaviour
fixaddr	equ	$3014		; long: descriptor the handler repairs (M_FIX)
fixval	equ	$3018		; long: value it stores there
scr	equ	$3020		; 8 bytes: PMOVE source operand
scr2	equ	$3028		; 8 bytes: PMOVE destination operand
udata	equ	$3030		; long: result from user mode code

M_FAIL	equ	0		; unexpected: report $F0xx
M_FIX	equ	1		; store fixval at fixaddr, flush the ATC, rerun
M_SKIP	equ	2		; clear DF (write dropped, read gets $DEADBEEF)
M_RTE	equ	3		; plain RTE (configuration exception)

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

; check a memory word
chkw	macro
	move.w	\1,d6
	and.l	#$FFFF,d6
	cmp.l	#\2,d6
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; store a longword at a physical address (identity mapped)
setl	macro
	move.l	#\2,(\1).l
	endm

; read a physical longword through the cache inhibited region 5
rdd	macro
	move.l	(\1+$500000).l,\2
	endm

; clear both caches (the caches are logical: mappings changed)
cclr	macro
	move.l	#$3111+$808,d0
	movec	d0,cacr
	move.l	#$3111,d0
	movec	d0,cacr
	endm

newmap	macro
	pflusha
	cclr
	endm

settc	macro
	move.l	#\1,(scr).l
	pmove	(scr).l,tc
	cclr
	endm

setcrp	macro
	move.l	#\1,(scr).l
	move.l	#\2,(scr+4).l
	pmove	(scr).l,crp
	cclr
	endm

settt0	macro
	move.l	#\1,(scr).l
	pmove	(scr).l,tt0
	cclr
	endm

; record the frame at a6
record2	macro
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	move.w	6(a6),d6
	lsr.w	#8,d6
	lsr.w	#4,d6
	move.w	d6,lastfmt
	move.l	2(a6),lastpc
	move.w	(a6),lastsr
	addq.w	#1,exccnt
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	dc.l	h_buserr	; 2
	dc.l	h_unexp		; 3
	rept	28		; 4-31
	dc.l	h_unexp
	endr
	dc.l	h_trap0		; 32
	rept	23		; 33-55
	dc.l	h_unexp
	endr
	dc.l	h_mmucfg	; 56
	rept	199		; 57-255
	dc.l	h_unexp
	endr

	org	$400
start:
	move.l	#$3111,d0	; caches on, bursts, write allocate
	movec	d0,cacr
	clr.w	mode
	clr.w	exccnt
	moveq	#1,d0
	movec	d0,sfc		; MOVES reads user data
	movec	d0,dfc

;================================================================ 1. registers (translation off)
	move.l	#$7FC84800,(scr).l	; bits 30-26 are reserved
	pmove	(scr).l,tc
	pmove	tc,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,$03C84800,1
	move.l	#$7FFFFFF2,(scr).l
	move.l	#$0001000F,(scr+4).l
	pmove	(scr).l,crp
	pmove	crp,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,$7FFF0002,2
	move.l	(scr2+4).l,d1
	chkl	d1,$00010000,3
	move.l	#$80000003,(scr).l
	move.l	#$00010200,(scr+4).l
	pmove	(scr).l,srp
	pmove	srp,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,$80000003,4
	move.l	(scr2+4).l,d1
	chkl	d1,$00010200,5
	move.l	#$FFFF7FFF,(scr).l	; E clear: an enabled CI window over everything
	pmove	(scr).l,tt0		; would leave the cache stale (UM 6.1)
	pmove	tt0,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,$FFFF0777,6
	move.l	#0,(scr).l
	pmove	(scr).l,tt0
	pmove	(scr).l,tt1
	move.w	#$FFFF,(scr).l
	pmove	(scr).l,mmusr
	pmove	mmusr,(scr2).l
	chkw	scr2,$EE47,7
	move.w	#0,(scr).l
	pmove	(scr).l,mmusr

;================================================================ 2. configuration exceptions
	move.w	#M_RTE,mode
	clr.w	exccnt
	move.l	#$80C84C00,(scr).l	; PS + IS + TIA + TIB = 36
	pmove	(scr).l,tc
cfgpc:
	chkw	exccnt,1,8
	chkw	lastvec,56,9
	chkw	lastfmt,2,10
	move.l	lastpc,d1
	chkl	d1,cfgpc,11		; post-instruction: PC = next instruction
	pmove	tc,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,$00C84C00,12		; loaded, E cleared
	move.l	#$80784800,(scr).l	; PS = 7 is reserved
	pmove	(scr).l,tc
	chkw	exccnt,2,13
	move.l	#$00000000,(scr).l	; root pointer with DT = 0
	move.l	#$00010000,(scr+4).l
	pmove	(scr).l,crp
	chkw	exccnt,3,14
	chkw	lastvec,56,15
	pmove	crp,(scr2).l
	move.l	(scr2).l,d1
	chkl	d1,0,16			; loaded before the exception
	move.l	#$7FFF0000,(scr).l
	pmove	(scr).l,srp
	chkw	exccnt,4,17

;================================================================ 3. build the translation trees
	lea	TBLA,a0
	move.w	#(TBL3C+256-TBLA)/4-1,d0
clrt:	clr.l	(a0)+
	dbf	d0,clrt

	; short A table: 16 regions of 1 MB
	setl	TBLA+0,$00000001	; region 0: early termination, identity (code, data, tables)
	setl	TBLA+4,TBLB+2		; region 1: short B table
	setl	TBLA+8,TBLBL+3		; region 2: long B table
	setl	TBLA+16,$00400005	; region 4: identity onto the CIIN port, write protected
	setl	TBLA+20,$00500041	; region 5: identity onto the async port, cache inhibited
	setl	TBLA+24,$00600001	; region 6: identity onto the async port, cachable
	setl	TBLA+28,$00FF0002	; region 7: B table in the bus error area
	setl	TBLA+60,$00F00041	; region F: test registers, cache inhibited

	; short B table of region 1: 4K pages
	setl	TBLB+0,$00220001	; $100000 -> $220000 (16-bit port)
					; $101000 invalid
	setl	TBLB+8,$00022005	; $102000 -> $022000, write protected
	setl	TBLB+12,INDT+2		; $103000 -> indirect, short target
	setl	INDT+0,$00023001	;   -> $023000
	setl	TBLB+16,$00024001	; $104000 -> $024000 (history bits)
	setl	TBLB+20,$00025001	; $105000 -> $025000
	setl	TBLB+24,$00026001	; $106000 -> $026000 (code)
	setl	TBLB+28,$00027001	; $107000 -> $027000 (code at the end of the page)
					; $108000 invalid
	lea	TBLB+64,a0		; $110000-$127000 -> $030000-$047000
	move.l	#$00030001,d1
	moveq	#23,d0
fillb:	move.l	d1,(a0)+
	add.l	#$1000,d1
	dbf	d0,fillb

	; long B table of region 2
	setl	TBLBL+0,$00000101	; $200000: supervisor only
	setl	TBLBL+4,$00020000	;   -> $020000
	setl	TBLBL+8,$00000005	; $201000: write protected
	setl	TBLBL+12,$00021000
	setl	TBLBL+16,$00000041	; $202000: cache inhibited
	setl	TBLBL+20,$00022000
	setl	TBLBL+24,$00000003	; $203000: indirect, long target
	setl	TBLBL+28,INDT+8
	setl	INDT+8,$00000001	;   -> $023000
	setl	INDT+12,$00023000
	setl	TBLBL+32,$00000001	; $204000 -> $024000
	setl	TBLBL+36,$00024000

	; alternate tree: region 1 maps $100000 elsewhere
	lea	TBLA,a0
	lea	TBLA2,a1
	moveq	#15,d0
cpa:	move.l	(a0)+,(a1)+
	dbf	d0,cpa
	setl	TBLA2+4,TBLB2+2
	setl	TBLB2+0,$00028001	; $100000 -> $028000

	; function code table
	setl	TBLFC+4,TBLA+2		; user data
	setl	TBLFC+8,TBLA+2		; user program
	setl	TBLFC+20,TBLA2+2	; supervisor data: the alternate tree
	setl	TBLFC+24,TBLA+2		; supervisor program

	; long A table
	setl	TBLAL+0,$7FFF0001	; region 0: identity, no limit
	setl	TBLAL+8,$00040002	; region 1: short B table, index 0..4
	setl	TBLAL+12,TBLB
	setl	TBLAL+16,$7FFF0003	; region 2: long B table
	setl	TBLAL+20,TBLBL
	setl	TBLAL+120,$7FFF0041	; region F: registers
	setl	TBLAL+124,$00F00000

	; three-level tree, 256-byte pages
	setl	TBL3A+0,$00000001	; region 0: identity
	setl	TBL3A+4,TBL3B+2		; region 1
	setl	TBL3A+60,$00F00041	; registers
	setl	TBL3B+0,TBL3C+2		; $100000-$103FFF: C table
	setl	TBL3B+4,$00030001	; $104000-$107FFF -> $030000 (early termination at B)
	setl	TBL3C+0,$00029001	; $100000 -> $029000
	setl	TBL3C+4,$00029A01	; $100100 -> $029A00
					; $100200 invalid

	; data behind the mapped pages
	setl	$20000,$11111111
	setl	$28000,$22222222
	setl	$22000,$33333333
	setl	$23000,$44444444
	setl	$24000,$55555555
	setl	$25000,$66666666
	setl	$21000,$77777777
	setl	$20FFC,$AAAACCDD
	setl	$29000,$31313131
	setl	$290FC,$41414242
	setl	$29A00,$43434444
	setl	$31234,$34343434
	; code: a subroutine at $26000, NOPs and a JMP (A1) ending page $27000
	lea	code_page(pc),a0
	lea	$26000,a1
	moveq	#3,d0
cpc:	move.l	(a0)+,(a1)+
	dbf	d0,cpc
	setl	$27FF4,$4E714E71
	setl	$27FF8,$4E714E71
	setl	$27FFC,$4E714ED1

;================================================================ 4. translation on: history bits
	setcrp	$7FFF0002,TBLA
	settc	TC_BASE
	nop
	move.l	($100000).l,d1		; through the tree onto the 16-bit port
	chkl	d1,$11111111,20
	rdd	TBLA+0,d1
	and.l	#8,d1
	chkl	d1,8,21			; U set by the instruction fetches
	rdd	TBLB+0,d1
	chkl	d1,$00220009,22		; U set, M clear: only read
	rdd	TBLA+4,d1
	chkl	d1,TBLB+$A,23		; the table descriptor has U too
	move.l	#$5A5A5A5A,($104000).l
	nop
	rdd	TBLB+16,d1
	chkl	d1,$00024019,24		; U and M
	rdd	$24000,d1
	chkl	d1,$5A5A5A5A,25
	move.l	($105000).l,d1		; read first ...
	chkl	d1,$66666666,26
	rdd	TBLB+20,d1
	chkl	d1,$00025009,27
	move.l	#$69696969,($105000).l	; ... then write: the entry is reloaded with M
	nop
	rdd	TBLB+20,d1
	chkl	d1,$00025019,28
	rdd	$25000,d1
	chkl	d1,$69696969,29

;================================================================ 5. PTEST, PLOAD, PFLUSH
	ptestr	#5,($105000).l,#7,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$0202,30		; M, two levels
	move.l	a1,d1
	chkl	d1,TBLB+20,31
	ptestr	#5,($105000).l,#1,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$0001,32		; stopped at the A table
	move.l	a1,d1
	chkl	d1,TBLA+4,33
	ptestr	#5,($101000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0402,34		; invalid
	ptestw	#5,($102000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0802,35		; write protected
	ptestr	#5,($000400).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0201,36		; early termination: one level (M: the region holds the variables)
	ptestr	#5,($103000).l,#7,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$0003,37		; indirect: three descriptors
	move.l	a1,d1
	chkl	d1,INDT,38
	ptestr	#1,($200000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$2002,39		; supervisor violation for user data
	ptestr	#5,($200000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0002,40
	ptestr	#5,($700000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$8401,41		; bus error fetching the B table
	; level 0: the ATC
	ptestr	#5,($105000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,42		; resident, modified
	ptestr	#5,($10F000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,43		; not resident
	ptestr	#1,($105000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,44		; other function code: another entry
	; PLOAD
	pflusha
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,45
	ploadr	#5,($104000).l
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,46
	rdd	TBLB+24,d1
	chkl	d1,$00026001,47		; $106000 untouched so far
	ploadw	#5,($106000).l
	rdd	TBLB+24,d1
	chkl	d1,$00026019,48		; PLOADW updates U and M
	; PFLUSH by function code
	ploadr	#1,($104000).l
	pflush	#5,#7
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,49
	ptestr	#1,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,50		; the user entry survives
	; PFLUSH by function code and address
	ploadr	#5,($104000).l
	ploadr	#5,($105000).l
	pflush	#5,#7,($104000).l
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,51
	ptestr	#5,($105000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,52
	pflush	#1,#4,($104000).l	; mask: only FC2 compared
	ptestr	#1,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,53
	ptestr	#5,($105000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,54
	; PMOVEFD keeps the ATC, PMOVE flushes it
	ploadr	#5,($104000).l
	move.l	#TC_BASE,(scr).l
	pmovefd	(scr).l,tc
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0200,55
	pmove	(scr).l,tc
	ptestr	#5,($104000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0400,56

;================================================================ 6. faults repaired by the handler, rerun by RTE
	move.w	#M_FIX,mode
	clr.w	exccnt
	move.l	#TBLB+4,fixaddr
	move.l	#$00021001,fixval
	move.l	($101000).l,d1		; invalid page: the handler maps it
	chkl	d1,$77777777,60
	chkw	exccnt,1,61
	chkw	lastvec,2,62
	chkw	lastfmt,$B,63
	move.l	lastfa,d1
	chkl	d1,$101000,64
	move.w	lastssw,d1
	and.l	#$01FF,d1
	chkl	d1,$0145,65		; DF, read, long, supervisor data
	; write protection: the handler clears WP
	move.l	#TBLB+8,fixaddr
	move.l	#$00022001,fixval
	move.l	#$3B3B3B3B,($102000).l
	nop
	nop
	chkw	exccnt,2,66
	chkw	lastfmt,$B,67
	move.w	lastssw,d1
	and.l	#$01FF,d1
	chkl	d1,$0105,68		; DF, write, long
	rdd	$22000,d1
	chkl	d1,$3B3B3B3B,69		; the rerun completed the write
	rdd	TBLB+8,d1
	chkl	d1,$00022019,70		; ... and set M
	; a dropped write (DF cleared by the handler)
	move.w	#M_SKIP,mode
	move.l	#$4C4C4C4C,($201000).l
	nop
	nop
	chkw	exccnt,3,71
	rdd	$21000,d1
	chkl	d1,$77777777,72
	move.l	($201000).l,d1		; reads are allowed
	chkl	d1,$77777777,73
	; write protection from an early termination descriptor
	move.l	($400400).l,d1
	move.l	#0,($400400).l
	nop
	nop
	chkw	exccnt,4,74
	; read-modify-write on a protected page
	tas	($201000).l
	nop
	chkw	exccnt,5,75
	move.w	lastssw,d1
	and.l	#$01FF,d1
	chkl	d1,$01D5,76		; DF, RM, read, byte

;================================================================ 7. user mode
	clr.w	exccnt
	move.l	#$3800,a0
	move.l	a0,usp
	move.w	#$0000,sr
	move.l	($204000).l,d1		; user access allowed
	move.l	d1,(udata).l
	move.l	($200000).l,d2		; supervisor only: fault, data from the handler
	trap	#0
	move.l	udata,d1
	chkl	d1,$5A5A5A5A,77
	chkl	d2,$DEADBEEF,78
	chkw	exccnt,1,79
	move.w	lastssw,d1
	and.l	#$0007,d1
	chkl	d1,1,80			; user data
	move.l	($200000).l,d1
	chkl	d1,$11111111,81		; the same page from supervisor mode

;================================================================ 8. function code lookup
	settc	TC_OFF
	setcrp	$7FFF0002,TBLFC
	settc	TC_FCL
	move.l	($100000).l,d1		; supervisor data: the alternate tree
	chkl	d1,$22222222,82
	lea	($100000).l,a0
	moves.l	(a0),d1			; user data: the main tree
	chkl	d1,$11111111,83
	ptestr	#5,($100000).l,#7,a1
	move.l	a1,d1
	chkl	d1,TBLB2,84
	ptestr	#1,($100000).l,#7,a1
	move.l	a1,d1
	chkl	d1,TBLB,85
	pmove	mmusr,(scr2).l
	chkw	scr2,$0003,86		; function code table, A, B
	ptestr	#3,($100000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0401,87		; no tree for FC 3

;================================================================ 9. supervisor root pointer
	settc	TC_OFF
	setcrp	$7FFF0002,TBLA
	move.l	#$7FFF0002,(scr).l
	move.l	#TBLA2,(scr+4).l
	pmove	(scr).l,srp
	settc	TC_SRE
	move.l	($100000).l,d1		; supervisor: the SRP tree
	chkl	d1,$22222222,88
	moves.l	(a0),d1			; user: the CRP tree
	chkl	d1,$11111111,89
	settc	TC_BASE
	move.l	($100000).l,d1
	chkl	d1,$11111111,90

;================================================================ 10. transparent translation
	settt0	$50008107		; $50xxxxxx, all function codes, reads and writes
	move.l	($50820000).l,d1	; untranslated: region 8 of the physical space
	chkl	d1,$11111111,91
	ptestr	#5,($50820000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$0040,92		; T
	settt0	0
	move.w	#M_SKIP,mode
	clr.w	exccnt
	move.l	($50820000).l,d1	; through the tree: region 8 is not mapped
	chkw	exccnt,1,93
	chkl	d1,$DEADBEEF,94
	settt0	$50008207		; reads only
	move.l	($50820000).l,d1
	chkl	d1,$11111111,95
	move.l	#$12121212,($50820000).l
	nop
	nop
	chkw	exccnt,2,96
	rdd	$20000,d1
	chkl	d1,$11111111,97		; the write was dropped
	settt0	$50008110		; user data only
	lea	($50820000).l,a0
	moves.l	(a0),d1
	chkl	d1,$11111111,98
	move.l	(a0),d1			; supervisor data goes through the tree
	chkw	exccnt,3,99
	settt0	$50008507		; cache inhibited
	move.l	CIOCNT,d4
	move.l	($50820000).l,d1
	move.l	($50820004).l,d1
	move.l	CIOCNT,d5
	sub.l	d4,d5
	chkl	d5,3,100		; two reads plus the first counter read (the register page is CI)
	settt0	$50008107
	move.l	CIOCNT,d4
	move.l	($50820000).l,d1
	move.l	($50820004).l,d1
	move.l	CIOCNT,d5
	sub.l	d4,d5
	chkl	d5,1,101
	settt0	0

;================================================================ 11. MMUDIS
	cclr
	move.w	#1,PINREG
	nop
	nop
	nop
	move.l	($100000).l,d1		; untranslated: physical $100000 is the vector table
	chkl	d1,$00003400,102
	move.w	#0,PINREG
	nop
	nop
	nop
	cclr
	move.l	($100000).l,d1
	chkl	d1,$11111111,103

;================================================================ 12. operands crossing a page boundary
	move.w	#M_FIX,mode
	clr.w	exccnt
	setl	TBLB+4,0		; $101000 invalid again
	newmap
	move.l	#TBLB+4,fixaddr
	move.l	#$00021001,fixval
	move.l	($100FFE).l,d1		; first half from $220FFE, the second faults
	chkl	d1,$CCDD7777,104
	chkw	exccnt,1,105
	move.l	lastfa,d1
	chkl	d1,$101000,106
	move.w	lastssw,d1
	and.l	#$0030,d1
	chkl	d1,$0020,107		; two bytes remained
	setl	TBLB+4,0
	newmap
	move.l	#$1234ABCD,($100FFE).l
	nop
	nop
	chkw	exccnt,2,108
	rdd	$20FFC,d1
	chkl	d1,$AAAA1234,109
	rdd	$21000,d1
	chkl	d1,$ABCD7777,110

;================================================================ 13. instruction fetches
	jsr	($106000).l
	chkl	d0,$C0DE0006,111
	setl	TBLB+24,0		; unmap the code page
	newmap
	move.l	#TBLB+24,fixaddr
	move.l	#$00026001,fixval
	clr.w	exccnt
	moveq	#0,d0
	jsr	($106000).l
	chkl	d0,$C0DE0006,112
	chkw	exccnt,1,113
	chkw	lastfmt,$B,114
	move.w	lastssw,d1
	and.l	#$8007,d1
	chkl	d1,$8006,115		; stage C faulted, supervisor program
	; prefetching beyond the end of a page into an unmapped one is harmless
	clr.w	exccnt
	move.w	#M_FAIL,mode
	lea	pf_back(pc),a1
	jmp	($107FF4).l		; five NOPs, then JMP (A1) in the last word of the page
pf_back:
	chkw	exccnt,0,116

;================================================================ 14. three levels, 256-byte pages
	settc	TC_OFF
	setcrp	$7FFF0002,TBL3A
	settc	TC_3L
	move.l	($100000).l,d1
	chkl	d1,$31313131,117
	move.l	($100100).l,d1
	chkl	d1,$43434444,118
	move.l	($1000FE).l,d1		; two pages
	chkl	d1,$42424343,119
	move.l	($105234).l,d1		; early termination at level B, contiguous
	chkl	d1,$34343434,120
	ptestr	#5,($105234).l,#7,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$0002,121
	move.l	a1,d1
	chkl	d1,TBL3B+4,122
	ptestr	#5,($100200).l,#7,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$0403,123		; invalid at level C
	move.l	a1,d1
	chkl	d1,TBL3C+8,124

;================================================================ 15. limits
	settc	TC_OFF
	setcrp	$7FFF0003,TBLAL
	settc	TC_BASE
	move.w	#M_SKIP,mode
	clr.w	exccnt
	move.l	($104000).l,d1		; index 4: within the limit
	chkl	d1,$5A5A5A5A,125
	move.l	($105000).l,d1		; index 5: violation
	chkw	exccnt,1,126
	ptestr	#5,($105000).l,#7,a1
	pmove	mmusr,(scr2).l
	chkw	scr2,$4401,127		; L, I, one descriptor fetched
	move.l	a1,d1
	chkl	d1,TBLAL+8,128
	setl	TBLAL+8,$80020002	; lower limit 2
	newmap
	move.l	($100000).l,d1
	chkw	exccnt,2,129
	move.l	($104000).l,d1
	chkl	d1,$5A5A5A5A,130
	; the root pointer limit
	setcrp	$00050003,TBLAL
	ptestr	#5,($600000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$4400,131		; L, I, nothing fetched
	ptestr	#5,($200000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$0002,132
	setcrp	$7FFF0003,TBLAL
	; the limit of a long early termination descriptor
	setl	TBLAL+0,$00030001	; region 0: pages 0-3 only
	newmap
	move.l	($4000).l,d1
	chkw	exccnt,3,133
	ptestr	#5,($4000).l,#7
	pmove	mmusr,(scr2).l
	chkw	scr2,$4401,134
	setl	TBLAL+0,$7FFF0001
	newmap

;================================================================ 16. more pages than ATC entries
	settc	TC_OFF
	setcrp	$7FFF0002,TBLA
	settc	TC_BASE
	move.w	#M_FAIL,mode
	lea	$30000,a0
	move.l	#$A0000000,d1
	moveq	#23,d0
fill1:	move.l	d1,(a0)
	add.l	#$1000,a0
	addq.l	#1,d1
	dbf	d0,fill1
	lea	($110000).l,a0
	move.l	#$A0000000,d1
	moveq	#23,d0
chk1:	cmp.l	(a0),d1
	bne.s	atcfail
	add.l	#$1000,a0
	addq.l	#1,d1
	dbf	d0,chk1
	lea	($110000).l,a0		; again: entries evicted and reloaded
	move.l	#$A0000000,d1
	moveq	#23,d0
chk2:	cmp.l	(a0),d1
	bne.s	atcfail
	add.l	#$1000,a0
	addq.l	#1,d1
	dbf	d0,chk2
	bra.s	atcok
atcfail:
	failt	135
atcok:

;================================================================ 17. bus error during the search
	move.w	#M_SKIP,mode
	clr.w	exccnt
	move.l	($700000).l,d1
	chkw	exccnt,1,136
	chkl	d1,$DEADBEEF,137
	ptestr	#5,($700000).l,#0
	pmove	mmusr,(scr2).l
	chkw	scr2,$8400,138		; the entry with B set

;================================================================ 18. cache inhibit from the descriptor
	move.w	#M_FAIL,mode
	move.l	#$1111,d0		; write allocation off: alias writes leave the cache alone
	movec	d0,cacr
	setl	$2A000,$C0C0C0C0
	move.l	($62A000).l,d1		; cachable region
	chkl	d1,$C0C0C0C0,139
	setl	$2A000,$C1C1C1C1	; memory changes behind the cache
	move.l	($62A000).l,d1
	chkl	d1,$C0C0C0C0,140	; hit: stale
	move.l	($52A000).l,d1		; cache inhibited region: memory
	chkl	d1,$C1C1C1C1,141
	setl	$2A000,$C2C2C2C2
	move.l	($52A000).l,d1
	chkl	d1,$C2C2C2C2,142	; never cached
	move.l	CIOCNT,d4
	move.l	($52A004).l,d1
	move.l	CIOCNT,d5
	sub.l	d4,d5
	chkl	d5,2,143		; CIOUT for the CI page and the register read
	move.l	#$3111,d0
	movec	d0,cacr

	settc	TC_OFF
	move.w	#$600D,(DONEREG).l
	stop	#$2700

;---------------------------------------------------------------- code copied to a mapped page
code_page:
	move.l	#$C0DE0006,d0
	rts
	nop
	nop
	nop
	nop
	nop

;---------------------------------------------------------------- handlers
; frame: 0(a6) SR, 2(a6) PC, 6(a6) format/vector, $A(a6) SSW, $10(a6) fault address
h_buserr:
	movem.l	d6/a5/a6,-(sp)
	move.l	sp,a6
	lea	12(a6),a6
	record2
	move.w	$A(a6),lastssw
	move.l	$10(a6),lastfa
	move.w	mode,d6
	cmp.w	#M_FIX,d6
	bne.s	hb1
	move.l	fixaddr,a5
	move.l	fixval,(a5)
	pflusha
	bra.s	hbx
hb1:	cmp.w	#M_SKIP,d6
	bne.s	hb2
	move.w	6(a6),d6
	and.w	#$F000,d6
	cmp.w	#$B000,d6
	bne.s	hb1a
	move.l	#$DEADBEEF,$2C(a6)	; data input buffer
hb1a:	andi.w	#$FEFF,$A(a6)		; DF cleared: no rerun
	bra.s	hbx
hb2:	cmp.w	#M_RTE,d6
	beq.s	hbx
	move.w	lastvec,d7
	or.w	#$F000,d7
	bra	fail_all
hbx:	movem.l	(sp)+,d6/a5/a6
	rte

h_mmucfg:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	movem.l	(sp)+,d6/a6
	rte

h_trap0:
	ori.w	#$2000,(sp)	; return in supervisor mode
	rte

h_unexp:
	move.w	6(sp),d7
	and.w	#$0FFF,d7
	lsr.w	#2,d7
	or.w	#$FF00,d7
	bra	fail_all

fail_all:
	move.l	#TC_OFF,(scr).l	; report with translation off
	pmove	(scr).l,tc
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1
