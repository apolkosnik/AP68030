; AP68030 MMU instruction encoding self test
; assembled with vasmm68k_mot -Fbin -m68030 -m68851
;
; "All F-line instructions with CP-ID = 0 (including MC68851 instructions)
; that the MC68030 does not support automatically cause F-line
; unimplemented instruction exceptions when their execution is attempted in
; the supervisor mode.  If execution of a unimplemented F-line instruction
; with CPID=0 is attempted in the user mode, the MC68030 takes a privilege
; violation exception." (UM 9.8)
; The supported encodings are the formats of the UM 3.3.3 descriptions of
; PFLUSH, PLOAD, PMOVE and PTEST (their zero bits, register and mode fields,
; function code field 10xxx/01ddd/0000x, control alterable EAs).  Where the
; UM does not say, the reserved encodings follow WinUAE (cpummu030.c
; mmu_op30_pmove: R/W=1 with FD; table68k MMUOP030: no PC relative or
; immediate EA field, also for the forms without an operand).
;
; protocol with the testbench:
;   word write to $F100 = failing test number
;   word write to $F102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102

; scratch variables
lastvec	equ	$3800		; word: vector number of the last exception
exccnt	equ	$3802		; word: exceptions taken
resume	equ	$3804		; long: the handler returns here
scr	equ	$3810		; 8 bytes: PMOVE operand
USTK	equ	$3000		; user stack

failt	macro
	move.w	#\1,d7
	bra	fail_all
	endm

; compare the word at \1 against \2, fail with number \3
chkw	macro
	moveq	#0,d7
	move.w	\1,d7
	cmp.l	#\2,d7
	beq.s	ok\@
	failt	\3
ok\@:
	endm

; run the two-word encoding \2,\3 (supervisor mode unless \5 = user): it
; must take exactly one exception with vector \4; test number \1
enc	macro
	move.l	#r\@,resume
	clr.w	lastvec
	clr.w	exccnt
	lea	scr,a0
	clr.l	(a0)
	clr.l	4(a0)		; a TC image with E = 0: translation stays off
	ifc	"\5","user"
	move.w	#$0700,sr
	endif
	dc.w	\2,\3
	nop
	nop
r\@:	chkw	exccnt,1,\1
	chkw	lastvec,\4,\1+100
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	rept	254
	dc.l	h_any
	endr

	org	$400
start:
	move.w	#$2700,sr
	moveq	#0,d0
	movec	d0,cacr
	lea	USTK,a1
	move.l	a1,usp

;---------------------------------------------------------------- supported forms: no exception
	clr.w	exccnt
	lea	scr,a0
	clr.l	(a0)
	clr.l	4(a0)
	dc.w	$F000,$2400	; pflusha
	dc.w	$F000,$30F5	; pflush #5,#7
	dc.w	$F010,$38F5	; pflush #5,#7,(a0)
	dc.w	$F010,$4200	; pmove tc,(a0)
	dc.w	$F010,$4000	; pmove (a0),tc
	dc.w	$F010,$6200	; pmove mmusr,(a0)
	dc.w	$F010,$0A00	; pmove tt0,(a0)
	clr.l	(a0)
	dc.w	$F010,$0D00	; pmovefd (a0),tt1
	dc.w	$F010,$8215	; ptestr #5,(a0),#0
	chkw	exccnt,0,1

;---------------------------------------------------------------- unsupported in supervisor mode: F-line
	; PMOVE: the low byte of the second word is zero (UM 3.3.3 PMOVE formats)
	enc	2,$F010,$4201,11	; pmove tc,(a0) + low byte
	enc	3,$F010,$4001,11	; pmove (a0),tc + low byte
	enc	4,$F010,$0A80,11	; pmove tt0,(a0) + low byte
	; PMOVE MMUSR has no FD bit (UM 3.3.3 PMOVE format for the MMUSR)
	enc	5,$F010,$6300,11	; pmove mmusr,(a0) with FD
	enc	6,$F010,$6100,11	; pmove (a0),mmusr with FD
	; PMOVE MRn,<ea> with FD (WinUAE mmu_op30_pmove: "read and fd set")
	enc	7,$F010,$4300,11	; pmove tc,(a0) with FD
	enc	8,$F010,$0B00,11	; pmove tt0,(a0) with FD
	; PLOAD: bits 8-5 are zero (UM 3.3.3 PLOAD format)
	enc	9,$F010,$2235,11	; ploadr #5,(a0) + bit 5
	enc	10,$F010,$2115,11	; ploadw #5,(a0) + bit 8
	; PFLUSH: bits 9-8 are zero (UM 3.3.3 PFLUSH format)
	enc	11,$F000,$31F5,11	; pflush #5,#7 + bit 8
	enc	12,$F010,$3AF5,11	; pflush #5,#7,(a0) + bit 9
	; function code field 11xxx (UM 3.3.3: 10xxx, 01ddd, 00000, 00001 only)
	enc	13,$F010,$861D,11	; ptestr fc 11101,(a0),#1
	enc	14,$F000,$3018,11	; pflush fc 11000,#0
	enc	15,$F010,$2218,11	; ploadr fc 11000,(a0)
	; PC relative or immediate EA field (WinUAE table68k MMUOP030), also
	; for the forms that take no operand
	enc	16,$F03A,$2400,11	; pflusha, EA field (d16,PC)
	enc	17,$F03C,$30F5,11	; pflush #5,#7, EA field #imm
	enc	18,$F03A,$4200,11	; pmove tc,(d16,pc)
	; other CpID 0 types (68851 PScc/PDBcc/PTRAPcc, PBcc, PSAVE, PRESTORE)
	enc	19,$F048,$0000,11
	enc	20,$F080,$0000,11
	enc	21,$F110,$0000,11

;---------------------------------------------------------------- user mode: privilege violation (UM 9.8)
	enc	30,$F010,$4200,8,user	; a supported one
	enc	31,$F010,$4201,8,user	; an unsupported second word
	enc	32,$F048,$0000,8,user	; another CpID 0 type
	enc	33,$F03C,$30F5,8,user	; an immediate EA field

	move.w	#$600D,(DONEREG).l
done:	bra.s	done

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:	bra.s	halt1

;---------------------------------------------------------------- handler
; every exception: count it, record the vector, return to resume in
; supervisor mode with IPL 7 (formats $0 and $2 only occur here)
h_any:
	move.l	d6,-(sp)
	move.w	10(sp),d6
	and.w	#$0FFF,d6
	lsr.w	#2,d6
	move.w	d6,lastvec
	addq.w	#1,exccnt
	move.l	resume,6(sp)
	move.w	#$2700,4(sp)
	move.l	(sp)+,d6
	rte
