; AP68030 coprocessor interface: encodings the main processor rejects.
; Protocol violations (vector 13, mid-instruction frame $9, no abort; the
; RTE reads the response again, UM 10.5.2.1): undefined primitives (bits
; 13:8 = $00, $28-$2B, UM 10.6) and evaluate EA and transfer data with a
; register length other than 1, 2 or 4, an odd immediate length above 1 or
; a write to a non-alterable EA (UM 10.4.9, Table 10-6).  F-line without any
; coprocessor access: cpTRAPcc opmodes other than 010/011/100 and cpScc
; without a data alterable EA (UM Table 10-1, 10.5.2.2).
	include	"asm/t_cpx.i"

; a protocol violation for check numbers \1..\3
proto	macro
	chkw	exccnt,1,\1
	chkw	logvec,13,\2
	chkw	logfmt,9,\3
	endm

start:
	bsr	init
	move.l	#4,skip			; F-line frames: step over two words
;---------------------------------------------------------------- 1. evaluate EA and transfer data, D5, length 3
	clrlog
	script	$9703,$0902
p1:	dc.w	$F205,$0030
	proto	1,2,3
	move.l	logia,d1
	chkl	d1,p1,4
;---------------------------------------------------------------- 2. immediate, length 3
	clrlog
	script	$9703,$0902
	dc.w	$F23C,$0030
	proto	5,6,7
;---------------------------------------------------------------- 3. immediate, written (DR=1)
	clrlog
	script	$B704,$0902
	dc.w	$F23C,$0030
	proto	8,9,10
;---------------------------------------------------------------- 4. control: an EA outside the category is still an F-line with abort
	clrlog
	script	$9104,$0902		; data alterable, immediate EA
f4:	dc.w	$F23C,$0030
	chkw	exccnt,1,11
	chkw	logvec,11,12
	chkw	logfmt,0,13
	move.l	logpc,d1
	chkl	d1,f4,14
	move.w	CTRLREG,d1
	and.w	#3,d1
	chkw	d1,1,15			; the abort mask
;---------------------------------------------------------------- 5. $8004 after an evaluated EA: undefined, not a write to the EA
	clrlog
	lea	buf,a0
	move.l	#$11111111,(a0)
	move.l	#$5A5A5A5A,SCROP
	script	$9704,$8004,$0902
	dc.w	$F210,$0030
	proto	16,17,18
	move.l	buf,d1
	chkl	d1,$11111111,19
;---------------------------------------------------------------- 6. null with DR=1 ($29)
	clrlog
	script	$2902,$0902
	dc.w	$F200,$0030
	proto	20,21,22
;---------------------------------------------------------------- 7. null with CA and DR ($A900)
	clrlog
	script	$A900,$0902
	dc.w	$F200,$0030
	proto	23,24,25
;---------------------------------------------------------------- 8. evaluate and transfer EA with DR=1 ($2A)
	clrlog
	lea	buf,a0
	script	$AA00,$0902
	dc.w	$F210,$0030
	proto	26,27,28
;---------------------------------------------------------------- 9. cpScc mode 7 register 5, cpTRAPcc opmodes 110/111: F-line, no coprocessor access
	move.l	SCRIPT,d5
	clrlog
i1:	dc.w	$F27D,$0001
	chkw	exccnt,1,29
	chkw	logvec,11,30
	chkw	logfmt,0,31
	move.l	logpc,d1
	chkl	d1,i1,32
	clrlog
i2:	dc.w	$F27E,$0001
	chkw	exccnt,1,33
	chkw	logvec,11,34
	clrlog
i3:	dc.w	$F27F,$0001
	chkw	exccnt,1,35
	chkw	logvec,11,36
	clrlog
i4:	dc.w	$F47E,$0001		; CpID 2
	chkw	exccnt,1,37
	chkw	logvec,11,38
	move.l	SCRIPT,d1
	sub.l	d5,d1
	chkl	d1,0,39			; no CPU space cycle at all
;---------------------------------------------------------------- 10. control: the valid neighbours still reach the coprocessor
	clrlog
	dc.w	$F27C,$0001		; cpTRAPcc, no operand, true: trap
	chkw	exccnt,1,40
	chkw	logvec,7,41
	clrlog
	dc.w	$F27A,$0001,$1234	; cpTRAPcc.W, true: trap
	chkw	logvec,7,42
	clr.b	buf
	dc.w	$F278,$0001		; cpScc (abs).W, true
	dc.w	buf
	move.b	buf,d1
	and.l	#$FF,d1
	chkl	d1,$FF,43
	endtest
