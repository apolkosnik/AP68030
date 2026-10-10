; AP68030 coprocessor interface self test (UM Section 10)
; assembled with vasmm68k_mot -Fbin -m68030
;
; The testbench answers CPU space type 2 accesses for CpID 1 with a scripted
; coprocessor (tb_cp_model.svh): the command word selects the sequence of
; response primitives, the condition word's bit 0 is the true/false answer.
; The model's state is visible through test registers.  The coprocessor
; instructions are written as data words since the assembler knows no
; coprocessor 1.
;
; protocol with the testbench:
;   word write to $F00100 = failing test number
;   word write to $F00102 = $BAD0 on failure, $600D when all tests passed

FAILREG	equ	$F00100
DONEREG	equ	$F00102
EAREG	equ	$F0017C		; last evaluated effective address received
SAVEFMT	equ	$F00180		; save CIR format word
OPREG0	equ	$F00188		; operands received
OPREG1	equ	$F0018C

; scratch variables
lastvec	equ	$3000		; word: vector number of the last exception
lastfmt	equ	$3002		; word: frame format
lastpc	equ	$3004		; long: stacked PC
lastia	equ	$3008		; long: instruction address (format 2)
exccnt	equ	$300C		; word: exceptions taken
skip	equ	$3010		; long: added to the PC of a format 0 frame
buf	equ	$3100		; 64 bytes
buf2	equ	$3140

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

chkw	macro
	move.w	\1,d6
	and.l	#$FFFF,d6
	cmp.l	#\2,d6
	beq.s	ok\@
	failt	\3
ok\@:
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
	addq.w	#1,exccnt
	endm

	org	0
	dc.l	$3400		; initial ISP
	dc.l	start
	rept	30		; 2-31
	dc.l	h_exc
	endr
	dc.l	h_trap0		; 32
	rept	223		; 33-255
	dc.l	h_exc
	endr

	org	$400
start:
	move.l	#$3111,d0
	movec	d0,cacr
	clr.w	exccnt
	move.l	#0,skip

;================================================================ 1. null: the instruction completes
	dc.w	$F200,$0001
	chkw	exccnt,0,1

;================================================================ 2. evaluate EA and transfer to the coprocessor, operand back to D4
	move.l	#$12345678,(buf).l
	dc.w	$F239,$0002	; cpGEN (buf).l
	dc.l	buf
	dc.w	$F200,$0005	; operand CIR -> D4
	chkl	d4,$12345678,2
	move.l	OPREG0,d1
	chkl	d1,$12345678,3

;================================================================ 3. transfer from the coprocessor to memory
	move.l	#0,(buf2).l
	dc.w	$F239,$0003
	dc.l	buf2
	move.l	buf2,d1
	chkl	d1,$12345678,4

;================================================================ 4. single register D3 to the coprocessor
	move.l	#$D3D3D3D3,d3
	dc.w	$F200,$0004
	dc.w	$F200,$0005
	chkl	d4,$D3D3D3D3,5

;================================================================ 5. operand from the instruction stream
	dc.w	$F200,$0007
	dc.l	$C0FFEE00
	dc.w	$F200,$0005
	chkl	d4,$C0FFEE00,6

;================================================================ 6. status register to the coprocessor
	move.w	#$2715,sr
	dc.w	$F200,$0008
	dc.w	$F200,$0005
	chkl	d4,$27152715,7	; word write: both halves of the port (UM Table 7-5)

;================================================================ 7. evaluate and transfer the effective address
	dc.w	$F239,$000E
	dc.l	$00123456
	move.l	EAREG,d1
	chkl	d1,$00123456,8

;================================================================ 8. write to a previously evaluated address
	dc.w	$F239,$0009
	dc.l	buf2
	move.l	buf2,d1
	chkl	d1,$0BADF00D,9

;================================================================ 9. supervisor check
	dc.w	$F200,$000A
	chkw	exccnt,0,10
	move.l	#4,skip
	move.l	#$3800,a0
	move.l	a0,usp
	move.w	#$0000,sr
ucp:	dc.w	$F200,$000A	; user mode: privilege violation, pre-instruction
	trap	#0
	chkw	exccnt,1,11
	chkw	lastvec,8,12
	chkw	lastfmt,0,13
	move.l	lastpc,d1
	chkl	d1,ucp,14

;================================================================ 10. control register (MSP, code $803 from the register select CIR)
	move.l	#$00AA5500,d0
	movec	d0,msp
	dc.w	$F200,$000B
	dc.w	$F200,$0005
	chkl	d4,$00AA5500,15

;================================================================ 11. multiple main processor registers (mask $0803: D0, D1, A3)
	move.l	#$0D000000,d0
	move.l	#$0D111111,d1
	move.l	#$0A333333,a3
	dc.w	$F200,$000F
	move.l	OPREG0,d2
	chkl	d2,$0D000000,16
	move.l	OPREG1,d2
	chkl	d2,$0D111111,17

;================================================================ 12. busy: the instruction restarts and completes
	dc.w	$F200,$0010
	chkw	exccnt,1,18

;================================================================ 12b. busy with an interrupt pending
; the interrupt is serviced with a pre-instruction frame: its PC is the
; coprocessor instruction, which restarts after RTE (UM 10.4.3)
	clr.w	exccnt
	move.w	#$2000,sr
	move.w	#3,$F001BC		; level 3 at the command write, busy again
bsy:	dc.w	$F200,$0010
	chkw	exccnt,1,68
	chkw	lastvec,27,69
	move.l	lastpc,d1
	chkl	d1,bsy,70
	move.w	#$2700,sr

;================================================================ 12c. come again (IA) with an interrupt pending
; the interrupt is serviced with a coprocessor mid-instruction frame (format
; $9: PC = the next instruction, instruction address = this one); its RTE
; reads the response CIR again and the instruction completes (UM 10.4.8)
	clr.w	exccnt
	clr.l	lastia
	lea	buf,a0
	clr.l	(a0)
	move.w	#$2000,sr
	move.w	#3,$F001BC		; level 3 at the command write
cag:	dc.w	$F210,$0011		; come again x3, then 4 bytes to (a0)
cagn:	chkw	exccnt,1,71
	chkw	lastvec,27,72
	chkw	lastfmt,9,73
	move.l	lastpc,d1
	chkl	d1,cagn,74
	move.l	lastia,d1
	chkl	d1,cag,75
	move.l	buf,d1
	chkl	d1,$11223344,76
	move.w	#$2700,sr

;================================================================ 13. exceptions requested by the coprocessor
	clr.w	exccnt
pre1:	dc.w	$F200,$0006	; pre-instruction, vector 48
	chkw	exccnt,1,19
	chkw	lastvec,48,20
	chkw	lastfmt,0,21
	move.l	lastpc,d1
	chkl	d1,pre1,22
post1:	dc.w	$F200,$000C	; post-instruction, vector 49
post1n:
	chkw	exccnt,2,23
	chkw	lastvec,49,24
	chkw	lastfmt,2,25
	move.l	lastpc,d1
	chkl	d1,post1n,26
	move.l	lastia,d1
	chkl	d1,post1,27
fl1:	dc.w	$F200,$0055	; unknown command: F-line
	chkw	exccnt,3,28
	chkw	lastvec,11,29
	move.l	lastpc,d1
	chkl	d1,fl1,30

;================================================================ 14. transfer to the top of the stack
	move.l	sp,a5
	dc.w	$F200,$000D
	move.l	sp,d1
	sub.l	a5,d1
	chkl	d1,-4,31
	move.l	(sp)+,d1
	chkl	d1,$0D000000,32	; the operand CIR holds D0 of test 11

;================================================================ 15. cpBcc
	moveq	#0,d5
	dc.w	$F281,$0004	; true: branch over the MOVEQ
	moveq	#1,d5
	chkl	d5,0,33
	moveq	#0,d5
	dc.w	$F280,$0004	; false: not taken
	moveq	#1,d5
	chkl	d5,1,34
	moveq	#0,d5
	dc.w	$F2C1		; 32-bit displacement
	dc.l	$00000006
	moveq	#1,d5
	chkl	d5,0,35
	bra.s	cpb1
cpb2:	moveq	#2,d5
	bra.s	cpb3
cpb1:	dc.w	$F281
	dc.w	cpb2-*		; backwards
cpb3:	chkl	d5,2,36

;================================================================ 16. cpDBcc
	moveq	#3,d2
cpdb:	dc.w	$F24A,$0000	; cpDBcc D2 while false
	dc.w	cpdb-*
	and.l	#$FFFF,d2	; the count is a word
	chkl	d2,$FFFF,37
	moveq	#3,d2
	dc.w	$F24A,$0001	; true: no decrement, no branch
	dc.w	$0002
	chkl	d2,3,38

;================================================================ 17. cpScc
	moveq	#0,d2
	dc.w	$F242,$0001	; cpScc D2
	and.l	#$FF,d2
	chkl	d2,$FF,39
	dc.w	$F242,$0000
	and.l	#$FF,d2
	chkl	d2,0,40
	dc.w	$F279,$0001	; cpScc (buf2).l
	dc.l	buf2
	move.b	buf2,d2
	and.l	#$FF,d2
	chkl	d2,$FF,41
; cpScc Dn changes only the low byte of the current Dn (UM 10.2.2.2): after
; the coprocessor loaded Dn during the dialog, and after a mid-instruction
; frame (interrupt during come again) whose handler and RTE ran in between
	move.l	#$AABBCC00,d2
	dc.w	$F242,$0021	; selector bit 5: D2 <- $11223344, then true
	chkl	d2,$112233FF,77
	move.l	#$AABBCC00,d2
	move.l	#$12345678,d7	; a different value in the handler's registers
	clr.w	exccnt
	move.w	#$2000,sr
	move.w	#3,$F001BC	; level 3 at the condition write
	dc.w	$F242,$0011	; selector bit 4: come again (IA) once, then true
	move.w	#$2700,sr
	chkw	exccnt,1,78
	chkw	lastfmt,9,79
	chkl	d2,$AABBCCFF,80

;================================================================ 18. cpTRAPcc
	move.l	#0,skip
	clr.w	exccnt
cpt1:	dc.w	$F27A,$0001,$1234	; word operand, true: vector 7
cpt1n:	chkw	exccnt,1,42
	chkw	lastvec,7,43
	chkw	lastfmt,2,44
	move.l	lastpc,d1
	chkl	d1,cpt1n,45
	move.l	lastia,d1
	chkl	d1,cpt1,46
	dc.w	$F27B,$0000	; long operand, false: skipped
	dc.l	$00000000
	chkw	exccnt,1,47
	dc.w	$F27C,$0001	; no operand, true
	chkw	exccnt,2,48

;================================================================ 19. cpSAVE and cpRESTORE
	move.w	#$0000,SAVEFMT	; empty/reset format
	lea	buf+16,a0
	dc.w	$F320		; cpSAVE -(a0)
	move.l	a0,d1
	sub.l	#buf+12,d1
	chkl	d1,0,49		; four bytes
	move.w	(a0),d1
	and.l	#$FFFF,d1
	chkl	d1,0,50
	move.w	#$0108,SAVEFMT	; not ready once, then format $F1 with 8 bytes of state
	lea	buf+32,a0
	dc.w	$F320
	move.l	a0,d1
	sub.l	#buf+20,d1
	chkl	d1,0,51		; 4 + 8 bytes
	move.w	(a0),d1
	and.l	#$FFFF,d1
	chkl	d1,$F108,52
	move.l	4(a0),d1
	chkl	d1,$0D000000,53	; the operand CIR, twice
	move.l	8(a0),d1
	chkl	d1,$0D000000,54
	move.l	#$5E5E0001,4(a0)
	move.l	#$5E5E0002,8(a0)
	dc.w	$F358		; cpRESTORE (a0)+
	move.l	a0,d1
	sub.l	#buf+32,d1
	chkl	d1,0,55
	move.l	OPREG0,d1
	chkl	d1,$5E5E0001,56
	move.l	OPREG1,d1
	chkl	d1,$5E5E0002,57
	lea	buf+12,a0
	dc.w	$F358		; the empty frame
	move.l	a0,d1
	sub.l	#buf+16,d1
	chkl	d1,0,58
	move.l	#2,skip		; a one-word instruction
	move.w	#$0200,(buf).l	; invalid format: format error, pre-instruction
	lea	buf,a0
cpr1:	dc.w	$F350		; cpRESTORE (a0)
	chkw	exccnt,3,59
	chkw	lastvec,14,60
	move.l	lastpc,d1
	chkl	d1,cpr1,61

;================================================================ 20. transfers with other addressing modes
	lea	buf,a0
	move.l	#$A0A0A0A0,(a0)
	dc.w	$F218,$0002	; (a0)+ to the coprocessor
	move.l	a0,d1
	sub.l	#buf+4,d1
	chkl	d1,0,62
	dc.w	$F200,$0005
	chkl	d4,$A0A0A0A0,63
	lea	buf+4,a0
	move.l	#0,(buf).l
	dc.w	$F220,$0003	; from the coprocessor to -(a0)
	move.l	buf,d1
	chkl	d1,$A0A0A0A0,64
	move.l	a0,d1
	sub.l	#buf,d1
	chkl	d1,0,65
	move.l	#$D5D5D5D5,d5
	dc.w	$F205,$0002	; data register direct
	dc.w	$F200,$0005
	chkl	d4,$D5D5D5D5,66
	dc.w	$F23C,$0002	; immediate
	dc.l	$1DEA1DEA
	dc.w	$F200,$0005
	chkl	d4,$1DEA1DEA,67

	move.w	#$600D,(DONEREG).l
	stop	#$2700

;---------------------------------------------------------------- handlers
; frame: 0(a6) SR, 2(a6) PC, 6(a6) format/vector, 8(a6) instruction address (format 2)
h_exc:
	movem.l	d6/a6,-(sp)
	move.l	sp,a6
	addq.l	#8,a6
	record2
	move.w	6(a6),d6
	and.w	#$F000,d6
	beq.s	hx_fmt0
	cmp.w	#$9000,d6
	beq.s	hx_fmt9
	cmp.w	#$2000,d6
	bne.s	hx_out
	move.l	8(a6),lastia
	bra.s	hx_out
hx_fmt9:
	move.l	8(a6),lastia	; coprocessor mid-instruction: instruction address
	move.w	6(a6),d6
	and.w	#$0FFF,d6
	cmp.w	#24*4,d6
	blo.s	hx_out
	cmp.w	#31*4,d6
	bhi.s	hx_out
	move.w	#0,$F00110	; an interrupt during come-again: release it
	bra.s	hx_out
hx_fmt0:
	move.w	6(a6),d6	; an interrupt: release the request
	and.w	#$0FFF,d6
	cmp.w	#24*4,d6
	blo.s	hx_skip
	cmp.w	#31*4,d6
	bhi.s	hx_skip
	move.w	#0,$F00110
	bra.s	hx_out
hx_skip:
	move.l	skip,d6		; pre-instruction frame: step over the instruction
	add.l	d6,2(a6)
hx_out:
	movem.l	(sp)+,d6/a6
	rte

h_trap0:
	ori.w	#$2000,(sp)	; return in supervisor mode
	rte

fail_all:
	move.w	d7,(FAILREG).l
	move.w	#$BAD0,(DONEREG).l
halt1:
	bra.s	halt1
