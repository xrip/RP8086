ideal
p8086
model	tiny

warn
;---------------------------------------------------------------------------------------------------
; Macros
;---------------------------------------------------------------------------------------------------
; Pad code to create entry point at specified address (needed for 100% IBM BIOS compatibility)
macro	entry	addr
	pad = str_banner - $ + addr - 0E000h
	if pad lt 0
		err	'No room for ENTRY point'
	endif
	if pad gt 0
		db	pad dup(0F4h)
	endif
endm

; Force far jump
macro	jmpfar	segm, offs
	db	0EAh;
	dw	offs, segm
endm

segment	code

	org	0E000h				; 8K ROM BIOS starts at F000:E000
	str_banner	db	'i8086 BIOS', 0
;---------------------------------------------------------------------------------------------------
; BIOS Power-On Self Test (POST)
;---------------------------------------------------------------------------------------------------
entry	0E05Bh				; IBM restart entry point
proc	post	near

warm_boot:					; Entered by POWER_ON/RESET
    cli
    	mov	ax, 30h 			; Set up IBM-compatible stack
    	mov	ss, ax				;   segment 0030h
    	mov	sp, 100h			;   offset  0100h

    mov [0x20], 0FEA5h
    mov [0x22], 0F000h
    sti ; enable ints

lck: jmp lck
nop
nop
nop
endp post


; Interrupt 8h - Hardware Clock
;---------------------------------------------------------------------------------------------------
entry	0FEA5h				; IBM entry, hardware clock
proc	int_8	far
;	sti					; Routine services clock tick
;	push	ds
;	push	dx
	push	ax

	mov	al, 55h 			;   send end_of_interrupt
	out	20h, al 			;   to 8259 interrupt chip
	pop	ax
;	pop	dx
	;pop	ds
	iret

endp	int_8

;--------------------------------------------------------------------------------------------------
; Power-On Entry Point
;--------------------------------------------------------------------------------------------------
	entry	0FFF0h				; Hardware power reset entry
proc	power	far				;   CPU begins here on power up

	jmpfar	0F000h, warm_boot

endp	power


;--------------------------------------------------------------------------------------------------
; BIOS Release Date and Signature
;--------------------------------------------------------------------------------------------------
	entry	0FFF5h
date	db	"10/28/17", 0			; Release date (MM/DD/YY)
						;   originally 08/23/87
	entry	0FFFEh
ifdef	IBM_PC
	db	0FFh				; Computer type (PC)
else
	db	0FEh				; Computer type (XT)
endif
;	db	0				; Checksum byte (8K ROM must sum to 0 mod 256)

ends	code

end
