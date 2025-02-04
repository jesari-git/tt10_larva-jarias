
######################################	
###   	   ROM Bootloader
###	       J. Arias (2024)
######################################

.section .text	

start:
	lui		x3,0xE0000	// I/O area
	li		x4,0x4c
1:	lbu 	x5,0(x3)
	bne		x4,x5, 1b
	sb		x5,0(x3)
	call	getw
	mv		x7,x5
	call	getw
	add		x8,x7,x5
	call	getw
	mv		x9,x5
	j		3f
	
2:	lbu 	x5,0(x3)
	sb		x5,0(x7)
	addi	x7,x7,1
3:	bne		x7,x8,2b	

	jr		x9

getw: // reads 32-bit (4-byte) data into x5, overwrite x6
	lbu 	x5,0(x3)
	lbu 	x6,0(x3)
	slli 	x6,x6,8
	or		x5,x5,x6
	lbu 	x6,0(x3)
	slli 	x6,x6,16
	or		x5,x5,x6
	lbu 	x6,0(x3)
	slli 	x6,x6,24
	or		x5,x5,x6
	ret

