
######################################	
###        Header
######################################
.section .text	# 16 bytes below code area

	.word	0x4CFFFFFF
	.word	start
	.word	_edata-start
	.word	start
######################################	
###        Main program
######################################

start:
	lui	sp,0x20020	# SP at RAM top (decrement before store)
# copy data section (only if code in ROM)
#	la 	a0, _sdata
#	la 	a1, _sdata_values
#	la 	a2, _edata
#	bge	a0, a2, end_init_data
#loop_init_data:
#	lw	a3,0(a1)
#	sw	a3,0(a0)
#	addi	a0,a0,4
#	addi	a1,a1,4
#	blt	a0, a2, loop_init_data
#end_init_data:
# zero-init bss section
	la 	a0, _sbss
	la 	a1, _ebss
	bge	a0, a1, end_init_bss
loop_init_bss:
	sw 	zero, 0(a0)
	addi 	a0, a0, 4
	blt 	a0, a1, loop_init_bss
end_init_bss:
# call main
	call 	main
loop:
	j 	loop
	
	.globl	delay_loop		// (3 + 3*A0) cycles
delay_loop:
	addi	a0,a0,-1
	bnez	a0, delay_loop
	ret

