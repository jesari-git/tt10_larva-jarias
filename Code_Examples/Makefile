##-- Riscv toolchain path
#RISCV = /opt/riscv32i/bin
RISCV = riscv64-unknown-elf
prog: code.bin debug.bin bload.bin

code.elf: sections.lds start.S main.c printf.c Makefile tv.c sieve.c
	$(RISCV)-gcc -march=rv32e -mabi=ilp32e \
	-Wl,-Bstatic,-T,sections.lds,--strip-debug,-Map=code.map \
	-ffreestanding -nostdlib \
	-Os -o $@ start.S main.c -lgcc
	$(RISCV)-objdump -M numeric -d $@ >code.lst
	$(RISCV)-size $@

code.bin: code.elf
	$(RISCV)-objcopy -O binary $< $@
	
debug.elf: sections_debug.lds start.S debug.c printf.c dissaRVE.c Makefile
	$(RISCV)-gcc -march=rv32e -mabi=ilp32e \
	-Wl,-Bstatic,-T,sections_debug.lds,--strip-debug,-Map=debug.map \
	-ffreestanding -nostdlib \
	-Os -o $@ start.S debug.c -lgcc
	$(RISCV)-objdump -M numeric -d $@ >debug.lst
	$(RISCV)-size $@

debug.bin: debug.elf
	$(RISCV)-objcopy -O binary $< $@
	
dis:	code.elf
	$(RISCV)-objdump -d $< >code.lst


bload.bin:	bload.S
	$(RISCV)-gcc -march=rv32e -mabi=ilp32e \
	-Wl,-Bstatic,-T,sections_brom.lds,--strip-debug \
	-ffreestanding -nostdlib \
	-Os -o $@ bload.S
	$(RISCV)-objdump -M numeric -d $@ >bload.lst

# ---- Clean ----

clean:
	rm -f *.elf *.bin *~ *.map

.PHONY: prog
