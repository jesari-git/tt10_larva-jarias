# tools
TOOL = arm-none-eabi

CC      = $(TOOL)-gcc
LD      = $(TOOL)-ld -v
AR      = $(TOOL)-ar
AS      = $(TOOL)-as
CP      = $(TOOL)-objcopy
OD		= $(TOOL)-objdump

BT2=~/UVA/Cosas_Nuestras/bootloader/bt2 -sb 115200
ISP=~/UVA/Cosas_Nuestras/lpc21isp-master/lpc21isp

# compilado en modo thumb
#CFLAGS  = -I./ -fno-common -Os -g -gdwarf-2 -mcpu=arm7tdmi -mthumb -mthumb-interwork -nostartfiles -static
# compilado en modo ARM
CFLAGS  = -I./ -fno-common -Os -g -gdwarf-2 -mcpu=arm7tdmi -nostartfiles -static

all:	code.bin coderom.elf
	$(BT2) -l code.bin

code.bin:	code.elf
	$(CP) -O binary $< $@

code.elf: crt.S main.c clock.h minilib.h linker_ram.ld Makefile
	$(CC) $(CFLAGS) -Wl,-Tlinker_ram.ld -o $@ -DCRLF crt.S main.c

coderom.bin:	coderom.elf
	$(CP) -O binary $< $@

coderom.elf:	crt.S main.c clock.h minilib.h linker_flash.ld Makefile
	$(CC) $(CFLAGS) -Wl,-Tlinker_flash.ld -o $@ -DCRLF crt.S main.c


burn:	coderom.bin
	$(ISP) -control -bin $< /dev/ttyUSB0 115200 14746
	$(BT2)

term:
	$(BT2)

dis:	coderom.elf
	$(OD) -d $< >coderom.lst

clean:
	-rm -f *~ *.lst *.bin a.map *.elf 


