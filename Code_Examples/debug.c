#include <stdarg.h>
#include <stdint.h>

//-- Registros mapeados
#define UARTDAT  (*(volatile uint32_t*)0xE0000000)
#define PFLAGS  (*(volatile uint32_t*)0xE0000004)
#define RXVAL	(1)
#define TXRDY	(2)
#define RXFE	(4)
#define RXOVE	(8)
#define CTRLC   (16)
#define DIRTY	(32)
#define PWMIRQ	(64)

#define PWM      (*(volatile uint8_t*)0xE0000020)
#define TCNT     (*(volatile uint32_t*)0xE0000060)

#define INTEN	 (*(volatile uint32_t*)0xE00000E0)

#define ENRX	(1)
#define ENTX	(2)
#define ENPWM	(4)
#define ENSS	(8)
#define ENCTLC	(16)

#define IRQVECT0 (*(volatile uint32_t*)0xE00000F0)
#define IRQVECT1 (*(volatile uint32_t*)0xE00000F4)
#define IRQVECT2 (*(volatile uint32_t*)0xE00000F8)
#define IRQVECT3 (*(volatile uint32_t*)0xE00000FC)

void delay_loop(uint32_t val);	// (3 + 3*val) cycles
#define CCLK (4000000)
#define _delay_us(n) delay_loop((n*(CCLK/1000)-3000)/3000)
#define _delay_ms(n) delay_loop((n*(CCLK/1000)-30)/3)

void _putch(int c)
{
	while((PFLAGS & TXRDY)==0);
	//if (c == '\n') _putch('\r');
	UARTDAT = c;
}

void _puts(const char *p)
{
	while (*p)
		_putch(*(p++));
}

#define putchar(d) _putch(d)
#include "printf.c"

int _getch()
{
	while((PFLAGS & RXVAL)==0);
	return UARTDAT;
}

int _getsn(char *pd, int max)
{
	int d;
	char *pb;
	pb=pd;
	
	while(1) {
		d=_getch();
		if (d=='\n' || d=='\r') {*pd=0; return (int)(pd-pb);}
		if (d=='\b' || d==127) {
			if (pd!=pb) {
				_puts("\b \b");
				pd--;
			}
			continue;
		}
		if ((int)(pd-pb)< (max-1)) {*pd++=d; _putch(d);}
	}
}

int rdhex(char *p)
{
	unsigned int d,a;
	d=0;
	while (1) {
		a=*p++;
		if (a>'Z') a-=32;	// uppercase
		if (a>='0' && a<='9') a-='0';
		else if (a>='A' && a<='F') a-='A'-10;
			else break;
		d<<=4; d+=a;
	}
	return d;
}

void hexdump(char *p)
{
	int i,j;
	
	for (i=0;i<16;i++) {
		_printf("%08x  ",(uint32_t)p);
		for (j=0;j<16;j++) _printf("%02x ",p[j]);
		_putch(' ');
		for (j=0;j<16;j++) _putch((p[j]>=32 && p[j]<127)? p[j] : '.');
		_putch('\n');
		p=&p[j];
	}
}

#include "dissaRVE.c"

void help()
{
	_puts(
	"\nCommands:\n"
	" h:\tHelp\n"
	" c:\tContinue wo breakpoints (fast)\n"
	" e:\tExecute with breakpoints (slow)\n"
	" s:\texecute Single instruction\n"
	" n:\texecute until Next instruction\n"
	" r:\texecute until subroutine Return (any)\n"
	" R:\texecute until Return at higher stack level\n"
	" b:\tset Breakpoint\n"
	" g:\tGoto (change PC)\n"
	" x:\tchange X register\n"
	" d:\tDissasemble\n"
	" m:\tMemory dump\n"
	" w:\tmemory Write\n"
	" l:\tLoad code to debug\n"
	" ctrl-c\tPauses execution\n"
	);
}

void __attribute__((naked)) debug_handler()
{
	asm volatile(
	"	addi	sp,sp,-64		\n"
	"	sw		x1,0(sp)		\n"
	"	addi	x1,sp,64		\n"
	"	sw		x1,4(sp)		\n"	// current SP
	"	sw		x3,8(sp)		\n"
	"	sw		x4,12(sp)		\n"
	"	sw		x5,16(sp)		\n"
	"	sw		x6,20(sp)		\n"
	"	sw		x7,24(sp)		\n"
	"	sw		x8,28(sp)		\n"
	"	sw		x9,32(sp)		\n"
	"	sw		x10,36(sp)		\n"
	"	sw		x11,40(sp)		\n"
	"	sw		x12,44(sp)		\n"
	"	sw		x13,48(sp)		\n"
	"	sw		x14,52(sp)		\n"
	"	sw		x15,56(sp)		\n"
	"	csrrw	x1,0x341,zero	\n"	// Interrupted PC
	"	sw		x1,60(sp)		\n"
	"	mv		a0,sp			\n"
	"	call	singlestep		\n"
	"	lw		x1,60(sp)		\n"
	"	csrrw	zero,0x341,x1	\n"	// allows PC changes
	"	lw		x1,0(sp)		\n"
	"	lw		x3,8(sp)		\n"
	"	lw		x4,12(sp)		\n"
	"	lw		x5,16(sp)		\n"
	"	lw		x6,20(sp)		\n"
	"	lw		x7,24(sp)		\n"
	"	lw		x8,28(sp)		\n"
	"	lw		x9,32(sp)		\n"
	"	lw		x10,36(sp)		\n"
	"	lw		x11,40(sp)		\n"
	"	lw		x12,44(sp)		\n"
	"	lw		x13,48(sp)		\n"
	"	lw		x14,52(sp)		\n"
	"	lw		x15,56(sp)		\n"
	"	lw		sp,4(sp)		\n" // Allows SP changes
	"	mret					\n"
	);
}
void load();
// Single-Step
void singlestep(uint32_t *stack){
	int i,j;
	uint32_t *pc;
	char buf[12];
	static uint8_t notrace,stopjalr;
	static uint32_t tbrk,brk,dadd,madd,wadd,stacklevel;
	
	pc=(uint32_t *)stack[15];
	
	// Trap vector also points here during debug
	if ((pc[-1]&(~(1<<20)))==0x73) {		// Was ECALL or EBREAK ?
		_puts("\n\n*** TRAP: ");
		_puts((pc[-1]==0x73) ? "ECALL\n" : "EBREAK\n");
		INTEN|=ENSS;	// enable single-step
		notrace=0;
	}
	
	if (PFLAGS & CTRLC) { // Was ctrl-C ?
		i=UARTDAT; 		// get ctrl-c out of UART RX
		INTEN|=ENSS;	// and enable single-step
		notrace=0;
	}

	if (notrace) {	// check for breakpoints & the like
		if((uint32_t)pc == brk) notrace=0;	// breakpoint
		else if ((uint32_t)pc == tbrk) notrace=tbrk=0;	// temporary breakpoint
			else if (stopjalr) { // Stop on subroutine returns
					if (pc[-1]==0x8067) { // op: JALR X0, 0(X1)
						if (stopjalr==1) notrace=0; // any return
						else if (stack[1]>stacklevel) notrace=0; // stop if at upper stack level
							else return;
					} else return;
				} else return;
	}

	if (PFLAGS & DIRTY) { // pause if the app wrote something to terminal
		_puts("\n<pause>\n");
		_getch();
	}
	
	// Interactive debug
	while (notrace==0) {
		stopjalr=0;
		// Display state
		_printf("\033[2J\033[HPC: %08x      SP: %08x       brk: %08x   INTEN: %02x\n\n",
				stack[15],stack[1],brk,INTEN);
		for (i=0;i<16;i+=8) {
			_printf("X%d:",i); //if (i<10) _putch(' ');
			for (j=0;j<8;j++) {     
				if ((i+j)==0) _puts("   zero  ");
				else _printf(" %08x",stack[i+j-1]);
				if (j==3) _puts("  ");
			}
			_putch('\n');
		}
		_putch('\n');
		for (i=0;i<16;i++) {
			j=pc[i];
			_printf("\033[J%08x: %08x  ",(uint32_t)&pc[i],j);
			dissasemble(j,(uint32_t)&pc[i]);
		}
		
		// and do commands
docmd:		
		_puts("hcesnrRbgxdmwl> ");
		i=_getch(); _putch(i);
		switch(i) {
		case 's':	// single step
			goto ssend;
			
		case 'n':	// stop on next instr
			tbrk=(uint32_t)pc + 4;
			notrace=1;
			goto ssend;
			
		case 'r':	// execute until Return
			notrace=1; stopjalr=1;
			goto ssend;
			
		case 'R':	// execute until Return at higher stack
			notrace=1; stopjalr=2; stacklevel=stack[1];
			goto ssend;
			
		case 'c':	// Continue wo breakpoints (fast)
			INTEN&=~ENSS;
			_puts("\033[2J\033[H");
			goto ssend;

		case 'e':	// Continue with breakpoints (slow)
			notrace=1;
			_puts("\033[2J\033[H");
			goto ssend;
		
		case 'd':	// disassemble
			_putch(' ');
			if (_getsn(buf,9)) dadd=rdhex(buf);
			_putch('\n');
			for (i=dadd; i<dadd+64;i+=4) {
				j=*(uint32_t *)i;
				_printf("%08x: %08x  ",i,j);
				dissasemble(j,i);
			}
			dadd=i;
			goto docmd;
			
		case 'm':	// memory dump
			_putch(' ');
			if (_getsn(buf,9)) madd=rdhex(buf);
			_putch('\n');
			hexdump((char *)madd);
			madd+=256;
			goto docmd;

		case 'g':	// goto (set PC)
			_putch(' ');
			_putch(' '); _getsn(buf,9);
			stack[15]=rdhex(buf);
			pc=(uint32_t *)stack[15];
			break;

		case 'b':	// set breakpoint
			_putch(' '); _getsn(buf,9);
			brk=rdhex(buf);
			break;
			
		case 'x':	// change register
			_getsn(buf,3);
			i=rdhex(buf);
			if (i>15) i-=6;
			_putch('=');
			_getsn(buf,9);
			j=rdhex(buf);
			if (i>0 && i<16) stack[i-1]=j;
			break;
			
		case 'w':	// memory write
			_putch(' ');
			if (_getsn(buf,9)) wadd=rdhex(buf);
			_putch('\n');
			while(1){
				_printf("[%08x]=(%02x)=",wadd,*(uint8_t *)wadd);
				i=_getsn(buf,3); _putch('\n');
				if (i) {i=rdhex(buf); *(uint8_t *)wadd++=i;}
				else break;
			}
			goto docmd;	

		case 'l':  // restart (Load)
			stack[15]=(uint32_t)load;
			INTEN=0;
			return;
			
		case 'h': 
			help();
			goto docmd;	
						
		}
		_puts("\r\033[J");
	}
ssend:
	PFLAGS=0;	// Clear dirty
}


// --------------------------------------------------------


//#define NULL ((void *)0)
void __attribute__((naked)) loader(){
asm volatile(
"	lui		x3,0xE0000	\n"
"	li		x4,0x4c		\n"
"1:	lbu 	x5,0(x3)	\n"
"	bne		x4,x5, 1b	\n"
"	sb		x5,0(x3)	\n"
"	jalr	x1,68(x0)	\n"		// bootloader getw
"	mv		x7,x5		\n"
"	jalr	x1,68(x0)	\n"		// bootloader getw
"	add		x8,x7,x5	\n"
"	jalr	x1,68(x0)	\n"		// bootloader getw
"	mv		x9,x5		\n"
"	j		3f			\n"
"	\n"
"2:	lbu 	x5,0(x3)	\n"
"	sb		x5,0(x7)	\n"
"	addi	x7,x7,1		\n"
"3:	bne		x7,x8,2b	\n"
"	lui		sp,0x20020	\n"		// SP at RAM top	
"	li		x4,0x18		\n"
"	sb		x4,4(x3)	\n"		// clear dirty
"	sb		x4,0xE0(x3)	\n"		// enable IRQs
"	jr		x9			\n"		// Jump to program
);
}

void load()
{
	_puts("\n\n      Upload code to debug...\n\n");
	loader();
}
void main()
{

	//_delay_ms(1000);
	//c = UARTDAT;		// Clear RX garbage
	IRQVECT0=(uint32_t)debug_handler; // ECALL / EBREAK
	IRQVECT1=(uint32_t)debug_handler; // Single-Step / Control-C

	_puts("\n\n\tlaRVa_TT debugger\n\t J. Arias (2025)\n");
	help();
	
	load();
}
