#include <stdarg.h>
#include <stdint.h>

typedef unsigned char  u8;
typedef unsigned short u16;
typedef unsigned int   u32;

typedef signed char  s8;
typedef signed short s16;
typedef signed int   s32;


//-- Registros mapeados
#define UARTDAT  (*(volatile uint32_t*)0xE0000000)
#define PFLAGS  (*(volatile uint32_t*)0xE0000004)
#define RXVAL	(1)
#define TXRDY	(2)
#define RXFE	(4)
#define RXOVE	(8)

#define PWM      (*(volatile uint8_t*)0xE0000020)
#define TCNT     (*(volatile uint32_t*)0xE0000060)

#define IRQEN	 (*(volatile uint32_t*)0xE00000E0)
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
/*
uint8_t _getch()
{
	while((PFLAGS & RXVAL)==0);
	return UARTDAT;
}

uint8_t haschar() {return PFLAGS&1;}
*/

#define putchar(d) _putch(d)
#include "printf.c"

const static char *menutxt="\n"
"\n\n"
"888                8888888b.  888     888\n"         
"888                888   Y88b 888     888\n"        
"888                888    888 888     888\n"       
"888        8888b.  888   d88P Y88b   d88P 8888b.\n"  
"888           '88b 8888888P'   Y88b d88P     '88b\n"
"888       .d888888 888 T88b     Y88o88P  .d888888\n" 
"888888888 888  888 888  T88b     Y888P   888  888\n"
"888888888 'Y888888 888   T88b     Y8P    'Y888888\n"
"\nIts Alive :-)\n"
"\n";             

uint8_t udat[32];
volatile uint8_t rdix,wrix;

uint8_t _getch()
{
	uint8_t d;
	while(rdix==wrix);
	d=udat[rdix++];
	rdix&=31;
	return d;
}
                     
uint8_t haschar() {return wrix-rdix;}

uint32_t __attribute__((naked)) getMEPC()
{
	asm volatile(
	"	csrrw	a0,0x341,zero	\n"
	"	csrrw	zero,0x341,a0	\n"
	"	ret						\n"
	);
}

// ECALL / EBREAK
void __attribute__((interrupt ("machine"))) irq0_handler()
{
	uint32_t *pc;
	pc=(uint32_t *)getMEPC(); pc--;
	_printf("\nTRAP at 0x%x -%s-\n",(uint32_t)pc,(*pc==0x73)?"ECALL":"EBREAK");	
}

/*
// Single-Step
void  __attribute__((interrupt ("machine"))) irq1_handler(){
	uint32_t *pc;
	pc=(uint32_t *)getMEPC(); pc--;
	_printf("\nSSIRQ PC=0x%x\n",(uint32_t)pc);
	while ((PFLAGS&1)==0);
	IRQEN&=0xF7;
}
*/

// UART RX / TX
void __attribute__((interrupt ("machine"))) irq2_handler()
{
	static uint8_t a=32;
	if (PFLAGS&1) {
		udat[wrix++]=UARTDAT;
		wrix&=31;
	}else{
		UARTDAT=a;
		if (++a>=128) a=32;
	}
}

// PWM
void  __attribute__((interrupt ("machine"))) irq3_handler(){
	static int16_t y1=22000,y2=22000;
	int y;
	
	y=y1+y1-(y1/64)-y2;
	y2=y1;
	y1=y;
	
	PWM=90+(y/256);
}

// --------------------------------------------------------


#define NULL ((void *)0)

uint8_t *_memcpy(uint8_t *pdst, uint8_t *psrc, uint32_t nb)
{
	if (nb) do {*pdst++=*psrc++; } while (--nb);
	return pdst;
}

//#include "tv.c"
#include "sieve.c"

void main()
{
	char c,buf[17];
	uint8_t *p;
	unsigned int i,j;
	int n;
	void (*pcode)();
	uint32_t *pi;
	uint16_t *ps;

	//_delay_ms(1000);
	//c = UARTDAT;		// Clear RX garbage
	//IRQVECT0=(uint32_t)irq0_handler;
	//IRQVECT1=(uint32_t)irq1_handler;
	IRQVECT2=(uint32_t)irq2_handler;
	IRQVECT3=(uint32_t)irq3_handler;

	//_puts(menutxt);
	
	asm volatile ("ecall");
	asm volatile ("ebreak");

	IRQEN|=4+1;			// Enable PWM, UART RX IRQ

	while (1)
	{
			_puts("Command [12xt]> ");
			char cmd = _getch();
			if (cmd > 32 && cmd < 127)
				_putch(cmd);
			_puts("\n");

			switch (cmd)
			{
			case '1':
			    _puts(menutxt);
				break;
			case '2':
				IRQEN^=2;	// Toggle IRQ enable for UART TX
				_delay_ms(100);
				break;
			case 'x':
				_puts("Upload APP from serial port (<crtl>-F) and execute\n");
				IRQEN=0;
				asm volatile ("jalr zero,zero");
			case 't':	sieve(); //tv();
				break;
				
			default:
				continue;
			}
	}
}
