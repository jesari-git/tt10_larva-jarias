////////////////////////////////////////////////////////////////////////////
// Playing with JTAG
//  J. Arias (2025)
//
// Code for Embedded-Artist LPC2103 boards, but easily portable to other
// microcontrollers
////////////////////////////////////////////////////////////////////////////

#include <stdint.h>
#include <stdlib.h>
#include "lpc21xx.h"
#include "minilib.h"

// Bitbanging JTAG pins:
#define TCK	(1<<4)
#define TMS	(1<<5)
#define TDI	(1<<6)
#define TDO	(1<<7)

// Portability macros:
#define TCK_H()	(IO0SET=TCK)
#define TCK_L()	(IO0CLR=TCK)
#define TMS_H()	(IO0SET=TMS)
#define TMS_L()	(IO0CLR=TMS)
#define TDI_H()	(IO0SET=TDI)
#define TDI_L()	(IO0CLR=TDI)

#define GET_TDO() (IO0PIN&TDO)
#define DELAY_T2() //(_delay_us(5)) // Half cycle delay for TCK

// Change state (max 32 TCK pulses, TMS values in val)
void shift_tms(int ncy, uint32_t val)
{
	for (;ncy;ncy--) {
		if (val&1) TMS_H(); else TMS_L();
		val>>=1;
		DELAY_T2();
		TCK_H();
		DELAY_T2();
		TCK_L();
	}
}

// Shift DR/IR
//  more than 32 cycles possible
//  *pin:  values to send to TDI
//  *pout: values retrieved from TDO

void shift_tdi(int ncy, uint32_t *pin, uint32_t *pout)
{
	int i,j,k;
	while (ncy) {
		i=ncy; if (i>32) i=32;
		*pout=0;
		for (j=0,k=1;j<i;j++) {
			if ((*pin)&k) TDI_H(); else TDI_L();
			DELAY_T2();	
			TCK_H();
			if (GET_TDO()) *pout |=k;
			DELAY_T2();
			TCK_L();
			k<<=1;
		}
		pin++; pout++;
		ncy-=i;
	}
}

// Same as "shift_tdi" but
//  on the last TDI bit sets also TMS, meaning we end on
//  Exit1-DR or Exit1-IR states
//  (this avoids shifting one more bit than desired on TDO)
void shift_tdi_exit1(int ncy, uint32_t *pin, uint32_t *pout)
{
	int i,j,k;
	while (ncy) {
		i=ncy; if (i>32) i=32;
		*pout=0;
		for (j=0,k=1;j<i;j++) {
			if ((*pin)&k) TDI_H(); else TDI_L();
			if ((ncy<=32) && j==i-1) TMS_H(); // last bit: change state to exit1
			DELAY_T2();
			TCK_H();
			if (GET_TDO()) *pout |=k;
			DELAY_T2();
			TCK_L();
			k<<=1;
		}
		pin++; pout++;
		ncy-=i;
	}
}

// Some macros for state machine changes:
#define GO_RESET() shift_tms(5,0b11111)
#define GO_IDLE()  shift_tms(6,0b011111)
#define IDLE_TO_SHIFTIR() shift_tms(4,0b0011) 
#define IDLE_TO_SHIFTDR() shift_tms(3,0b001) 
#define SHIFT_TO_IDLE() shift_tms(3,0b011) 
#define EXIT1_TO_IDLE() shift_tms(2,0b01) 

// Storage for input & output shift registers
#define MAXBSTAP	(1024)	// max. number of bits for any shift register
uint32_t irlen,drlen;		// measured register lenghts
uint32_t shin[MAXBSTAP/32],shout[MAXBSTAP/32]; //

// Measure actual number of bits in register (IR or DR)
int get_reg_len()
{
	int i;
	
	// fill with 0s
	shin[0]=0;
	for (i=0;i<MAXBSTAP/32; i++) shift_tdi(32,shin,shout);
	// wait for 1
	shin[0]=1;
	for (i=0;i<MAXBSTAP;i++) {
		shift_tdi(1,shin,shout);
		if (shout[0]) break;
	}
	return i;
}

// Set IR value (irlen must be valid)
void set_ir(int val)
{
	uint32_t i,j;
	IDLE_TO_SHIFTIR();
	i=val;
	shift_tdi_exit1(irlen, &i, &j);
	EXIT1_TO_IDLE();
}

// Print shift register as binary data
void prtbin(int nbits, uint32_t *pdat)
{
	nbits--;
	while (nbits>=0) {
		_putch((pdat[nbits>>5]&(1<<(nbits&31))) ? '1':'0');
		if (nbits) {
			if ((nbits&3)==0) _putch('_');
			if ((nbits&15)==0) _putch('_');
		}
		nbits--;
	}
}


///////////////////////////////////////////////////////////////////////////
// BSR bits for TinyTlaRVA
#define RESB	(1)
#define CLK		(1<<1)
#define RXD		(1<<2)

#define XDIstart (7)
#define XDOstart (15)
#define XBH		(1<<23)
#define XLAL	(1<<24)
#define XLAH	(1<<25)
#define TXD		(1<<26)
#define XHH		(1<<27)
#define XOEB	(1<<28)
#define XWEB	(1<<29)

// IR commands
#define SAMPLE	(1)
#define PRELOAD	(1)
#define EXTEST	(2)
#define INTEST	(3)


////////////////////////////////////////////////////////////////////
// UART RX emulator:
// Samples the serial data line and retrieves the data
#define TBIT	(208) // cycles per serial bit (24MHz / 115200 baud)
void uart_emulator(int rx)
{
	static int8_t orx;
	static int16_t cnt;
	static int16_t sh=0x3FF;
	int i,j;
	
	rx=(rx) ? 1: 0;
	if (rx!=orx) {
		orx=rx;
		cnt=0;
	} 
	cnt++;
	if (cnt==TBIT) cnt=0;
	if (cnt==TBIT/2) {
		sh>>=1;
		sh|=rx<<9;
		if ((sh&1)==0) {
			i=(sh>>1)&0xff;
			sh=0x3FF;
			j=(i>=32 && i<127) ? i : '.';
			//_printf("\n\t\t\t\t\t\t\tTXD: %02x '%c'",i,j);
			_printf("\033[64GTXD: %02x '%c'",i,j);
		}
	}
}

// Capture and Update DR
void update_bsr()
{
	IDLE_TO_SHIFTDR(); 
	shift_tdi_exit1(drlen,shin,shout);
	EXIT1_TO_IDLE(); // Set pin values
}

// Emulated CLK pulse (during INTEST)
void clkpulse()
{
	shin[0]|=CLK; update_bsr();
	shin[0]^=CLK; update_bsr();
}


// Internal test #1: Send serial data to bootloader:
//    address: 0			(doesn't matter)
//    size:    0		  	(no code upload)
//    entry:   0x20000000 	(start of external RAM)
// and keep pulsing CLK until XLAL is set (external memory access)

void intest1()
{
	int i,j,k,l,d;
	static const uint8_t header[13]={'L', 0,0,0,0, 0,0,0,0, 0,0,0,0x20};

	_printf("\nINTEST1. Executing bootloader with dummy header and jump to 0x20000000\n");
	set_ir(PRELOAD);
	shin[0]=XWEB | XOEB | TXD | RXD;	// Preload values (RSTB active)
	update_bsr(); update_bsr();
	set_ir(INTEST);

	for (k=0;k<20;k++) clkpulse();
	shin[0]|=RESB;

	// Send UART data, starting with a dummy STOP bit	
	for (i=0;i<sizeof(header); i++) {
		d=(header[i]<<2)+1;
		for (j=0;j<10;j++) {
			if (d&1) shin[0]|=RXD; else shin[0]&=~RXD;
			d>>=1;
			update_bsr();
			for (k=0;k<TBIT;k++) {clkpulse(); uart_emulator(shout[0]&TXD);}
		}
	}

	// Set STOP bit level
	shin[0]|=RXD;
	// give clocks until an external memory access
	do { clkpulse(); } while ((shout[0]&XLAL)==0);
	_putch('\n');
}

// Internal test #2: 
// Emulates the external RAM and executes its contents
#define RAMSZ	(1024)
uint32_t RAM[RAMSZ/4]={
0xe00001b7,	//00:	 lui  x3,0xE0000
0x200002b7, //04:    lui  x5,0x20000
0x02000213, //08:	 li   x4,0x20
0x00418023, //0C: 1: sb   x4,0(x3)
0x04429023, //10:    sh   x4,0x40(x5)
0x00228293, //14:    addi x5,x5,2
0x00120213, //18:    addi x4,x4,1
0xff1ff06f, //1C:    j    1b
};


void intest2()
{
	uint32_t a, d, wr;
	uint8_t *pram;
	
	_printf("\nINTEST2. Executing from emulated external RAM (press key to start / stop)\n"); _getch();
	pram=(uint8_t *)RAM;
	wr=0;
	while(1) {
		shin[0]&=~CLK; update_bsr();
		update_bsr();
		if (U0LSR&1) {d=_getch(); _putch('\n'); return; }
		// CLK low
		uart_emulator(shout[0]&TXD);
		if (shout[0] & XLAL) {a=((shout[0]>>XDOstart)&0xFF)<<2; _putch('\n');}
		if (shout[0] & XLAH) a|=((shout[0]>>XDOstart)&0xFF)<<10;

		if ((shout[0] & XWEB)==0) {
			a&=~3; 
			if (shout[0]&XBH) a|=1; 
			if (shout[0]&XHH) a|=2; 
			pram[a&(RAMSZ-1)]=(shout[0]>>XDOstart)&0xFF;
			if(!wr) _printf("\t\t\t\tw:[%8x] <- ",0x20000000+a); 
			_printf("%02x ",pram[a&(RAMSZ-1)]);
			wr=1;
		}
		shin[0]|=CLK; update_bsr();
		update_bsr();
		// CLK high
		//if ((shout[0] & XWEB)==0) _printf("WR");
		if ((shout[0] & XOEB)==0) {
			wr=0;
			a&=~3; 
			if (shout[0]&XBH) a|=1; 
			if (shout[0]&XHH) a|=2;
			d=pram[a&(RAMSZ-1)];
			if ((a&3)==0) _printf("r:[%8x] -> ",0x20000000+a);
			_printf("%02x ",d);
			shin[0]&=~(0xFF<<XDIstart); shin[0]|=d<<XDIstart;
		}
	}

}

////////////////////////////////////////////////////////////////////
//    MAIN function (never returns)
////////////////////////////////////////////////////////////////////

void main()
{
	unsigned int idcode,i,j,k;

// GPIO init (LPC2103 specific)
	IO0DIR|= TCK | TMS | TDI;	// outputs
	IO0CLR = TCK | TMS | TDI;	// start as 0

// JTAG code
//  Assumes: 
//    - Only 1 device in the chain
//    - TinyTlaRVa chip (for internal test)
init:
	GO_IDLE();	// reset & idle

	// read IDCODEs
	IDLE_TO_SHIFTDR();
	shin[0]=0;	// shift zeroes from TDI
	_puts("Boundary Scan chain:\n--------------------\n    TDO\n");
	for (i=0;i<MAXBSTAP/32;i++) {
		shift_tdi(32,shin,shout);
		j=shout[0];	
		if (j==0) break;
		idcode=j;
		_printf("chip #%d > 0x%08x ",i+1,idcode);
		_printf("(manuf.: 0x%03x  part: 0x%04x  rev.: 0x%x)\n",
			idcode&0xFFF, (idcode>>12)&0xFFFF, (idcode>>28) );
	}
	_puts("    TDI\n--------------------\n");
	SHIFT_TO_IDLE();
	if (i>1) {
		_printf("\nOnly one chip in chain supported. Sorry\n");
		_delay_ms(1000);
		goto init;
	}

	// get IR LEN	
	IDLE_TO_SHIFTIR();
	irlen=get_reg_len();
	_printf("IR.len=%d\n",irlen);
	SHIFT_TO_IDLE();
	
	// get DR lengths for each IR value
	for (i=k=drlen=0;i<(1<<irlen); i++) {
		set_ir(i);
		IDLE_TO_SHIFTDR();
		_printf("IR=0x%x  DR.len=%d",i,j=get_reg_len());
		SHIFT_TO_IDLE();
		if (j==32 && k==0) {k=1; _puts(" <DR: DIR");}
		else if (j==1) _puts("  <DR: Bypass");
		else if (!drlen) {drlen=j; _puts(" <DR: BSR");}
		_putch('\n');
	}

	if (idcode!=0x00047FAB) {
		_printf("\nOnly TinyTlaRVa chip supported. Sorry\n");
		_delay_ms(1000);
		goto init;
	}
	
	set_ir(SAMPLE); // Sample/preload
	
	while(1) {
		_printf("> "); i=_getch();
		switch(i){
		case 'q': goto init;
		case 's': set_ir(SAMPLE); _printf("SAMPLE/PRELOAD\n"); continue;
		case 'x': // esternal test
			update_bsr();		// Sample current values
			for (i=0;i<MAXBSTAP/32; i++) shin[i]=shout[i]; // and copy them to shin
			shin[0]|=RESB; 		// Disable Reset, just in case...
			update_bsr();
			set_ir(EXTEST); 	// Enter EXTEST mode
			_printf("EXTEST\n> "); 
			break;
		case 'i': 	// Internal test
			update_bsr();		// Sample current values
			for (i=0;i<MAXBSTAP/32; i++) shin[i]=shout[i]; // and copy them to shin
			shin[0]|=RESB; 		// Disable Reset, just in case...
			update_bsr();
			set_ir(INTEST);  	// Enter INTEST mode
			_printf("INTEST\n> ");  
			break;
		
		case 'r': shin[0]^=RESB; break; // Togle Reset (during INTEST/EXTEST)
		case 'c': shin[0]^=CLK; break;	// Togle CLK (during INTEST/EXTEST)
		
		case '1': intest1(); continue;	// Reset and skip bootloader
		case '2': intest2(); continue;	// Code execution throung JTAG
		}
		
		update_bsr();	// Set pin values
		update_bsr();	// and sample new values
		prtbin(drlen,shout); _putch('\n');

	}

}
