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
#define TCK_H()	(FIO0SET=TCK)
#define TCK_L()	(FIO0CLR=TCK)
#define TMS_H()	(FIO0SET=TMS)
#define TMS_L()	(FIO0CLR=TMS)
#define TDI_H()	(FIO0SET=TDI)
#define TDI_L()	(FIO0CLR=TDI)

#define GET_TDO() (FIO0PIN&TDO)
#define DELAY_T2() //(_delay_ns(100)) // Half cycle delay for TCK

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
#define GO_RESET() 			shift_tms(5,0b11111)
#define GO_IDLE()  			shift_tms(6,0b011111)
#define IDLE_TO_SHIFTIR() 	shift_tms(4,0b0011) 
#define IDLE_TO_SHIFTDR()	shift_tms(3,0b001) 
#define SHIFT_TO_IDLE() 	shift_tms(3,0b011) 
#define EXIT1_TO_IDLE() 	shift_tms(2,0b01) 

// Storage for input & output shift registers
#define MAXDRBITS	(1024)	// max. number of bits for any shift register chain
#define MAXCHIPS	(10)	// max. number of chips in the chain
uint32_t irlen,drlen;		// measured register lenghts
uint32_t shin[MAXDRBITS/32],shout[MAXDRBITS/32]; //
#define MAXIRBITS	(MAXCHIPS*8)	// max. number of bits for IR chain
uint32_t shir[(MAXIRBITS+31)/32];
uint8_t chipirpos[MAXCHIPS],chipirlen[MAXCHIPS];
uint8_t cchip;		// current chip for test

// BIT write 
void bitwr(uint32_t *pbuf, int bit, int val)
{
	uint32_t msk=1<<(bit&31);
	
	pbuf=&pbuf[bit>>5];
	if (val) *pbuf |= msk;
	else     *pbuf &= ~msk;
}

// Bit read
uint32_t bitrd(uint32_t *pbuf, int bit)
{
	return (pbuf[bit>>5]&(1<<(bit&31))) ? 1 : 0;
}

// Measure actual number of bits in register (IR or DR)
int get_reg_len()
{
	int i;
	
	// fill with 0s
	shin[0]=0;
	for (i=0;i<MAXDRBITS/32; i++) shift_tdi(32,shin,shout);
	// fill with 1s and wait for 1 at TDO
	shin[0]=1;
	for (i=0;i<MAXDRBITS;i++) {
		shift_tdi(1,shin,shout);
		if (shout[0]) break;
	}
	return i;
}

uint32_t bypassir[MAXIRBITS/32]; // Bypass values for all chips in the chain

// Set IR value (irlen, chipirpos[], chipirlen[], and bypassir[], must be valid)

void set_ir(int chip, int val)
{
	int i;
	for (i=0;i<MAXIRBITS/32;i++) shir[i]=bypassir[i]; // copy bypassir to shir
	for (i=chipirpos[chip]; i<chipirpos[chip]+chipirlen[chip]; i++) { 
		bitwr(shir,i,val&1); val>>=1;	// change bits for the chip
	} 
	IDLE_TO_SHIFTIR();
	shift_tdi_exit1(irlen, shir, shout);
	EXIT1_TO_IDLE();
}

// Capture and Update DR, skipping bypassed chips
// (cchip and drlen must have valid values)
void update_bsr()
{
	int i;
	IDLE_TO_SHIFTDR(); 
	for (i=0;i<cchip;i++) {	// Skip bypass bits on LSBs
		DELAY_T2();	TCK_H();
		DELAY_T2();	TCK_L();
	}
	shift_tdi_exit1(drlen-cchip,shin,shout);
	EXIT1_TO_IDLE(); // Set pin values
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
// print n dashes "----"
void prline(int n)
{
	while(n) {_putch('-'); n--;}
	_putch('\n');
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
// TinyTlaRVA stuff
///////////////////////////////////////////////////////////////////////////
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

// Emulated CLK pulse (during INTEST)
void clkpulse()
{
	shin[0]|=CLK; update_bsr();
	shin[0]^=CLK; update_bsr();
}


//////////////////////////////////////////////////////////////////
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
	set_ir(cchip,PRELOAD);
	shin[0]=XWEB | XOEB | TXD | RXD;	// Preload values (RSTB active)
	update_bsr(); //update_bsr();
	set_ir(cchip,INTEST);

	for (k=0;k<20;k++) clkpulse();
	shin[0]|=RESB;

	// Send UART data, starting with a dummy STOP bit	
	for (i=0;i<sizeof(header); i++) {
		d=(header[i]<<2)+1;
		for (j=0;j<10;j++) {
			if (d&1) shin[0]|=RXD; else shin[0]&=~RXD;
			d>>=1;
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
////////////////////////////////////////////////////////////////////
//    Chip database (minimal info)
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
const struct {
	uint32_t	idcode;		// manuf. ID
	uint16_t	irlen;		// IR size in bits
	uint16_t	bypass;		// IR instruction for Bypass
} chipdb[] ={
	0x00047FAB, 3, 0x7,		// TTLaRVa
	0xCABECEAD, 5, 0x1F,	// test value
	0x0DEFECAD, 3, 0x7,		// test value
	0xBEBEDCAF, 4, 0xF,		// test value
	0,0,0					// end of list
};


////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
//    MAIN function (never returns)
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////

void main()
{
	unsigned int idcode[MAXDRBITS/32],nchips,i,j,k,l,bsl;

// GPIO init (LPC2103 specific)
	SCS=1;						// Enable fast-GPIO
	FIO0DIR|= TCK | TMS | TDI;	// outputs
	FIO0CLR = TCK | TMS | TDI;	// start as 0

// JTAG code
//  Assumes: 
//    - Known devices in the chain
//    - TinyTlaRVa chip present (for internal tests)
init:
	GO_IDLE();	// reset & idle => DR = Device ID chain

	cchip=0xff;	// current chip starts as invalid
	
	// read IDCODEs
	IDLE_TO_SHIFTDR();
	shin[0]=0;	// shift zeroes from TDI
	_puts("\nBoundary Scan chain:\n"); prline(78); _puts("    TDO\n");
	for (nchips=k=0;nchips<MAXCHIPS;nchips++) {
		shift_tdi(32,shin,shout);
		j=shout[0];	
		if (j==0) break;
		idcode[nchips]=j;
		_printf("chip #%d > 0x%08x ",nchips,j);
		_printf("(manuf.: 0x%03x  part: 0x%04x  rev.: 0x%x)",
			j&0xFFF, (j>>12)&0xFFFF, (j>>28) );
		for (i=0;chipdb[i].idcode;i++) {
			if (chipdb[i].idcode==j) {
				if (j==0x00047FAB) { cchip=nchips; _puts(" < TTlaRVa");}
				chipirpos[nchips]=k;
				chipirlen[nchips]=chipdb[i].irlen;
				for (l=0;l<chipdb[i].irlen; l++) bitwr(bypassir,k+l,chipdb[i].bypass&(1<<l));
				k+=chipdb[i].irlen;
				break;
			}
		}
		if (chipdb[i].idcode==0) {
			_puts(" *** Unknown part");
			chipirpos[nchips]=k;
			chipirlen[nchips]=4;
			for (l=0;l<4; l++) bitwr(bypassir,k+l,1);
			k+=4;
		}
		_putch('\n');
		
	}
	_puts("    TDI\n"); prline(78);
	SHIFT_TO_IDLE();

	if (cchip==0xff) {
		_puts("\nNo TinyTlarVa found\n");
		_delay_ms(1000);
		goto init;
	} else _printf("Testing chip #%d (TinyTlaRVa)\n",cchip);

	// get IR LEN	
	IDLE_TO_SHIFTIR();
	irlen=get_reg_len();
	SHIFT_TO_IDLE();
	_printf("IRchain.len=%d\n",irlen);

	// get BSR+bypass length
	set_ir(cchip,SAMPLE); // Select BSR, default to Sample/preload
	IDLE_TO_SHIFTDR();
	drlen=get_reg_len();
	SHIFT_TO_IDLE();
	_printf("DRchain.len=%d (BSR + %d bypass)\n",drlen,nchips-1); 
	
	prline(32); _puts(
	"q  start again\n"
	"d  check DR lengths\n"
	"s  enter SAMPLE/preload\n"
	"x  enter EXTEST\n"
	"i  enter INTEST\n"
	"r  toggle RESET_n\n"
	"c  toggle CLK\n"
	"1  intest #1 (TTlaRVa)\n"
	"2  intest #2 (TTlaRVa)\n"
	); prline(32);

	while(1) {
		_puts("> "); i=_getch();
		switch(i){
		case 'q': goto init;
		case 's': set_ir(cchip,SAMPLE); _printf("SAMPLE/PRELOAD\n"); continue;
		case 'x': // external test
			update_bsr();		// Sample current values
			for (i=0;i<MAXDRBITS/32; i++) shin[i]=shout[i]; // and copy them to shin
			shin[0]|=RESB; 		// Disable Reset, just in case...
			update_bsr();
			set_ir(cchip,EXTEST); 	// Enter EXTEST mode
			_printf("EXTEST\n> "); 
			break;
		case 'i': 	// Internal test
			update_bsr();		// Sample current values
			for (i=0;i<MAXDRBITS/32; i++) shin[i]=shout[i]; // and copy them to shin
			shin[0]|=RESB; 		// Disable Reset, just in case...
			update_bsr();
			set_ir(cchip,INTEST);  	// Enter INTEST mode
			_printf("INTEST\n> ");  
			break;
		
		case 'r': shin[0]^=RESB; break; // Togle Reset (during INTEST/EXTEST)
		case 'c': shin[0]^=CLK; break;	// Togle CLK (during INTEST/EXTEST)
		
		case '1': intest1(); continue;	// Reset and skip bootloader
		case '2': intest2(); continue;	// Code execution through JTAG
		case 'd':
			// get DR lengths for each IR value
			for (l=0;l<nchips;l++) {
				_printf("--------\nchip #%d\n",l);
				for (i=k=bsl=0;i<(1<<chipirlen[l]); i++) {
					set_ir(l,i);
					IDLE_TO_SHIFTDR();
					_printf("IR=0x%x  DR.len=%d",i,j=get_reg_len());
					SHIFT_TO_IDLE();
					if (j==31+nchips && k==0) {k=1; _puts(" <DR: DIR");}
					else if (j==nchips) _puts("  <DR: Bypass");
					else if (!bsl) {bsl=j; _puts(" <DR: BSR");}
					_putch('\n');
				}
			}
			continue;
		
		
		}
		
		update_bsr();	// Set pin values
		update_bsr();	// and sample new values
		prtbin(drlen-nchips+1,shout); _putch('\n');

	}

}
