/*
 * Copyright (c) 2025 Jesús Arias
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

//`include "laRVa.v"
//`include "uart_simple.v"

module tt_um_larva (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
    `ifdef FPGA
    	,output [2:0]debug
    `endif
);
`ifdef FPGA
	assign debug={pwmout,~pwmout,1'b0};
`endif
/*
  // All output pins must be assigned. If not used, assign to 0.
  assign uo_out  = ui_in + uio_in;  // Example: ou_out is the sum of ui_in and uio_in
  assign uio_out = 0;
  assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
  wire _unused = &{ena, clk, rst_n, 1'b0};
*/  

wire pwmpin = streset ? pwmout : tdo;
assign uo_out=exintest ? {bsq[29:26],       pwmpin ,bsq[25:23]} : 
			  		   {xweb,xoeb,xhh,txd, pwmpin ,xlah,xlal,xbh};
assign uio_out=exintest ? bsq[22:15] : cuio_out;

wire _unused = &{ ena, 1'b0};

////////////// JTAG ///////////////

localparam BSLEN=30;
wire tdo,extest,intest,streset;
wire [BSLEN-1:0]bsd=intest ?
			{xweb,xoeb,xhh,txd,xlah,xlal,xbh,cuio_out,juio_in,gpin,rxd,jclk,~reset} :
			{uo_out[7:4],uo_out[2:0],uio_out,uio_in,ui_in[7:3],clk,rst_n};
wire [BSLEN-1:0]bsq;
JTAG_TAP #(.BSLEN(BSLEN)) jtag0( .reset(~rst_n), 
				.tck(ui_in[0]),.tms(ui_in[1]), .tdi(ui_in[2]),
				.tdo(tdo), .extest(extest), .intest(intest), .streset(streset),
				.bsd(bsd), .bsq(bsq) );

wire reset= (~rst_n) | ((extest|intest)&(~bsq[0]));
assign rxd=exintest ? bsq[2] : ui_in[3];
wire [3:0]gpin=exintest ? bsq[6:3] : ui_in[7:4];
wire [7:0]juio_in= exintest ? bsq[14:7] : uio_in;

// clock synchronizer
/*
reg [1:0]exintest;	// extest delayed
always @(posedge clk) exintest<={extest,exintest[1]};
wire [3:0]jclkv={bsq[1],1'b1,1'b1,clk}; // clock mux
wire jclk=jclkv[exintest]; 
*/
reg exintest;	// extest delayed
always @(posedge clk or negedge rst_n) 
	if (~rst_n) exintest<=0; else exintest<=extest|intest;
reg bsq1r;		// BS clock sampled
always @(posedge clk) bsq1r<=bsq[1];
wire jclk= exintest ? bsq1r : clk;

///////////////// internal system /////////////////////
parameter FCLK=24000000;
parameter BAUD=115200;

/////////////////////////////// MEMORY ////////////////////////////////
// Internal boot ROM (32 words), Combinational

reg [31:0]brom[0:31];
wire [31:0]bromo=brom[ca[6:2]];
initial begin
brom[ 0]=32'he00001b7; //          	lui		x3,0xE0000	// I/O area
brom[ 1]=32'h04c00213; //          	li		x4,0x4c		// char 'L'
brom[ 2]=32'h0001c283; //	1:		lbu 	x5,0(x3)	// UART RX (blocking)
brom[ 3]=32'hfe521ee3; //          	bne		x4,x5,1b
brom[ 4]=32'h00518023; //          	sb		x5,0(x3)	// echo 'L'
brom[ 5]=32'h030000ef; //          	call	getw
brom[ 6]=32'h00028393; //          	mv		x7,x5		// loading address
brom[ 7]=32'h028000ef; //          	call	getw		// number of bytes
brom[ 8]=32'h00538433; //          	add		x8,x7,x5	// end address
brom[ 9]=32'h020000ef; //          	call	getw
brom[10]=32'h00028493; //          	mv		x9,x5		// Entry address
brom[11]=32'h0100006f; //          	j		3f
brom[12]=32'h0001c283; //	2:		lbu 	x5,0(x3)	// data block
brom[13]=32'h00538023; //          	sb		x5,0(x7)
brom[14]=32'h00138393; //          	addi	x7,x7,1
brom[15]=32'hfe839ae3; //   3:     	bne		x7,x8,2b
brom[16]=32'h00048067; //          	jr		x9			// Jump to code

brom[17]=32'h0001c283; //   getw:  	lbu 	x5,0(x3)	// get 32-bit word in X5
brom[18]=32'h0001c303; //          	lbu 	x6,0(x3)
brom[19]=32'h00831313; //          	slli 	x6,x6,8
brom[20]=32'h0062e2b3; //          	or		x5,x5,x6
brom[21]=32'h0001c303; //          	lbu 	x6,0(x3)
brom[22]=32'h01031313; //          	lli 	x6,x6,16
brom[23]=32'h0062e2b3; //          	or		x5,x5,x6
brom[24]=32'h0001c303; //          	lbu 	x6,0(x3)
brom[25]=32'h01831313; //          	slli 	x6,x6,24
brom[26]=32'h0062e2b3; //          	or		x5,x5,x6
brom[27]=32'h00008067; //          	ret
end



	
////////////////////////////////////////////////
// External SRAM (8bit + address latches)

///// clock counter & divider (1/6) /////
reg [2:0]ckd=0;
always @(posedge jclk or posedge reset) 
	if (reset) ckd<=0; 
	else ckd<=(~ckd[2])&ckd[0] ? 3'b100 : ckd+1;

wire cksys = ~ckd[2];			// System clock

/////// External memory control signals /////
wire xlal = ((~ckd[2])&(~ckd[0])) & xramcs;		// latch address low  (for '373)
wire xlah = ((~ckd[2])&( ckd[0])) & xramcs;		// latch address high (for '373)
wire xbh  = ckd[0] & xramcs;					// high byte (A[0] in SRAM)
wire xhh  = ckd[1] & xramcs;					// high halfword (A[1] in SRAM)
wire xoeb = ~((~we) & ckd[2] & xramcs);			// /OE for SRAM
wire xweb = jclk | (~ckd[2]) | (~mwe[ckd[1:0]]) | (~xramcs);	// /WE for SRAM

//// 8-bit BUS mux ////
wire [7:0]d8o[0:7];
assign d8o[0]= ca[9:2];
assign d8o[1]= ca[17:10];
assign d8o[2]=8'hXX;
assign d8o[3]=8'hXX;
assign d8o[4]=cdo[ 7: 0];
assign d8o[5]=cdo[15: 8];
assign d8o[6]=cdo[23:16];
assign d8o[7]=cdo[31:24];
wire [7:0]cuio_out=d8o[ckd];
// uo_out[6] = xoeb
assign uio_oe=uo_out[6] ? 8'hff : 8'h00;

// input registers for low bytes
reg [7:0]xdl0;
reg [7:0]xdl1;
reg [7:0]xdl2;
always @(posedge jclk) begin
	if (ckd==3'b100) xdl0<=juio_in;
	if (ckd==3'b101) xdl1<=juio_in;
	if (ckd==3'b110) xdl2<=juio_in;
end

wire [31:0]xdi = {juio_in,xdl2,xdl1,xdl0}; // data input (word) from external RAM

///////////////////////////////////////////////////////
////////////////////////// CPU ////////////////////////
///////////////////////////////////////////////////////

wire stopcpu; 		// stop CPU if 1
wire cclk = cksys | stopcpu;	// CPU clock
wire [31:2]	ca;		// CPU Address
wire [31:0]ca32={ca,2'b00};
wire [31:0]	cdo;	// CPU Data Output
wire [3:0]	mwe;	// Memory Write Enable (4 signals, one per byte lane)
wire we=|mwe;		// Write enable (any lane)
wire irq;
wire [31:2]ivector;	// Where to jump on IRQ
wire trap;			// Trap irq (to IRQ vector generator)
wire usermode;

laRVa #(.NREG(16)) cpu (
		.clk     (cclk ),
		.reset   (reset),
		.addr    (ca[31:2] ),
		.wdata   (cdo  ),
		.wstrb   (mwe  ),
		.rdata   (cdi  ),
		.usermode(usermode),
		.irq     (irq  ),
		.ivector (ivector),
		.trap    (trap)
	);

///////////////////////////////////////////////////////
///// Memory mapping
wire bromcs= (ca[31:29]==3'b000);
wire xramcs= (ca[31:29]==3'b001);
wire iocs=   (ca[31:29]==3'b111);

// Input bus mux
wire [31:0]cdi = 
	(bromcs ? bromo : 0) |
	(xramcs ? xdi   : 0) |
	(iocs   ? iodo  : 0);

//////////////////////////////////////////////////
////////////////// Peripherals ///////////////////
//////////////////////////////////////////////////

reg [31:0]tcount=0;
always @(posedge cksys) tcount<=tcount+1;

// Write chip-selects
wire uartcs = iocs&(ca[7:5]==3'b000); // UART  at offset 0x00
wire pwmcs  = iocs&(ca[7:5]==3'b001); // PWM  at offset 0x20
wire irqcs  = iocs&(ca[7:5]==3'b111); // IRQEN at offset 0xE0

// Peripheral output bus mux
wire [31:0]iodo = 
	((ca[7:5]==3'b000)&(~ca[2]) ? {24'h0,uart_do} 	: 0 ) |
	((ca[7:5]==3'b000)&( ca[2]) ? {25'h0,pwmirq,udirty,uctrlc,urxoverr,urxframeer,utxrdy,urxvalid} : 0 ) |
	((ca[7:5]==3'b001) 			? {28'h0,gpin}		: 0 ) |
	((ca[7:5]==3'b011) 			? tcount 			: 0 ) |
	((ca[7:5]==3'b111) 			? {27'h0,irqen} 	: 0 );

/////////////////////////////
// UART
wire txd, rxd;
wire utxrdy,urxvalid,urxframeer,urxoverr; // Flags
wire [7:0] uart_do;	// RX output data
wire uwrtx;			// UART TX write
wire urd;			// UART RX read (for flag clearing)

// Strobes
// Offset 0: write: TX reg
// Offset 0: read strobe: Clear rxvalid and rxoverr

assign uwrtx   = uartcs & (~ca[2]) & mwe[0];
assign urd     = uartcs & (~ca[2]) & (mwe==4'b0000); // Clear DV, OVE flgas

localparam DIVIDER=(FCLK/6+(BAUD/2))/BAUD;

UART_core #(.DIVIDER(DIVIDER)) uart0 ( 
	.clk(cksys), .reset(reset),
	.d(cdo[7:0]), .wr(uwrtx&(~stopcpu)),. rd(urd&(~stopcpu)), .q(uart_do),
	.rxvalid(urxvalid), .txrdy(utxrdy),
	.rxoverr(urxoverr), .rxframeer(urxframeer),
	.nstop(1'b0),
	.txd(txd), .rxd(rxd)
);
assign stopcpu = (urd & (~urxvalid)) | (uwrtx & (~utxrdy));

// debug flags
wire uctrlc=urxvalid & (uart_do==8'h03);
reg udirty;
wire udirtyclr=uartcs & ca[2] & mwe[0]; // write to PFLAGS clears udirty
always @(posedge cclk) 
  udirty<= uwrtx ? 1 : (udirtyclr ? 0 : udirty);

//////////////////////////////////////////
//  PWM
wire pwmwr = pwmcs & mwe[0];
reg [7:0]pwmc;
reg [7:0]pwmbuf;
reg [7:0]pwm;
reg pwmout;
reg pwmirq;
wire pwmtc = pwmc[7]&pwmc[5]&pwmc[4]&pwmc[2]; // MAX = 0xB4 (180)
wire pwmzc = (pwmc==0);				// zero count

always @(posedge cclk or posedge reset) begin
	if (reset) begin
	   	pwmc<=0;
	   	pwmout<=0;
		pwmirq<=0; 
	end	else begin
		pwmc<= pwmtc ? 0 : pwmc + 1;
		if (pwmwr) pwm<=cdo[7:0];
		if (pwmtc) pwmbuf<=pwm;
		pwmout<= (pwmc==pwmbuf) ? 0 : (pwmzc ? 1 : pwmout);
		pwmirq<= pwmzc ? 1 : (pwmwr  ? 0 : pwmirq);
	end
end

//////////////////////////////////////////
//    Interrupt control

// IRQ enable reg
reg [4:0]irqen=0;
always @(posedge cclk or posedge reset) begin
	if (reset) irqen<=0; else
	if (irqcs & (~ca[4]) &mwe[0]) irqen<=cdo[4:0];
end

// IRQ vectors
reg [31:2]irqvect[0:3];
always @(posedge cclk) if (irqcs & ca[4] & (mwe==4'b1111)) irqvect[ca[3:2]]<=cdo[31:2];

// Pending IRQs
wire [2:0]irqpen={ (irqen[3] & usermode) | (irqen[4] & uctrlc),
				   irqen[2] & pwmirq,
				   (irqen[1] & utxrdy) | (irqen[0] & urxvalid) };	// pending IRQs

// Priority encoder
wire [1:0]vecn = trap      ? 2'b00 : (	// ECALL, EBREAK: highest priority
	 			 irqpen[2] ? 2'b01 : (	// Single Step / Crtl-c
				 irqpen[0] ? 2'b10 : 	// UART RX/TX
				 			 2'b11 ));	// PWM
assign ivector = irqvect[vecn];
assign irq = (irqpen!=0)|trap;

endmodule



///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// Simple UART: data formats: 8N1, 8N2. Fixed baud rate (selected by DIVIDER)
// by J. Arias
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
module UART_core (
	input	clk,		// System clock
	input   reset,
	input	[7:0]d,		// Input bus for TX
	input	wr,			// write strobe: starts TX
	input	rd,			// read strobe: clears RX flags
	output	[7:0]q,		// Output bus for RX
	output	txrdy,		// TX ready flag
	output	rxvalid,	// RX valid data flag
	output	rxoverr,	// RX overrun error flag
	output	rxframeer,	// RX framing error flag
	input   nstop,		// Set to 1 for two stop bits
	input 	rxd,		// serial data input
	output 	txd			// serial data output
);
parameter DIVIDER= 217;	// BAUD = f_clk / DIVIDER
localparam DBITS = $clog2(DIVIDER);	// number of bits for clock counters

///////////////////// UART TX ////////////////////
reg [DBITS-1:0]txdiv;
wire txdivmax=(txdiv==(DIVIDER-1));
always @(posedge clk or posedge reset) 
	if (reset) txdiv<=0;
	else if (wr | txdivmax) txdiv<=0; // Reset if max or on writes
		else txdiv<=txdiv+1;

reg [8:0]txsh;	// 9-bit shift register
always @(posedge clk or posedge reset)
	if (reset) txsh<=9'h1FF; 
	else begin
		if (wr) begin 
			`ifdef SIMULATION
		        $write ("%c",d); $fflush ( );
			`endif
			 txsh<={d,1'b0}; // Load data plus START bit
		end	else if (txdivmax) txsh<={1'b1,txsh[8:1]}; // shift 1'b1 in for future STOP bit
	end
	
assign txd = txsh[0];

reg [3:0]txbitcnt;	// Bit counter (0000 means idle)
wire utxbusy=|txbitcnt;
always @(posedge clk or posedge reset)
	if (reset) txbitcnt<=0;
	else
		if (wr) txbitcnt<={3'b101,nstop}; // set to eleven for 2 STOP bits, ten for 1 STOP bit
		else if (utxbusy&txdivmax) txbitcnt<=txbitcnt-1;
	
assign txrdy=~utxbusy;
	
///////////////////// UART RX ////////////////////
reg [1:0]rxreg; // two samples of RXD
always @(posedge clk or posedge reset) 
	if (reset) rxreg<=0; else rxreg<={rxreg[0],rxd};
	
reg [DBITS-1:0]rxdiv;
always @(posedge clk or posedge reset)
	if (reset) rxdiv<=0;
	else 
		if ((rxreg[1]^rxreg[0])|(rxdiv==(DIVIDER-1))) rxdiv<=0; // Reset if max or on any RXD edge
			else rxdiv<=rxdiv+1;
wire rxsample = (rxdiv==(DIVIDER/2-1)); // sample at the middle of the bit

reg [9:0]urxsh; // 10-bit shift register
reg [8:0]urxbuffer;		// 9-bit buffer: data + STOP
always @(posedge clk or posedge reset) begin
	if (reset)  urxsh<=10'h3FF; 
	else begin
		if (~urxsh[0]) urxsh<=10'h3FF;  // if START bit at LSB set to all ones and copy results
		else if (rxsample) urxsh<={rxreg[1],urxsh[9:1]};
	end
end

always @(posedge clk) 
	if (~urxsh[0]) urxbuffer<=urxsh[9:1]; 	  // store the received data plus STOP

reg [1:0]urxval;		// received data flags (00: no data, 01: data available, 11: overrun)
always @(posedge clk or posedge reset)
	if (reset) urxval<=0;
	else if (~urxsh[0]) urxval<={urxval[0],1'b1};  // set data valid and (maybe) overrun
	else if (rd) urxval<=0;  // clear data valid and overrun flags on reads

assign q = urxbuffer[7:0];	
assign rxframeer = ~urxbuffer[8];	// STOP bit in 0 means Frame error
assign rxvalid = urxval[0];
assign rxoverr = urxval[1];

endmodule






///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
//                laRVa RV32E / RV32I pipelined core                         //
//                       Jesús Arias (2022)                                  //
//                                                                           //
// Public Domain code (bugs included). Credits to:                           //
//  Claire Wolf (PicoRV core). The first I got working                       //
//  Bruno Levy & Matthias Koch (FemtoRV core). Much code was taken from here.//
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

module laRVa (
		input  clk,			// 
		input  reset,		// active high, asynchronous
		output [31:2]addr,	// Bits 0 and 1 missing (words aligned to 32 bits)
		output [31:0]wdata,	// Data output
		output [3:0]wstrb,	// Byte strobes for writes
		input  [31:0]rdata,	// Data input
		output usermode,	// 1: user mode, 0: machine (IRQ) mode
		input  irq,			// Interrupt, active high
		input  [31:2]ivector,// IRQ vector 
		output trap			// Trap interrupt requested for ECALL...
	);

parameter NREG=16;			// Number of registers
parameter RESET_PC=0;		// Where to start executing code

///////////////////////////////////////////////
//////////////// register bank ////////////////
///////////////////////////////////////////////

localparam LOG2NREG=$clog2(NREG);
localparam MAXREG=(1<<LOG2NREG)-1;

reg [31:0] regs [1:MAXREG];

integer i;
initial begin
	for (i=1; i<=MAXREG; i=i+1) regs[i]=0;
end

wire [4:0]rd;	// Destination register address
wire [4:0]rs1;	// Source 1 register address
wire [4:0]rs2;	// Source 2 register address
wire regswr;	// Write enable

wire [31:0]regsD;	// Input data to register bank
wire [31:0]regsQ1;	// Output 1	(to ALU)
wire [31:0]regsQ2;	// Output 2 (to ALU)
assign regsQ1=(rs1[LOG2NREG-1:0]!=0) ? regs[rs1[LOG2NREG-1:0]] : 32'h0;
assign regsQ2=(rs2[LOG2NREG-1:0]!=0) ? regs[rs2[LOG2NREG-1:0]] : 32'h0;

always @(posedge clk) begin
	if ((rd[LOG2NREG-1:0]!=0)&regswr) regs[rd[LOG2NREG-1:0]]<=regsD;
end

//////////////////////////////////////////////////////
////////      Instr Reg. and some decoding    ////////
//////////////////////////////////////////////////////
wire opload, opstore, opjal, opjalr, opbranch, opimm, opreg;
wire oplui, opauipc, opsystem, jump;

// Valid op-code flag
//  During the execution cycle of Loads, Stores, and Jumps
//  an invalid op-code is loaded into the IR register, and
//  this flag has to be set to zero
reg opvalid=0;	
always @(posedge clk or posedge reset) begin
	if (reset) opvalid<=0;
	else opvalid<=~(opload | opstore | jump | irqstart | mret);
end

// Instruction register
reg [31:0]IR;
always @(posedge clk) IR<=rdata;

// Decoding of op-code fields
wire [6:0]opcode=IR[6:0];
wire [2:0]funct3=IR[14:12];
wire [6:0]funct7=IR[31:25];

assign rd  = opvalid ? IR[11:7]: 0;
assign rs1 = opvalid ? IR[19:15]: 0;
assign rs2 = opvalid ? IR[24:20]: 0;

// The ten instruction opcodes
assign opreg    = opvalid & (opcode==7'b0110011);	// Rd = Rs1 op Rs2
assign opimm    = opvalid & (opcode==7'b0010011);	// Rd = Rs1 op imm
assign opload   = opvalid & (opcode==7'b0000011); 
assign opstore  = opvalid & (opcode==7'b0100011);
assign opbranch = opvalid & (opcode==7'b1100011);	// Branch if condition
assign opjal    = opvalid & (opcode==7'b1101111);
assign opjalr   = opvalid & (opcode==7'b1100111);
assign oplui    = opvalid & (opcode==7'b0110111);
assign opauipc  = opvalid & (opcode==7'b0010111);
assign opsystem = opvalid & (opcode==7'b1110011);

// Jumps: JAL, JALR, and taken Branches
assign jump = opjal | opjalr | (opbranch & predicate);
// Write register file on all instructions except Store, Branches, and System
assign regswr = (opreg|opimm|opload| opjal | opjalr | oplui | opauipc | csrrw_mepc);
// System opcodes supported
wire mret  = opsystem & (funct3==3'b000) & IR[21];	//Return from interrupts (user or machine)	
wire ecall = opsystem & (funct3==3'b000) & (IR[21:20]==2'b00);
wire ebreak= opsystem & (funct3==3'b000) & (IR[21:20]==2'b01);
wire csrrw_mepc = opsystem  & (funct3==3'b001) & (IR[31:20]==12'h341);
/////////////////////////
// Inmediate values
/////////////////////////
// The five immediate formats
wire [31:0] Uimm = {    IR[31],   IR[30:12], {12{1'b0}}};			// Upper
wire [31:0] Iimm = {{21{IR[31]}}, IR[30:20]};						// Imm
wire [31:0] Simm = {{21{IR[31]}}, IR[30:25],IR[11:7]};				// Store
wire [31:0] Bimm = {{20{IR[31]}}, IR[7],IR[30:25],IR[11:8],1'b0};	// Branch
wire [31:0] Jimm = {{12{IR[31]}}, IR[19:12],IR[20],IR[30:21],1'b0};	// JAL

//////////////////////////////////////////////////////
////////      ALU & other data processing     ////////
//////////////////////////////////////////////////////

// First ALU source
wire [31:0] aluIn1 = oplui ? 0 : (opauipc ? {PCci,2'b00} : regsQ1);

// Second ALU source
wire [31:0] aluIn2 = 
	( (opreg|opbranch)      ? regsQ2 : 32'h0 ) |
	( (opimm|opload|opjalr) ? Iimm   : 32'h0 ) |
	( opstore               ? Simm   : 32'h0 ) |
	( (oplui|opauipc)       ? Uimm   : 32'h0 ) ;

// The ALU has to subtract on SUB, SLT, SLTU, SLTI, SLTIU, and branches
wire issub = (opreg & (funct7[5] | (funct3==2) | (funct3==3))) |
			 (opimm & ((funct3==2) | (funct3==3))) | opbranch;

// Use a single 33 bits subtract (32+carry) for SUB and all comparisons
wire [32:0] aluAdder = {issub, (issub ? ~aluIn2 : aluIn2)} + aluIn1 + issub;
wire LT  = (aluIn1[31] ^ aluIn2[31]) ? aluIn1[31] : aluAdder[32];
wire LTU = aluAdder[32];
wire EQ  = (aluAdder[31:0] == 0);

  /////////////////////////////////////////////////
  // Barrel shifter
  /////////////////////////////////////////////////

wire [31:0]sh_in;	// Input to right shift
wire [31:0]sh_out;	// output
wire left_sh;		// Shift to left
wire signed_sh;		// Shift right preserving negatives
assign left_sh = (opreg | opimm) & (funct3==3'h1);

// reverse the bit order at the input for left shifts
assign sh_in = left_sh ? 
	{aluIn1[ 0],aluIn1[ 1],aluIn1[ 2],aluIn1[ 3],aluIn1[ 4],aluIn1[ 5],aluIn1[ 6],aluIn1[ 7],
	 aluIn1[ 8],aluIn1[ 9],aluIn1[10],aluIn1[11],aluIn1[12],aluIn1[13],aluIn1[14],aluIn1[15],
	 aluIn1[16],aluIn1[17],aluIn1[18],aluIn1[19],aluIn1[20],aluIn1[21],aluIn1[22],aluIn1[23],
	 aluIn1[24],aluIn1[25],aluIn1[26],aluIn1[27],aluIn1[28],aluIn1[29],aluIn1[30],aluIn1[31]
	} : aluIn1;
// Arithmetic shift right (sign preserved)
assign signed_sh = (opreg | opimm) & (funct3==3'h5) & funct7[5];
// The five layers of the barrel shifter
wire [31:0]sh_t1;
wire [31:0]sh_t2;
wire [31:0]sh_t3;
wire [31:0]sh_t4;
wire [31:0]sh_t5;
assign sh_t1 = aluIn2[4] ? {{16{ signed_sh & sh_in[31]}},sh_in[31:16]} : sh_in;
assign sh_t2 = aluIn2[3] ? {{ 8{ signed_sh & sh_t1[31]}},sh_t1[31: 8]} : sh_t1;
assign sh_t3 = aluIn2[2] ? {{ 4{ signed_sh & sh_t2[31]}},sh_t2[31: 4]} : sh_t2;
assign sh_t4 = aluIn2[1] ? {{ 2{ signed_sh & sh_t3[31]}},sh_t3[31: 2]} : sh_t3;
assign sh_t5 = aluIn2[0] ? {{  { signed_sh & sh_t4[31]}},sh_t4[31: 1]} : sh_t4;

// reverse the bit order at the output for left shifts
assign sh_out = left_sh ? 
	{sh_t5[ 0],sh_t5[ 1],sh_t5[ 2],sh_t5[ 3],sh_t5[ 4],sh_t5[ 5],sh_t5[ 6],sh_t5[ 7],
	 sh_t5[ 8],sh_t5[ 9],sh_t5[10],sh_t5[11],sh_t5[12],sh_t5[13],sh_t5[14],sh_t5[15],
	 sh_t5[16],sh_t5[17],sh_t5[18],sh_t5[19],sh_t5[20],sh_t5[21],sh_t5[22],sh_t5[23],
	 sh_t5[24],sh_t5[25],sh_t5[26],sh_t5[27],sh_t5[28],sh_t5[29],sh_t5[30],sh_t5[31]
	} : sh_t5;

  /////////////////////////////////////////////////
  // ALU output select
  /////////////////////////////////////////////////

wire [31:0] aluOut =
     (((funct3==0)|oplui|opauipc) ? aluAdder[31:0]  : 32'b0) |
     ((funct3==2)                 ? {31'b0, LT}     : 32'b0) |
     ((funct3==3)                 ? {31'b0, LTU}    : 32'b0) |
     ((funct3==4)                 ? aluIn1 ^ aluIn2 : 32'b0) |
     ((funct3==6)                 ? aluIn1 | aluIn2 : 32'b0) |
     ((funct3==7)                 ? aluIn1 & aluIn2 : 32'b0) |
     (((funct3==1)|(funct3==5))   ? sh_out          : 32'b0) ;

  //////////////////////////////////////////
  // The predicate for conditional branches.
  //////////////////////////////////////////
wire [7:0]prlist = {
	~LTU,	// 7: BGEU
	LTU,	// 6: BLTU
	~LT,	// 5: BGE
	LT,		// 4: BLT
	2'b00,	// 3,2: not used
	~EQ,	// 1: BNE
	EQ		// 0: BEQ
};
wire predicate = prlist[funct3];

//////////////////////////////////////////
// LOAD/STORE
//////////////////////////////////////////
// Address come from the adder in the ALU (rs1 + Imm)
// All memory accesses are aligned on 32 bits boundary. For this
// reason, we need some circuitry that does unaligned halfword
// and byte load/store, based on:
// - funct3[1:0]:  00->byte 01->halfword 10->word
// - aluAdder[1:0]: indicates which byte/halfword is accessed

   wire mem_byteAccess     =  funct3[1:0] == 2'b00;
   wire mem_halfwordAccess =  funct3[1:0] == 2'b01;

// address output              
//assign addr=(opload|opstore) ? aluAdder[31:2] : PC[31:2];
// a little bit faster...
wire [31:0]ldstaddr= regsQ1 + (opload? Iimm : Simm);
assign addr=(opload|opstore) ? ldstaddr[31:2] : PC;

// LOAD, in addition to funct3[1:0], LOAD depends on:
// - funct3[2]: 0->do sign expansion   1->no sign expansion

   wire LOAD_sign =
	(!funct3[2]) & (mem_byteAccess ? LOAD_byte[7] : LOAD_halfword[15]);

   wire [31:0] LOAD_data =
         mem_byteAccess ? {{24{LOAD_sign}},     LOAD_byte} :
     mem_halfwordAccess ? {{16{LOAD_sign}}, LOAD_halfword} :
                          rdata ;

   wire [15:0] LOAD_halfword =
	       aluAdder[1] ? rdata[31:16] : rdata[15:0];

   wire  [7:0] LOAD_byte =
	       aluAdder[0] ? LOAD_halfword[15:8] : LOAD_halfword[7:0];

// STORE

assign wdata[ 7: 0] = regsQ2[7:0];
assign wdata[15: 8] = aluAdder[0] ? regsQ2[7:0]  : regsQ2[15: 8];
assign wdata[23:16] = aluAdder[1] ? regsQ2[7:0]  : regsQ2[23:16];
assign wdata[31:24] = aluAdder[0] ? regsQ2[7:0]  :
			     aluAdder[1] ? regsQ2[15:8] : regsQ2[31:24];

// The memory write mask:
assign wstrb  = opstore ? 
	      mem_byteAccess      ?
	            (aluAdder[1] ?
		          (aluAdder[0] ? 4'b1000 : 4'b0100) :
		          (aluAdder[0] ? 4'b0010 : 4'b0001)
                    ) :
	      mem_halfwordAccess ?
	            (aluAdder[1] ? 4'b1100 : 4'b0011) :
              4'b1111:
              4'b0000;

///////////////////////////////////////////////
// The value written back to the register file.
///////////////////////////////////////////////

assign regsD  =
	((opreg | opimm | opauipc | oplui) ? aluOut     : 32'b0) |
	(opload                            ? LOAD_data  : 32'b0) |
	((opjal | opjalr)                  ? {PC,2'b00} : 32'b0) |
	(csrrw_mepc                        ? {PCreg0,2'b00} : 32'b0);

//////////////////////////////////////////////////////
///////////////////      PC     //////////////////////
//////////////////////////////////////////////////////
// The 2 LSB bits of PC are always 00, and aren't implemented
// PC points one instruction ahead of the one being executed (4 bytes)
// Two register stack: PC[0]: normal mode (user)
//                     PC[1]: interrupts (machine)
// Additional register PCci:  address of current instruction 

reg  [31:2]PCreg0; reg  [31:2]PCreg1;	// The Two PCs
wire [31:2]PC=mmode ? PCreg1 : PCreg0;	// Current mode PC
wire [31:2]PCimm=			// Immediate value to add to PC	
	((opbranch & predicate) ? Bimm[31:2] : 30'h0) |
	(opjal                  ? Jimm[31:2] : 30'h0);

// Next PC logic
wire [31:2]PCadd1= PC+(irqstart | opload | opstore ? 0 : 1); // Incremented/Same PC
wire [31:2]PCadd2= PCci+PCimm;								 // Jump address
wire [31:2]PCnext= opjalr ? aluAdder[31:2] : (jump? PCadd2 : PCadd1);

always @(posedge clk or posedge reset) begin
	if (reset) begin PCreg0<=(RESET_PC>>2); end
	else begin
		if (!mmode) PCreg0<= PCnext;
		else if (csrrw_mepc) PCreg0<=regsQ1[31:2]; 
		PCreg1<=(mmode&(~mret))? PCnext : ivector;
	end
end

// The PC value last cycle = PC of current instruction
reg [31:2]PCci;	
always @(posedge clk) PCci<=PC;

///////////////////////////////////////////////
// Interrupts:
// 	IRQ sequencer logic taken from GUS16
//////////////////////////////////////////////

reg q0=0;				// First FF (samples IRQ)
reg mmode=0;			// Machine mode: 0=normal, 1=IRQ 
wire ireti=mret&(~irq);	// Return from interrupt if no more IRQs pending
assign trap=ecall|ebreak;	// ECALL or EBREAK executed

always @(posedge clk or posedge reset ) 
	begin
	if (reset) begin 
		q0<=1'b0;
		mmode<=1'b0;
	end
	else begin
		q0 <= (~ireti) & (q0 | irq);
		mmode <= (~ireti) & (q0|mmode|trap);
	end
end

wire irqstart = (~mmode) & (q0|trap) ; // Single cycle pulse

assign usermode = ~mmode;

endmodule

///////////////////////////////////////////////
// JTAG port: J. Arias (2025)
// 	
//////////////////////////////////////////////

module JTAG_TAP (
	input tck,
	input reset,		
	input tms,
	input tdi,
	output tdo,
	//output [3:0]tapst, // debug
	input  [BSLEN-1:0]bsd,
	output reg [BSLEN-1:0]bsq,
	output extest,
	output intest,
	output streset	
);
parameter BSLEN=26;

// Sampled TDI & TMS (rising clock)
reg stdi;
reg stms;
always @(posedge tck) {stdi,stms}<={tdi,tms};

// TAP state machine
localparam RESET 	=0;
localparam IDLE  	=8;
localparam DRSCAN	=1;
localparam DRCAPTURE=2;
localparam DRSHIFT	=3;
localparam DREXIT1	=4;
localparam DRPAUSE	=5;
localparam DREXIT2	=6;
localparam DRUPDATE	=7;
localparam IRSCAN	=9;
localparam IRCAPTURE=10;
localparam IRSHIFT	=11;
localparam IREXIT1	=12;
localparam IRPAUSE	=13;
localparam IREXIT2	=14;
localparam IRUPDATE	=15;

reg [3:0]tapst;

always @(negedge tck or posedge reset)
	if (reset) tapst<=RESET;
	else case(tapst)
	RESET:		tapst<= stms ? RESET	: IDLE;
	IDLE:		tapst<= stms ? DRSCAN	: IDLE;
	DRSCAN:		tapst<= stms ? IRSCAN	: DRCAPTURE; 
	DRCAPTURE:	tapst<= stms ? DREXIT1	: DRSHIFT; 
	DRSHIFT:	tapst<= stms ? DREXIT1	: DRSHIFT;
	DREXIT1:	tapst<= stms ? DRUPDATE	: DRPAUSE;
	DRPAUSE:	tapst<= stms ? DREXIT2	: DRPAUSE; 
	DREXIT2:	tapst<= stms ? DRUPDATE	: DRSHIFT; 
	DRUPDATE:	tapst<= stms ? DRSCAN	: IDLE; 
	IRSCAN:		tapst<= stms ? RESET	: IRCAPTURE; 
	IRCAPTURE:	tapst<= stms ? IREXIT1	: IRSHIFT; 
	IRSHIFT:	tapst<= stms ? IREXIT1	: IRSHIFT;
	IREXIT1:	tapst<= stms ? IRUPDATE	: IRPAUSE;
	IRPAUSE:	tapst<= stms ? IREXIT2	: IRPAUSE; 
	IREXIT2:	tapst<= stms ? IRUPDATE	: IRSHIFT; 
	IRUPDATE:	tapst<= stms ? DRSCAN	: IDLE; 
	endcase

	
// IR register
parameter IRLEN=3;
parameter IDCODE          =3'b000;
parameter SAMPLE_PRELOAD  =3'b001;
parameter EXTEST          =3'b010;
parameter INTEST          =3'b011;
parameter BYPASS          =3'b111;

reg [IRLEN-1:0]irsh;	// shift reg
reg [IRLEN-1:0]ir;		// output latch

assign streset=(tapst==RESET)|(tapst==IDLE);
always @(negedge tck)
	if (tapst==IRSHIFT) irsh<={stdi,irsh[IRLEN-1:1]};
always @(negedge tck or posedge reset)
	if (reset) ir<=IDCODE;
	else if (tapst==RESET) ir<=IDCODE;
		else if (tapst==IRUPDATE) ir<=irsh; 

wire sel_id=(ir==IDCODE);
wire sel_bypass=(ir==BYPASS);
wire sel_sample=(ir==SAMPLE_PRELOAD);
wire sel_main= (~sel_id)&(~sel_bypass);

assign extest=(ir==EXTEST);
assign intest=(ir==INTEST);

// ID register
parameter IDVAL=32'h00047FAB;
//parameter IDVAL=32'h0x05B4603F; // Atmel ARM926EJ-S
reg [31:0]idr;
always @(negedge tck)
	idr <= (tapst==DRCAPTURE) ? IDVAL : ((tapst==DRSHIFT) ? {stdi,idr[31:1]} : idr);

// Bypass register
reg byp;
always @(negedge tck) if (sel_bypass) byp<=stdi;

// Main scan chain
reg [BSLEN-1:0]bssh;
//reg [BSLEN-1:0]bsq;
always @(negedge tck)
	bssh <= (tapst==DRCAPTURE) ? bsd : 
			((tapst==DRSHIFT)  ? {stdi,bssh[BSLEN-1:1]} : bssh);
always @(negedge tck)
	if (tapst==DRUPDATE) bsq<=bssh; 

// TDO select
assign tdo = (tapst==IRSHIFT) ? irsh[0] : 
				(sel_id ? idr[0] : (sel_bypass ? byp : bssh[0] ));

endmodule





