

void dissasemble(uint32_t op, uint32_t pc){
	uint32_t opcode, rd, funct3, rs1, rs2, funct7, imm;
	int i;
	static const uint8_t *rtypen[8]={"ADD   ","SLL   ","SLT   ","SLTU  ",
									 "XOR   ","SRL   ","OR    ","AND   "};
	static const uint8_t *itype1n[8]={"ADDI  ","SLLI  ","SLTI  ","SLTIU ",
									  "XORI  ","SRLI  ","ORI   ","ANDI  "};
	static const uint8_t *itype2n[6]={"LB    ","LH    ","LW    ","?     ",
									  "LBU   ","LHU   "};
	static const uint8_t *stypen[3]={"SB    ","SH    ","SW    "};
	static const uint8_t *btypen[8]={"BEQ   ","BNE   ","?     ","?     ",
									 "BLT   ","BGE   ","BLTU  ","BGEU  "};
	
	opcode=op&0x7F;
	rd=(op>>7)&0x1F;
	funct3=(op>>12)&7;
	rs1=(op>>15)&0x1F;
	rs2=(op>>20)&0x1F;
	funct7=op>>25;
	imm=op>>20; if (imm&(1<<11)) imm|=0xFFFFF000;
	
	switch(opcode) {
	case 0x33:	// R-type
		if (funct7==0x20) {
			if (funct3==0) _puts("SUB   ");
			else if (funct3==0x5) _puts("SRA   ");
				else goto unk_op;
		} else {
			if (funct7!=0) goto unk_op;
			_puts(rtypen[funct3]);
		}
		_printf("  X%d, X%d, X%d\n",rd,rs1,rs2);
		break;
	
	case 0x13:	// I-type (I)
		if (funct3==5 && (imm>>5)) {_puts("SRAI  "); imm&=0x1F;}
		else _puts(itype1n[funct3]);
		_printf("  X%d, X%d, %d\n",rd,rs1,imm);
		break;	
		
	case 0x03:	// I-type (II) (Load)
		if (funct3>5) goto unk_op;
		_printf("%s  X%d, %d (X%d)\n",itype2n[funct3],rd,imm,rs1);
		break;
		
	case 0x23:	// S-type (store)
		imm=rd | ((op>>25)<<5); if (imm&(1<<11)) imm|=0xFFFFF000;
		if (funct3>2) goto unk_op;
		_printf("%s  X%d, %d (X%d)\n",stypen[funct3],rs2,imm,rs1);
		break;
		
	case 0x63:	// B-type (branches)
		imm=(rd&0x1E) |((op>>25)&0x3F)<<5;
		if (rd&1) imm|=(1<<11);
		if (op&(1<<31)) imm|=(1<<12);
		if (imm&(1<<12)) imm|=0xFFFFE000;
		pc+=imm;
		_printf("%s  X%d, X%d, %08x\n",btypen[funct3],rs1,rs2,pc);
		break;
		
	case 0x67:	// I-type (JALR)
		_printf("JALR    X%d, %d (X%d)\n",rd,imm,rs1);
		break;
		
	case 0x73:	// I-type (SYSTEM)
		if (op==0x30200073) {_puts("MRET\n"); break;}
		if (funct3==1) {_printf("CSRRW   X%d, %03x, X%d\n",rd,imm,rs1); break;}
		_puts(imm ? "EBREAK\n" : "ECALL\n");
		break;
	
	case 0x37:	// U-type (LUI)
		_printf("LUI     X%d, %05x\n",rd,(op>>12));
		break;
	
	case 0x17:	// U-type (AUIPC)
		_printf("AUIPC   X%d, %05x\n",rd,(op>>12));
		break;
	
	case 0x6F:	// J-type (JAL)
		imm=(op&(0xff<<12)) | ((op>>20)&0x7fe);
		if(op&(1<<20)) imm|=(1<<11);
		if(op&(1<<31)) imm|=(1<<20);
		if (imm&(1<<20)) imm|=0xFFE00000;
		pc+=imm;
		if(rd) _printf("JAL     X%d, %08x\n",rd,pc);
		else   _printf("J       %08x\n",pc);
		break;
		
	default:
unk_op:	_puts("?     \n");
		break;
	}
}
