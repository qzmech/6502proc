#include <iostream>

using namespace std;

uint8_t A = 0x0; // accumulator
uint8_t X = 0x0; // index register
uint8_t Y = 0x0; // index register
uint8_t S = 0xFF; // stack pointer

bool C = false; // carry flag
bool Z = false; // zero flag
bool I = false; // interrupt flag
bool D = false; // BCD flag
bool B = false; // BRK flag
bool V = false; // overflow flag
bool N = false; // negative flag

uint16_t PC = 0x00; // next instruction address

uint8_t cpu_memory[UINT16_MAX]; // cpu memory bank

/*
$0000-$07FF RAM
$0800-$1FFF RAM Mirror (x3)
$2000-$2007 Registers Video
$2008-$3FFF Registrers Video Mirror (x1023)
$4000-$4017 Registers Audio & DMA & I/O
$4018-$4FFF Not used
$5000-$5FFF Expansion ROM (for MMC5)
$6000-$7FFF SRAM (WRAM)
$8000-$BFFF PRG-ROM (1)
$C000-$FFFF PRG-ROM (0)
*/


uint8_t clock = 0x00; // clock counter

void (*instructions_set[16][16])(uint8_t*, uint8_t) = {
	//				x0			 x1			x2			x3			x4			x5			x6			x7			x8			x9			xA			xB			xC			xD			xE			xF
	/* 0x */	{ _BRK(),		_ORA(),		ILL(),		ILL(),		ILL(),		_ORA(),		_ASL(),		ILL(),		_PHP(),		_ORA(),		_ASL(),		ILL(),		ILL(),		_ORA(),		_ASL(),		ILL()},		/* 0x */
	
	/* 1x */	{ _BPL(),		_ORA(),		ILL(),		ILL(),		ILL(),		_ORA(),		_ASL(),		ILL(),		_CLC(),		_ORA(),		ILL(),		ILL(),		ILL(),		_ORA(),		_ASL(),		ILL()},		/* 1x */
	
	/* 2x */	{ _JSR(),		_AND(),		ILL(),		ILL(),		_BIT(),		_AND(),		_ROL(),		ILL(),		_PLP(),		_AND(),		_ROL(),		ILL(),		_BIT(),		_AND(),		_ROL(),		ILL()},		/* 2x */
	
	/* 3x */	{ _BMI(),		_AND(),		ILL(),		ILL(),		ILL(),		_AND(),		_ROL(),		ILL(),		_SEC(),		_AND(),		ILL(),		ILL(),		ILL(),		_AND(),		_ROL(),		ILL()},		/* 3x */
	
	/* 4x */	{ _RTI(),		_EOR(),		ILL(),		ILL(),		ILL(),		_EOR(),		_LSR(),		ILL(),		_PHA(),		_EOR(),		_LSR(),		ILL(),		_JMP(),		_EOR(),		_LSR(),		ILL()},		/* 4x */
	
	/* 5x */	{ _BVC(),		_EOR(),		ILL(),		ILL(),		ILL(),		_EOR(),		_LSR(),		ILL(),		_CLI(),		_EOR(),		ILL(),		ILL(),		ILL(),		_EOR(),		_LSR(),		ILL()},		/* 5x */
	
	/* 6x */	{ _RTS(),		_ADC(),		ILL(),		ILL(),		ILL(),		_ADC(),		_ROR(),		ILL(),		_PLA(),		_ADC(),		_ROR(),		ILL(),		_JMP(),		_ADC(),		_ROR(),		ILL()},		/* 6x */
	
	/* 7x */	{ _BVS(),		_ADC(),		ILL(),		ILL(),		ILL(),		_ADC(),		_ROR(),		ILL(),		_SEI(),		_ADC(),		ILL(),		ILL(),		ILL(),		_ADC(),		_ROR(),		ILL()},		/* 7x */
	
	/* 8x */	{  ILL(),		_STA(),		ILL(),		ILL(),		_STY(),		_STA(),		_STX(),		ILL(),		_DEY(),		ILL(),		_TXA(),		ILL(),		_STY(),		_STA(),		_STX(),		ILL()},		/* 8x */
	
	/* 9x */	{ _BCC(),		_STA(),		ILL(),		ILL(),		_STY(),		_STA(),		_STX(),		ILL(),		_TYA(),		_STA(),		_TXS(),		ILL(),		ILL(),		_STA(),		ILL(),		ILL()},		/* 9x */
	
	/* Ax */	{ _LDY(),		_LDA(),		_LDX(),		ILL(),		_LDY(),		_LDA(),		_LDX(),		ILL(),		_TAY(),		_LDA(),		_TAX(),		ILL(),		_LDY(),		_LDA(),		_LDX(),		ILL()},		/* Ax */
	
	/* Bx */	{ _BCS(),		_LDA(),		ILL(),		ILL(),		_LDY(),		_LDA(),		_LDX(),		ILL(),		_CLV(),		_LDA(),		_TSX(),		ILL(),		_LDY(),		_LDA(),		_LDX(),		ILL()},		/* Bx */
	
	/* Cx */	{ _CPY(),		_CMP(),		ILL(),		ILL(),		_CPY(),		_CMP(),		_DEC(),		ILL(),		_INY(),		_CMP(),		_DEX(),		ILL(),		_CPY(),		_CMP(),		_DEC(),		ILL()},		/* Cx */
	
	/* Dx */	{ _BNE(),		_CMP(),		ILL(),		ILL(),		ILL(),		_CMP(),		_DEC(),		ILL(),		_CLD(),		_CMP(),		ILL(),		ILL(),		ILL(),		_CMP(),		_DEC(),		ILL()},		/* Dx */
		
	/* Ex */	{ _CPX(),		_SBC(),		ILL(),		ILL(),		_CPX(),		_SBC(),		_INC(),		ILL(),		_INX(),		_SBC(),		_NOP(),		ILL(),		_CPX(),		_SBC(),		_INC(),		ILL()},		/* Ex */
	
	/* Fx */	{ _BEQ(),		_SBC(),		ILL(),		ILL(),		ILL(),		_SBC(),		_INC(),		ILL(),		_SED(),		_SBC(),		ILL(),		ILL(),		ILL(),		_SBC(),		_INC(),		ILL()}};	/* Fx */
	//				x0			x1			x2			x3			x4			x5			x6			x7			x8			x9			xA			xB			xC			xD			xE			xF



uint8_t clock_set[16][16] = {
	//				x0  x1	x2	x3	x4	x5	x6	x7	x8	x9	xA	xB	xC	xD	xE	xF
	/* 0x */	{	7,	6,	0,	8,	3,	3,	5,	5,	3,	2,	2,	2,	4,	4,	6,	6	},	/* 0x */
	/* 1x */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	4,	2,	7,	4,	4,	7,	7	},	/* 1x */
	/* 2x */	{	6,	6,	0,	8,	3,	3,	5,	5,	4,	2,	2,	2,	4,	4,	6,	6	},	/* 2x */
	/* 3x */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	4,	2,	7,	4,	4,	7,	7	},	/* 3x */
	/* 4x */	{	6,	6,	0,	8,	3,	3,	5,	5,	3,	2,	2,	2,	3,	4,	6,	6	},	/* 4x */
	/* 5x */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	4,	2,	7,	4,	4,	7,	7	},	/* 5x */
	/* 6x */	{	6,	6,	0,	8,	3,	3,	5,	5,	4,	2,	2,	2,	5,	4,	6,	6	},	/* 6x */
	/* 7x */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	2,	2,	7,	4,	4,	7,	7	},	/* 7x */
	/* 8x */	{	2,	6,	2,	6,	3,	3,	3,	3,	2,	2,	2,	2,	4,	4,	4,	4	},	/* 8x */
	/* 9x */	{	2,	6,	0,	6,	4,	4,	4,	4,	2,	5,	2,	5,	5,	5,	5,	5	},	/* 9x */
	/* Ax */	{	2,	6,	2,	6,	3,	3,	3,	3,	2,	2,	2,	2,	4,	4,	4,	4	},	/* Ax */
	/* Bx */	{	2,	5,	0,	5,	4,	4,	4,	4,	2,	4,	2,	4,	4,	4,	4,	4	},	/* Bx */
	/* Cx */	{	2,	6,	2,	8,	3,	3,	5,	5,	2,	2,	2,	2,	4,	4,	6,	6	},	/* Cx */
	/* Dx */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	4,	2,	7,	4,	4,	7,	7	},	/* Dx */
	/* Ex */	{	2,	6,	2,	8,	3,	3,	5,	5,	2,	2,	2,	2,	4,	4,	6,	6	},	/* Ex */
	/* Fx */	{	2,	5,	0,	8,	4,	4,	6,	6,	2,	4,	2,	7,	4,	4,	7,	7 	}};	/* Fx */
	//				x0  x1	x2	x3	x4	x5	x6	x7	x8	x9	xA	xB	xC	xD	xE	xF




// flag update functions

void update_Z(uint8_t reg) {
	Z = (reg == 0x0);
}

void update_N(uint8_t reg) {
	N = (reg >> 7);
}

void update_C(uint16_t reg) {
	C = (reg & 0x100);
}



// addressing modes

void RELATIVE() {
	PC += (int8_t) (cpu_memory[PC + 1] + 2);
}

uint8_t* IMMEDIATE() {
	return &cpu_memory[PC + 1];
}

uint8_t* ABSOLUTE() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	return &cpu_memory[data];
}

uint8_t* ABSOLUTE_X() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	data += X;
	return &cpu_memory[data];
}

uint8_t* ABSOLUTE_Y() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	data += Y;
	return &cpu_memory[data];
}

uint8_t* ZERO_PAGE() {
	uint8_t data = cpu_memory[PC + 1];
	return &cpu_memory[data];
}

uint8_t* ZERO_PAGE_X() {
	uint8_t data = cpu_memory[PC + 1];
	data += X;
	return &cpu_memory[data];
}

uint8_t* ZERO_PAGE_Y() {
	uint8_t data = cpu_memory[PC + 1];
	data += Y;
	return &cpu_memory[data];
}

uint8_t* INDIRECT() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	return &cpu_memory[data];
}

uint8_t* INDIRECT_X() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	data += X;
	return &cpu_memory[data];
}

uint8_t* INDIRECT_Y() {
	uint16_t data = cpu_memory[PC + 2];
	data << 8;
	data |= cpu_memory[PC + 1];
	data += Y;
	return &cpu_memory[data];
}





void ILL() {
	throw new exception("ILLEGAL COMMAND");
}

//Opcodes implementation

//Load Accumulator with Memory
void _LDA(uint8_t* operand, uint8_t command_length) {
	A = *operand;

	update_Z(A);
	update_N(A);

	PC += command_length;
}

//Load Index X with Memory
void _LDX(uint8_t* operand, uint8_t command_length) {
	X = *operand;

	update_Z(X);
	update_N(X);

	PC += command_length;
}

//Load Index Y with Memory
void _LDY(uint8_t* operand, uint8_t command_length) {
	Y = *operand;

	update_Z(Y);
	update_N(Y);

	PC += command_length;
}

//Store Accumulator in Memory
void _STA(uint8_t* operand, uint8_t command_length) {
	*operand = A;

	PC += command_length;
}

//Store Index X in Memory
void _STX(uint8_t* operand, uint8_t command_length) {
	*operand = X;

	PC += command_length;
}

//Store Index Y in Memory
void _STY(uint8_t* operand, uint8_t command_length) {
	*operand = Y;

	PC += command_length;
}

//Transfer Accumulator to Index X
void _TAX() {
	X = A;

	update_Z(X);
	update_N(X);

	PC++;
}

//Transfer Accumulator to Index Y 
void _TAY() {
	Y = A;

	update_Z(Y);
	update_N(Y);

	PC++;
}

//Transfer Index X to Accumulator
void _TXA() {
	A = X;

	update_Z(A);
	update_N(A);

	PC++;
}

//Transfer Index Y to Accumulator
void _TYA() {
	A = Y;

	update_Z(A);
	update_N(A);

	PC++;
}

//Transfer Stack Pointer to Index X
void _TSX() {
	X = S;

	update_Z(X);
	update_N(X);

	PC++;
}

//Transfer Index X to Stack Pointer
void _TXS() {
	S = X;

	update_Z(S);
	update_N(S);

	PC++;
}

//Push Accumulator on Stack
void _PHA() {
	uint16_t stack_address = 0x0100 + S;
	cpu_memory[stack_address] = A;
	?
	S--;
	PC++;
}

//Pull Accumulator from Stack
void _PLA() {
	uint16_t stack_address = 0x0100 + S;
	A = cpu_memory[stack_address];
	?
	S++;
	PC++;
}

//Push Processor Status on Stack
void _PHP() {
	uint16_t stack_address = 0x0100 + S;
	?
	cpu_memory[stack_address] = N;
	cpu_memory[stack_address] <<= 1;
	cpu_memory[stack_address] |= V;
	cpu_memory[stack_address] <<= 2;
	cpu_memory[stack_address] |= B;
	cpu_memory[stack_address] <<= 1;
	cpu_memory[stack_address] |= D;
	cpu_memory[stack_address] <<= 1;
	cpu_memory[stack_address] |= I;
	cpu_memory[stack_address] <<= 1;
	cpu_memory[stack_address] |= Z;
	cpu_memory[stack_address] <<= 1;
	cpu_memory[stack_address] |= C;

	S--;
	PC++;
}

//Pull Processor Status from Stack
void _PLP() {
	uint16_t stack_full_address = 0x0100 + S;
	?
	N = (cpu_memory[stack_full_address] & 0b10000000) >> 7;
	V = (cpu_memory[stack_full_address] & 0b01000000) >> 6;
	B = (cpu_memory[stack_full_address] & 0b00010000) >> 4;
	D = (cpu_memory[stack_full_address] & 0b00001000) >> 3;
	I = (cpu_memory[stack_full_address] & 0b00000100) >> 2;
	Z = (cpu_memory[stack_full_address] & 0b00000010) >> 1;
	C = (cpu_memory[stack_full_address] & 0b00000001);

	S++;
	PC++;
}

//"AND" Memory with Accumulator
void _AND(uint8_t* operand, uint8_t command_length) {
	A &= *operand;

	update_Z(A);
	update_N(A);

	PC += command_length;
}

//"Exclusive-Or" Memory with Accumulator
void _EOR(uint8_t* operand, uint8_t command_length) {
	A ^= *operand;

	update_Z(A);
	update_N(A);

	PC += command_length;
}

//"OR" Memory with Accumulator
void _ORA(uint8_t* operand, uint8_t command_length) {
	A |= *operand;
	
	update_Z(A);
	update_N(A);
	
	PC += command_length;
}

//Test Bits in Memory with Accumulator
void _BIT(uint8_t* operand, uint8_t command_length) {
	update_Z(A & *operand);
	N = (*operand >> 7) & 0x1;
	V = (*operand >> 6) & 0x1;

	PC += command_length;
}

//Add Memory to Accumulator with Carry
void _ADC(uint8_t* operand, uint8_t command_length) {
	uint16_t data = A + C + *operand;
	?
	V = ~((A ^ *operand) & 0x80) && ((A ^ *operand) & 0x80);
	A = data;

	update_C(data);
	update_Z(A);
	update_N(A);

	PC += command_length;
}

//Subtract Memory from Accumulator with Borrow
void _SBC(uint8_t* operand, uint8_t command_length) {
	uint8_t data = A - (C ? 0 : 1) - *operand;
	?
	V =  ~((A ^ *operand) & 0x80) && ((A ^ *operand) & 0x80);
	A = data;

	update_C(data);
	update_Z(A);
	update_N(A);

	PC += command_length;
}

//Compare Memory and Accumulator
void _CMP(uint8_t* operand, uint8_t command_length) {
	uint16_t data = A - *operand;

	update_C(data);
	update_Z(data);
	update_N(data);

	PC += command_length;
}

//Compare Memory and Index X
void _CPX(uint8_t* operand, uint8_t command_length) {
	uint16_t data = X - *operand;

	update_C(data);
	update_Z(data);
	update_N(data);	
	
	PC += command_length;
}

//Compare Memory and Index Y
void _CPY(uint8_t* operand, uint8_t command_length) {
	uint16_t data = Y - *operand;

	update_C(data);
	update_Z(data);
	update_N(data);	
	
	PC += command_length;
}

//Increment Memory by One
void _INC(uint8_t* operand, uint8_t command_length) {
	*operand++;

	update_Z(*operand);
	update_N(*operand);

	PC += command_length;
}

//Increment Index X by One
void _INX() {
	X++;

	update_Z(X);
	update_N(X);
}

//Increment Index Y by One
void _INY() {
	Y++;

	update_Z(Y);
	update_N(Y);
}

//Decrement Memory by One
void _DEC(uint8_t* operand, uint8_t command_length) {
	*operand--;

	update_Z(*operand);
	update_N(*operand);

	PC += command_length;
}

//Decrement Index X by One
void _DEX() {
	X--;

	update_Z(X);
	update_N(X);
}

//Decrement Index Y by One
void _DEY() {
	Y--;

	update_Z(Y);
	update_N(Y);
}

//Shift Left One Bit(Memory or Accumulator)
void _ASL(uint8_t* operand, uint8_t command_length) {
	C = (*operand >> 7);
	*operand <<= 1;

	update_Z(*operand);
	update_N(*operand);
	
	PC += command_length;
}

//Shift Right One Bit (Memory or Accumulator)
void _LSR(uint8_t* operand, uint8_t command_length) {
	C = (*operand & 0x1);
	*operand >>= 1;

	update_Z(*operand);	
	N = 0;

	PC += command_length;
}

//Rotate One Bit Left (Memory or Accumulator)
void _ROL(uint8_t* operand, uint8_t command_length) {
	C = (*operand >> 7);
	*operand <<= 1;
	*operand |= C;

	update_Z(*operand);
	update_N(*operand);

	PC += command_length;
}

//Rotate One Bit Right (Memory or Accumulator)
void _ROR(uint8_t* operand, uint8_t command_length) {
	C = (*operand << 7);
	*operand >>= 1;
	*operand |= C;
	C >>= 7;

	update_Z(*operand);
	update_N(*operand);
	
	PC += command_length;
}

//Branch on Carry Clear
void _BCC() {
	if (C == 0) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Carry Set
void _BCS() {
	if (C == 1) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Result not Zero
void _BNE() {
	if (Z == 0) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Result Zero
void _BEQ() {
	if (Z == 1) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Result Plus
void _BPL() {
	if (N == 0) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Result Minus
void _BMI() {
	if (N == 1) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Overflow Clear
void _BVC() {
	if (V == 0) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Branch on Overflow Set
void _BVS() {
	if (V == 1) {
		RELATIVE();
		return;
	}
	PC += 2;
}

//Clear Carry Flag
void _CLC() {
	C = false;
	PC++;
}

//Clear Decimal Mode
void _CLD() {
	D = false;
	PC++;
}

//Clear interrupt Disable Bit
void _CLI() {
	I = false;
	PC++;
}

//Clear Overflow Flag
void _CLV() {
	V = false;
	PC++;
}

//Set Carry Flag
void _SEC() {
	C = true;
	PC++;
}

//Set Decimal Mode
void _SED() {
	D = true;
	PC++;
}

//Set Interrupt Disable Status
void _SEI() {
	I = true;
	PC++;
}

//No Operation
void _NOP() {
	PC++;
}

//Force Break
void _BRK() {
	?
	PC++;
	cpu_memory[0x100 + S] = ((PC >> 8) & 0xff);
	S++;
	cpu_memory[0x100 + S] = (PC & 0xff);
	S++;
	B = 1;
	cpu_memory[0x100 + S] = S;
	S++;
	I = 1;
	PC = (cpu_memory[0xFFFE] | (cpu_memory[0xFFFF] << 8));
}

//Jump to New Location
void _JMP(uint8_t* operand) {
	?
	PC += *operand;
}

//Jump to New Location Saving Return Address
void _JSR(uint8_t type_addressing) {
	?
	PC--;
	cpu_memory[0x100 + S] = ((PC >> 8) & 0xff);
	S--;
	cpu_memory[0x100 + S] = (PC & 0xff);
	S--;
	PC = cpu_memory[PC + 2];
	PC <<= 8;
	PC |= cpu_memory[PC + 1];
}

//Return from Subroutine
void _RTS() {
	?
	S++;
	PC = cpu_memory[0x100 + S];
	S++;
	PC += (cpu_memory[0x100 + S] << 8) + 1;
}

//Return from Interrupt
void _RTI() {
	?
	S++;
	S = cpu_memory[0x100 + S];
	S++;
	PC = cpu_memory[0x100 + S];
	S++;
	PC |= cpu_memory[0x100 + S] << 8;
}

void nmi() {
	?
	cpu_memory[0x0100 + S] = (PC >> 8) & 0x00FF;
	S--;
	cpu_memory[0x0100 + S] = PC & 0x00FF;
	S--;

	B = 0;
	I = 1;

	uint16_t LO = cpu_memory[0xFFFA + 0];
	uint16_t HI = cpu_memory[0xFFFA + 1];

	PC = (HI << 8) | LO;

	clock = 8;
}

void reset() {
	?
	uint16_t LO = cpu_memory[0xFFFC + 0];
	uint16_t HI = cpu_memory[0xFFFC + 1];

	PC = (HI << 8) | LO;

	A = X = Y = 0;
	S = 0xFD;

	clock = 8;
}

void irq() {
	?
	if (I == 0) {
		return;
	}

	cpu_memory[0x100 + S] = (PC >> 8) & 0x00FF;
	S--;
	cpu_memory[0x100 + S] = PC & 0x00FF;
	S--;

	B = 0;
	I = 1;

	uint16_t LO = cpu_memory[0xFFFE + 0];
	uint16_t HI = cpu_memory[0xFFFE + 1];

	PC = (HI << 8) | LO;

	clock = 7;
}

int main() {
}