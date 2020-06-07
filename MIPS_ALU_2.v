/*
	REFFERENCE 	: "Computer_Organization_and_Design_5th_Edi.pdf"
*/

//Program Counter
module PC(PC_in,PC_out,RESET,CLK);
	input		[31:0]	PC_in;
	output	reg	[31:0]	PC_out;
	input		RESET,CLK;
/*
initial begin
	PC_out 	= 32'h0;
end
*/
always @ (RESET)	begin
	PC_out	<=	32'h0;
end

always @ (negedge CLK) begin
	PC_out	<= PC_in;
end
endmodule

//program counter adder
module PC_ADDER(PC_in,PC_out);
	input		[31:0]	PC_out;
	output 	reg 	[31:0]	PC_in;
	//input	CLK;
	reg	[31:0]	PC_next;
initial begin
	PC_in = 32'h0;
end

always @ (PC_out)	begin
	PC_in	<= PC_out + 4;	
end
endmodule

//INSTRUCTION MEMORY
module INSTRUCTION_MEMORY(PC_out,INSTRUCTION);
	input 		[31:0]	PC_out;
	output	reg	[31:0]	INSTRUCTION;
	reg	[7:0]	memReg[1023:0];

initial begin
	INSTRUCTION	=	32'h0;
/*
	register0 + register1 => register2 ;
	op	:ADD 		-	000010
	rs	:register0	-	00000  
	rt	:register1 	-	00001
	rd	:register0 	-	00010
	shmt	:		-	00000
	funct	:		-	100000
	
	therefore::::
	INSTRUCTION		-	|000010 00|000 00001 |00010 000|00 100000
*/
	//ADD	
	memReg[0] = 8'b00000000; //8'h8; 
	memReg[1] = 8'b00000001;
	memReg[2] = 8'b00010000;
	memReg[3] = 8'b00100000;
	
	//AND	{register3 AND register4 => register5 ;}
	memReg[4] = 8'b00000000;
	memReg[5] = 8'b01100100;
	memReg[6] = 8'b00101000;
	memReg[7] = 8'b00100100;
	
	//OR	{register6 OR register7 => register8 ;}
	memReg[8] = 8'b00000001;
	memReg[9] = 8'b00100111;
	memReg[10] = 8'b01000000;
	memReg[11] = 8'b00100101;
	
	//SUB	{register9 - register10 => register11} ;
	memReg[12] = 8'b00000001;
	memReg[13] = 8'b00101010;
	memReg[14] = 8'b01011000;
	memReg[15] = 8'b00100010;
	
	//SLT	register12 < register13 => register14? ;
	memReg[16] = 8'b00000001;
	memReg[17] = 8'b10001101;
	memReg[18] = 8'b01110000;
	memReg[19] = 8'b00101010;
	//SLT	register18 < register19 => register20? ;
	/*
	memReg[24] = 8'b00000010;
	memReg[25] = 8'b01010011;
	memReg[26] = 8'b10100000;
	memReg[27] = 8'b00101010;
	*/
	//NOR	register15 NOR register16 => register17 ;
	memReg[20] = 8'b00000001;
	memReg[21] = 8'b11110000;
	memReg[22] = 8'b10001000;
	memReg[23] = 8'b00100111;

	//Load word  ;
	memReg[24] = 8'b10001111;
	memReg[25] = 8'b11111110;
	memReg[26] = 8'b00000000;
	memReg[27] = 8'b00001010;

	//store word
	memReg[28] = 8'b10101111;
	memReg[29] = 8'b10011111;
	memReg[30] = 8'b00000000;
	memReg[31] = 8'b00001000;

	//beq 
	memReg[32] = 8'b00100010;
	memReg[33] = 8'b11111000;
	memReg[34] = 8'b00000000;
	memReg[35] = 8'b00001010;
	
	//bneq
	memReg[36] = 8'b00100011;
	memReg[37] = 8'b00011001;
	memReg[38] = 8'b00000000;
	memReg[39] = 8'b00001010;
	
end
	always @ (PC_out) begin
		//memReg[PC_out] = memReg[PC_out] + 8'h1;
		INSTRUCTION[7:0] 	<= memReg[PC_out + 3];
		INSTRUCTION[15:8] 	<= memReg[PC_out + 2];
		INSTRUCTION[23:16] 	<= memReg[PC_out + 1];
		INSTRUCTION[31:24] 	<= memReg[PC_out];

	end
endmodule

// instruction register
module INSTRUCTION_REGISTER(INSTRUCTION,CLK,ReadReg1,ReadReg2,WriteReg0,FuncCode,opcode,signExtIn);
	input	[31:0]	INSTRUCTION;
	input		CLK;
	output	reg	[4:0]	ReadReg1,ReadReg2,WriteReg0;
	output	reg	[5:0]	FuncCode,opcode;
	output	reg	[15:0]	signExtIn;
	reg	[31:0]	CurrentINS;
initial begin
	ReadReg1	=	5'h0;
	ReadReg2	=	5'h0;	
	WriteReg0	=	5'h0;	
	FuncCode	=	6'h0;
	opcode		=	6'h0;
	signExtIn	<=	16'h0;
end
always @ (posedge	CLK) begin
	CurrentINS	<=	INSTRUCTION;
	ReadReg1	<=	INSTRUCTION[25:21];
	ReadReg2	<=	INSTRUCTION[20:16];
	WriteReg0	<=	INSTRUCTION[15:11];
	FuncCode	<=	INSTRUCTION[5:0];
	opcode		<=	INSTRUCTION[31:26];
	signExtIn	<=	INSTRUCTION[15:0];	
end
endmodule

//
//Register file
module REGISTERS(ReadReg1,ReadReg2,WriteReg,WriteData,RegWrite,CLK,A,ReadData2);
	input		[4:0]	ReadReg1,ReadReg2,WriteReg;
	input			CLK,RegWrite;
	input		[31:0]	WriteData;
	output	reg	[31:0]	A,ReadData2;
	
	//DEFINING REGISTERS
	reg		[31:0]	REGS[31:0];
initial begin
	A		=	32'h0;
	ReadData2	=	32'h0;
	// ASSIGN VALUES TO REGISTER 
	REGS[0]		=	32'h3;
	REGS[1]		=	32'h4;
	REGS[2]		=	32'h1;
	REGS[3]		=	32'h2;
	REGS[4]		=	32'h3;
	REGS[5]		=	32'h0;
	REGS[6]		=	32'h5;
	REGS[7]		=	32'ha;
	REGS[8]		=	32'h2;
	REGS[9]		=	32'hd;
	REGS[10]	=	32'ha;
	REGS[11]	=	32'h0;
	REGS[12]	=	32'h10;
	REGS[13]	=	32'hf;
	REGS[14]	=	32'h0;
	REGS[15]	=	32'h2;
	REGS[16]	=	32'h4;
	REGS[17]	=	32'h0;
	REGS[18]	=	32'h0;
	REGS[19]	=	32'h10;
	REGS[20]	=	32'hf;
	REGS[21]	=	32'h0;
	REGS[22]	=	32'h0;
	REGS[23]	=	32'ha;	//for branch equal
	REGS[24]	=	32'ha;	//for branch equal
	REGS[25]	=	32'hb;	//for branch not-equal
	REGS[26]	=	32'h0;
	REGS[27]	=	32'h0;
	REGS[28]	=	32'hc;	//for store
	REGS[29]	=	32'h0;
	REGS[30]	=	32'h0;
	REGS[31]	=	32'ha;	//for load
end
always @ (ReadReg1 or ReadReg2)	begin
	A		<=	REGS[ReadReg1];
	ReadData2	<=	REGS[ReadReg2];
end
always @ (WriteData) begin
	REGS[WriteReg]		<= WriteData;
end

endmodule

//ALU CONTROLER
module ALUControl(ALUOp,FuncCode,ALUCtl);
	input		[1:0]	ALUOp;
	input		[5:0]	FuncCode;
	output	reg	[3:0]	ALUCtl;

	always @(*)	begin
		case(ALUOp)	
			2 :case(FuncCode)
				32: ALUCtl <= 2;		// ADD
				34: ALUCtl <= 6;		// SUBSTRACT
				36: ALUCtl <= 0;		// AND
				37: ALUCtl <= 1;		// OR
				39: ALUCtl <= 12;		// NOR
				42: ALUCtl <= 7;		// SLT (Set Less Than)
				default: ALUCtl <= 15;	// Not happened
			endcase
			0 : ALUCtl <= 2;	// for lw/sw
		endcase
	end

endmodule

// MIPS ALU
module MIPSALU(ALUCtl,A,B,ALUOut,Zero);
	input 		[3:0] ALUCtl;
	input 		[31:0] A,B;
	output 	 reg	[31:0] ALUOut;
	output		Zero;

	assign Zero = (ALUOut==0);	// Zero is true if ALUOut is 0
	always @(ALUCtl,A,B) begin	// reevaluate if these change
		case (ALUCtl)
			0: ALUOut <= A & B;
			1: ALUOut <= A | B;
			2: ALUOut <= A + B;
			6: ALUOut <= A - B;
			7: ALUOut <= A < B ? 1 : 0;
			12: ALUOut <= ~(A | B);
			default: ALUOut <= 0;
		endcase
	end
endmodule


//central controller
module CONTROL(opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Zero);
	input	[5:0]	opcode;
	input		Zero;
	output	reg	[1:0]	ALUOp;
	output	reg	RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc,RegWrite;
	reg	[8:0]	outputCode;

initial begin
	outputCode = 9'h0;
end
/*
outputCode format :
[RegDst,ALUSrc,MemtoReg,RegWrite,MemRead,MemWrite,Branch,ALUOp1,ALUOp0]
*/
always @ (opcode)	begin
	case(opcode)
		//dont cares has implemented as zero	
		0 :	outputCode	<= 	9'b100100010;	//R-Type
		35: 	outputCode	<= 	9'b011110000;	//load word
		43:	outputCode	<= 	9'b010001000;	//store word
		8 :	outputCode	<= 	9'b000000101;
		default: outputCode	<= 	9'b100100010;	//R-Type	//default 
	endcase
	
	//assign to output registers
	RegDst		=	outputCode[8];
	ALUSrc		=	outputCode[7];
	MemtoReg	=	outputCode[6];
	RegWrite	=	outputCode[5];
	MemRead		=	outputCode[4];
	MemWrite	=	outputCode[3];
	Branch		=	outputCode[2];
	ALUOp		<=	outputCode[1:0];
	
end

endmodule


//signExtension unit
module SIGN_EXTENSION(signExtIn,signExtOut);
	input		[15:0]	signExtIn;
	output	reg	[31:0]	signExtOut;
initial begin
	signExtOut	= 32'h0;
end
always @ (*) begin
	signExtOut <= ~signExtIn + 1;
end
endmodule
//mux0 switch instruction[20:16] and  instruction[20:16] to write register
module MUX0(ReadReg2,WriteReg0,RegDst,WriteReg);
	input	[4:0]	ReadReg2;
	input	[4:0]	WriteReg0;
	input		RegDst;
	output	reg	[4:0]	WriteReg;
initial begin
	WriteReg	<=	4'h0;
end
always @ (*)	begin
	case(RegDst)
		0 :	WriteReg <=	ReadReg2;
		1 :	WriteReg <=	WriteReg0;
	endcase
end
endmodule

//mux1 switch readData2 and signExt to B in ALU
module MUX1(ReadData2,signExtOut,ALUSrc,B);
	input	[31:0]	ReadData2;
	input	[31:0]	signExtOut;
	input		ALUSrc;
	output	reg	[31:0]	B;
initial begin
	B	=	32'h0;
end
always @ (*)	begin
	case(ALUSrc)
		0 :	B	<=	ReadData2;
		1 :	B	<=	signExtOut;
	endcase
end 	
endmodule
//mux2 switch ALUout and DataMem(ReadData3) to write data in REGISTERS
module MUX2(ReadData3,ALUOut,WriteData,MemtoReg);
	input	[31:0]	ReadData3;
	input	[31:0]	ALUOut;
	input		MemtoReg;
	output	reg	[31:0]	WriteData;
initial begin
	WriteData	<=	32'h0;
end
always @ (*)	begin
	case(MemtoReg)
		0 :	WriteData	<=	ReadData3;
		1 :	WriteData	<=	ALUOut;
	endcase
end	
	
endmodule

//Data Memory
module DATAMEM(ALUOut,ReadData2,ReadData3,MemWrite,MemRead);
	input	[31:0]	ALUOut;			//Address for the DataMemory
	input	[31:0]	ReadData2;		//WriteData For DataMem
	input		MemWrite,MemRead;	//Control signals for DataMem
	output	reg	[31:0]	ReadData3;	//output port of the module
	//DEFINING REGISTERS
	reg		[7:0]	GPREGS[1048576:0];	//General purpous regs
	integer i;	
initial begin
	ReadData3 	<= 32'ha;
	// initialize locations
	for (i=0; i<1048576; i=i+1) begin
		GPREGS[i]	<= 8'h0;
	end
	GPREGS[0]	<= 8'ha;
end
always @ (MemRead)	begin
	//Reading from the memory

	ReadData3[7:0]		<=	GPREGS[ALUOut];
	ReadData3[15:8]		<=	GPREGS[ALUOut+1];
	ReadData3[23:16]	<=	GPREGS[ALUOut+2];
	ReadData3[31:24]	<=	GPREGS[ALUOut+3];

end

always	@ (posedge MemWrite)	begin
	//writing on the Memory
	GPREGS[ALUOut]		<=	ReadData2[7:0];
	GPREGS[ALUOut+1]	<=	ReadData2[15:8];
	GPREGS[ALUOut+2]	<=	ReadData2[23:16];
	GPREGS[ALUOut+3]	<=	ReadData2[31:24];
end 
endmodule
//test bench
module tb_MIPSALU2();
	reg	CLK	=	1;
	wire	[31:0]	PC_in,PC_out,INSTRUCTION,WriteData,A,B,ALUOut,ReadData2,ReadData3;
	reg		RESET;
	wire	[4:0]	ReadReg1,ReadReg2,WriteReg,WriteReg0;
	wire	[5:0]	FuncCode;
	wire	[1:0]	ALUOp;
	wire	[3:0]	ALUCtl;
	wire		Zero;

	wire	[5:0]	opcode;
	wire		RegWrite,RegDst,Branch,MemRead,MemtoReg,MemWrite,ALUSrc;
	wire	[15:0]	signExtIn;
	wire	[31:0]	signExtOut;

	PC			PC	(PC_in,PC_out,RESET,CLK);
	PC_ADDER		PA	(PC_in,PC_out);
	INSTRUCTION_MEMORY 	IM	(PC_out,INSTRUCTION);
	INSTRUCTION_REGISTER	IR	(INSTRUCTION,CLK,ReadReg1,ReadReg2,WriteReg0,FuncCode,opcode,signExtIn);
	REGISTERS		REGFILE	(ReadReg1,ReadReg2,WriteReg,WriteData,RegWrite,CLK,A,ReadData2);
	ALUControl		ALUCNTL	(ALUOp,FuncCode,ALUCtl);
	MIPSALU			ALU	(ALUCtl,A,B,ALUOut,Zero);
	CONTROL			CTRL	(opcode,RegDst,Branch,MemRead,MemtoReg,ALUOp,MemWrite,ALUSrc,RegWrite,Zero);
	SIGN_EXTENSION		SIGNEXT	(signExtIn,signExtOut);
	MUX0			MUX0	(ReadReg2,WriteReg0,RegDst,WriteReg);
	MUX1			MUX1	(ReadData2,signExtOut,ALUSrc,B);
	MUX2			MUX2	(ReadData3,ALUOut,WriteData,MemtoReg);
	DATAMEM			DATAMEM	(ALUOut,ReadData2,ReadData3,MemWrite,MemRead);
initial begin
	//PC_in = 32'h0;
	RESET	= 0;
	//RegWrite= 0;
	/*	
	#90	RESET	= 1;
	#5	RESET	= 0;
	*/
end

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
