
//Program Counter
module PC(PC_in,PC_out,RESET,CLK);
	input		[31:0]	PC_in;
	output	reg	[31:0]	PC_out;
	input		RESET,CLK;

	//reg		[31:0]	PC_val;
	//reg		[31:0]	PC_out;
initial begin
	PC_out 	= 32'h0;
end

always @ (posedge RESET)	begin
	PC_out	=	32'h0;
end

always @ (negedge CLK) begin
	//PC_val	<=	PC_in;
	PC_out	= PC_in;
end
endmodule

//program counter adder
module PC_ADDER(PC_in,PC_out,CLK);
	input		[31:0]	PC_out;
	output 	reg 	[31:0]	PC_in;
	input	CLK;
	reg	[31:0]	PC_next;
/*
always @ (posedge CLK) begin
	PC_next = PC_out + 4;
end
*/
initial begin
	PC_in = 32'h0;
end
always @ (posedge CLK)	begin
	PC_in	= PC_out + 4;	
end
endmodule

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
	funct	:		-	000000
	
	therefore::::
	INSTRUCTION		-	|000010 00|000 00001 |00010 000|00 000000
*/
	//ADD	
	memReg[0] = 8'b00001000; //8'h8; 
	memReg[1] = 8'b00000001;
	memReg[2] = 8'b00010000;
	memReg[3] = 8'b00000000;
	
	//AND	{register3 AND register4 => register5 ;}
	memReg[4] = 8'b00000000;
	memReg[5] = 8'b01100100;
	memReg[6] = 8'b00101000;
	memReg[7] = 8'b00000000;
	
	//OR	{register6 OR register7 => register8 ;}
	memReg[8] = 8'b00000100;
	memReg[9] = 8'b11000111;
	memReg[10] = 8'b01000000;
	memReg[11] = 8'b00000000;
	
	//SUB	{register9 - register10 => register11} ;
	memReg[12] = 8'b00011001;
	memReg[13] = 8'b10001001;
	memReg[14] = 8'b01011000;
	memReg[15] = 8'b00000000;
	
	//SLT	register12 < register13 ?=> register14 ;
	memReg[16] = 8'b00011101;
	memReg[17] = 8'b10001101;
	memReg[18] = 8'b01110000;
	memReg[19] = 8'b00000000;
	
	//NOR	register15 + register16 => register17 ;
	memReg[20] = 8'b00110001;
	memReg[21] = 8'b11110000;
	memReg[22] = 8'b10001000;
	memReg[23] = 8'b00000000;
	
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
module INSTRUCTION_REGISTER(INSTRUCTION,CLK,ReadReg1,ReadReg2,WriteReg,FuncCode);
	input	[31:0]	INSTRUCTION;
	input		CLK;
	output	reg	[4:0]	ReadReg1,ReadReg2,WriteReg;
	output	reg	[5:0]	FuncCode;
	reg	[31:0]	CurrentINS;

always @ (posedge	CLK) begin
	CurrentINS	<=	INSTRUCTION;
	ReadReg1	<=	INSTRUCTION[25:21];
	ReadReg2	<=	INSTRUCTION[20:16];
	WriteReg	<=	INSTRUCTION[15:11];
	FuncCode	<=	INSTRUCTION[5:0];
end
endmodule

module REGISTERS(ReadReg1,ReadReg2,WriteReg,WriteData,RegWrite,CLK,A,B);
	input		[4:0]	ReadReg1,ReadReg2,WriteReg;
	input			CLK,RegWrite;
	input		[31:0]	WriteData;
	output	reg	[31:0]	A,B;
	
	//DEFINING REGISTERS
	reg		[31:0]	REGS[31:0];
initial begin
	A		=	32'h0;
	B		=	32'h0;

	REGS[0]		=	32'h0;
	REGS[1]		=	32'h0;
	REGS[2]		=	32'h0;
	REGS[3]		=	32'h0;
	REGS[4]		=	32'h0;
	REGS[5]		=	32'h0;
	REGS[6]		=	32'h0;
	REGS[7]		=	32'h0;
	REGS[8]		=	32'h0;
	REGS[9]		=	32'h0;
	REGS[10]	=	32'h0;
	REGS[11]	=	32'h0;
	REGS[12]	=	32'h0;
	REGS[13]	=	32'h0;
	REGS[14]	=	32'h0;
	REGS[15]	=	32'h0;
	REGS[16]	=	32'h0;
	REGS[17]	=	32'h0;
	REGS[18]	=	32'h0;
	REGS[19]	=	32'h0;
	REGS[20]	=	32'h0;
	REGS[21]	=	32'h0;
	REGS[22]	=	32'h0;
	REGS[23]	=	32'h0;
	REGS[24]	=	32'h0;
	REGS[25]	=	32'h0;
	REGS[26]	=	32'h0;
	REGS[27]	=	32'h0;
	REGS[28]	=	32'h0;
	REGS[29]	=	32'h0;
	REGS[30]	=	32'h0;
	REGS[31]	=	32'h0;
end
always @ (posedge CLK)	begin
	A	<=	REGS[ReadReg1];
	B	<=	REGS[ReadReg2];

end

endmodule

//test bench
module tb_MIPSALU2();
	reg	CLK	=	0;
	wire	[31:0]	PC_in,PC_out,INSTRUCTION,WriteData,A,B;
	reg	RESET,RegWrite;
	wire	[4:0]	ReadReg1,ReadReg2,WriteReg;
	wire	[5:0]	FuncCode;

	PC			PC	(PC_in,PC_out,RESET,CLK);
	PC_ADDER		PA	(PC_in,PC_out,CLK);
	INSTRUCTION_MEMORY 	IM	(PC_out,INSTRUCTION);
	INSTRUCTION_REGISTER	IR	(INSTRUCTION,CLK,ReadReg1,ReadReg2,WriteReg,FuncCode);
	REGISTERS		REGFILE	(ReadReg1,ReadReg2,WriteReg,WriteData,RegWrite,CLK,A,B);
initial begin
	//PC_in = 32'h0;
	RESET	= 0;
	RegWrite= 0;
	#30	RESET	= 1;
	#5	RESET	= 0;
end

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
