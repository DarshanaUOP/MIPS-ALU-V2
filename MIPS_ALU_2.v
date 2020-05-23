
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
	reg	[7:0]	memReg[15:0];

/*
	register0 + register1 => register0 ;
	op	:ADD 		-	000010
	rs	:register0	-	00000  
	rt	:register1 	-	00001
	rd	:register0 	-	00000
	shmt	:		-	00000
	funct	:		-	000000
	
	therefore::::
	the INSTRUCTION		-	|000010 00|000 00001 |00000 000|00 000000
*/

initial begin
	memReg[0] = 8'h8;// 8'b00001000;
	memReg[1] = 8'b00000001;
	memReg[2] = 8'b00000000;
	memReg[3] = 8'b00000000;
end
	always @ (PC_out) begin
		//memReg[PC_out] = memReg[PC_out] + 8'h1;
		INSTRUCTION[7:0] <= memReg[PC_out];
		INSTRUCTION[15:8] <= memReg[PC_out + 1];
		INSTRUCTION[23:16] <= memReg[PC_out + 2];
		INSTRUCTION[31:24] <= memReg[PC_out + 3];

	end
endmodule

//test bench
module tb_MIPSALU2();
	reg	CLK	=	0;
	wire	[31:0]	PC_in,PC_out,INSTRUCTION;
	reg	RESET;
	
	PC			PC0	(PC_in,PC_out,RESET,CLK);
	PC_ADDER		PA	(PC_in,PC_out,CLK);
	INSTRUCTION_MEMORY 	IM	(PC_out,INSTRUCTION);
initial begin
	//PC_in = 32'h0;
	RESET	= 0;
	#100	RESET	= 1;
end

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
