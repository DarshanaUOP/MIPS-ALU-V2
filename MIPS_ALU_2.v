
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

//test bench
module tb_MIPSALU2();
	reg	CLK	=	0;
	wire	[31:0]	PC_in,PC_out;
	reg	RESET;
	
	PC		PC0	(PC_in,PC_out,RESET,CLK);
	PC_ADDER	PA	(PC_in,PC_out,CLK);
initial begin
	//PC_in = 32'h0;
	RESET	= 0;
end

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
