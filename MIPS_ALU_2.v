
//Program Counter
module PC(PC_in,PC_out,RESET,CLK);
	input		[31:0]	PC_in;
	output	reg	[31:0]	PC_out;
	input		RESET,CLK;

	reg		[31:0]	PC_val;

always @ (posedge RESET)	begin
	PC_val	=	32'h0;
end
always @ (posedge CLK)	begin
	PC_val	<=	PC_in;
end
always @ (negedge CLK) begin
	PC_out	<= PC_val;
end
endmodule

//program counter adder
module PC_ADDER(PC_in,PC_out,CLK);
	input	[31:0]	PC_out;
	output 	reg [31:0]	PC_in;
	input	CLK;
always @ (negedge CLK)	begin
	PC_in	= PC_out + 4;	
end
endmodule

//test bench
module tb_MIPSALU2();
	reg	CLK	=	0;
	reg	[31:0]	PC_in,PC_out;
	wire	RESET;
	
	PC		PC0	(PC_in,PC_out,RESET,CLK);
	PC_ADDER	PA	(PC_in,PC_out,CLK);
initial begin
	PC_in = 32'h0;
end

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
