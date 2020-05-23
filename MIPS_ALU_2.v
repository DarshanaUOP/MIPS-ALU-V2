module PC(PC_in,PC_out,RESET);
	input	[31:0]	PC_in;
	output	[31:0]	PC_out;
	input		RESET;

endmodule

module tb_MIPSALU2();
	reg	CLK	=	0;

always begin
        #5 	CLK 	= 	~CLK;		//CLOCK PULSE WITH EVERY 5 TIME UNITS
end
endmodule
