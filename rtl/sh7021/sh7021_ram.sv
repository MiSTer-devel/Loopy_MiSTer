// Copyright (c) 2026 Jamie Blanks
//
// SH7021 on-chip RAM, 1 KB on a 32-bit internal bus, one state per access.
// Contents survive sleep and standby, so nothing clears it.

module sh7021_ram (
	input  wire        clk_i,
	input  wire [7:0]  addr_i,        // longword address
	input  wire        we_i,
	input  wire [3:0]  be_i,
	input  wire [31:0] wdata_i,
	output wire [31:0] q_o,

	input  wire [7:0]  addr_b_i,
	input  wire        we_b_i,
	input  wire [3:0]  be_b_i,
	input  wire [31:0] wdata_b_i,
	output wire [31:0] q_b_o
);
	cache_ram_dp_be #(.ADDR_WIDTH (8), .DATA_WIDTH (32)) u_ram (
		.clk_i     (clk_i),
		.addr_a_i  (addr_i),
		.wren_a_i  (we_i),
		.be_a_i    (be_i),
		.wdata_a_i (wdata_i),
		.q_a_o     (q_o),
		.addr_b_i  (addr_b_i),
		.wren_b_i  (we_b_i),
		.be_b_i    (be_b_i),
		.wdata_b_i (wdata_b_i),
		.q_b_o     (q_b_o)
	);
endmodule
