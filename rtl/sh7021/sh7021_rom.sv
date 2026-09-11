// Copyright (c) 2026 Jamie Blanks
//
// SH7021 on-chip mask ROM, 32 KB on a 32-bit internal bus, one state per
// access. Big-endian: the byte at H'0000000 is the most significant of the
// first longword. The second port loads the BIOS image before reset releases.

module sh7021_rom (
	input  wire        clk_i,
	input  wire [12:0] addr_i,       // longword address
	output wire [31:0] q_o,

	input  wire [12:0] wr_addr_i,
	input  wire        wr_en_i,
	input  wire [31:0] wr_data_i
);
	/* verilator lint_off UNUSEDSIGNAL */
	wire [31:0] unused_q;
	/* verilator lint_on UNUSEDSIGNAL */

	cache_ram_dp_be #(.ADDR_WIDTH (13), .DATA_WIDTH (32)) u_rom (
		.clk_i     (clk_i),
		.addr_a_i  (addr_i),
		.wren_a_i  (1'b0),
		.be_a_i    (4'b0000),
		.wdata_a_i (32'd0),
		.q_a_o     (q_o),
		.addr_b_i  (wr_addr_i),
		.wren_b_i  (wr_en_i),
		.be_b_i    (4'b1111),
		.wdata_b_i (wr_data_i),
		.q_b_o     (unused_q)
	);
endmodule
