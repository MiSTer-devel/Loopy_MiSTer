// Copyright (c) 2026 Jamie Blanks
//
// SH-1 general register file, R0-R15. Three read ports because the indexed
// addressing modes need Rn, Rm and R0 in the same slot, and two write ports
// because an EX result and a load result can retire in the same slot.
// Port A is the EX write, port B the MA/WB write; port A wins a collision.

module sh1_regfile (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire [3:0]  rn_i,
	input  wire [3:0]  rm_i,
	output wire [31:0] rn_o,
	output wire [31:0] rm_o,
	output wire [31:0] r0_o,

	input  wire        we_a_i,
	input  wire [3:0]  wa_a_i,
	input  wire [31:0] wd_a_i,

	input  wire        we_b_i,
	input  wire [3:0]  wa_b_i,
	input  wire [31:0] wd_b_i,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [31:0] r [0:15];

	assign rn_o = r[rn_i];
	assign rm_o = r[rm_i];
	assign r0_o = r[0];

	// --------------------------------------------------------- savestate
	// Two registers to a word, the even-numbered one in the low half.
	wire [63:0] SS_R [0:7];
	wire [63:0] ss_q [0:7];

	genvar g;
	generate
		for (g = 0; g < 8; g = g + 1) begin : g_ss
			ss_reg #(.ADDR (SSW_SH1_REGS + g), .DEFAULT (64'd0)) u_ss (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				.bus_rst_i  (ss_rst),
				.bus_dout_o (ss_q[g]),
				.din_i      ({r[2*g + 1], r[2*g]}),
				.dout_o     (SS_R[g])
			);
		end
	endgenerate

	integer q;
	reg [63:0] ss_or;
	always @* begin
		ss_or = 64'd0;
		for (q = 0; q < 8; q = q + 1) ss_or = ss_or | ss_q[q];
	end
	assign ss_dout = ss_or;

	// R0-R15 are undefined after reset on the real part; the restore value
	// (zero by default) serves as the reset value.
	integer i;
	always @(posedge clk_i) begin
		if (rst_i) begin
			for (i = 0; i < 8; i = i + 1) begin
				r[2*i]     <= SS_R[i][31:0];
				r[2*i + 1] <= SS_R[i][63:32];
			end
		end else if (ce_i) begin
			if (we_b_i) r[wa_b_i] <= wd_b_i;
			if (we_a_i) r[wa_a_i] <= wd_a_i;
		end
	end
endmodule
