// Copyright (c) 2026 Jamie Blanks

// One 64-bit word of savestate scalar state.
//
// A module packs its live state into `din_i` and reads its restored state out
// of `dout_o`. On bus reset the word takes DEFAULT, the power-on value, so the
// same reset branch serves a cold start and a restore. `bus_dout_o` is zero
// unless this word is being read, so the parent can wire-OR every block.

module ss_reg #(
	// A word number, not a bit vector: blocks that place their words with a
	// generate loop pass a plain integer expression.
	parameter integer ADDR    = 0,
	parameter [63:0]  DEFAULT = 64'd0
) (
	input  wire        clk_i,
	input  wire [63:0] bus_din_i,
	input  wire [9:0]  bus_addr_i,
	input  wire        bus_wren_i,
	input  wire        bus_rst_i,
	output wire [63:0] bus_dout_o,

	input  wire [63:0] din_i,        // live value, the module's _BACK vector
	output reg  [63:0] dout_o        // restored value, the module's SS_ vector
);
	wire sel = (bus_addr_i == ADDR[9:0]);

	initial dout_o = DEFAULT;

	always @(posedge clk_i) begin
		if (bus_rst_i)             dout_o <= DEFAULT;
		else if (sel && bus_wren_i) dout_o <= bus_din_i;
	end

	assign bus_dout_o = sel ? din_i : 64'd0;
endmodule
