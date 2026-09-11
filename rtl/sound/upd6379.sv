// Copyright (c) 2026 Jamie Blanks

// IC251, the NEC uPD6379: 16-bit stereo DAC fed by the CDT109. The serial
// link ends at an analogue pin, so only the frame latch is modelled.
//
// The sound banks were written for keyboards using an LC7881C, whose LRCK
// polarity is opposite to the uPD6379's, so they play with channels swapped
// relative to the bank layout. A board fitted with the uPD6379A (keyboard
// polarity) would need SWAP set.

module upd6379 #(
	parameter bit SWAP = 1'b0
) (
	input  wire        clk_i,
	input  wire        rst_i,

	input  wire        lrck_i,            // one pulse per stereo frame
	input  wire signed [15:0] l_i,
	input  wire signed [15:0] r_i,

	output reg signed [15:0] aout_l_o,
	output reg signed [15:0] aout_r_o
);
	always @(posedge clk_i) begin
		if (rst_i) begin
			aout_l_o <= 16'sd0;
			aout_r_o <= 16'sd0;
		end else if (lrck_i) begin
			aout_l_o <= SWAP ? r_i : l_i;
			aout_r_o <= SWAP ? l_i : r_i;
		end
	end
endmodule
