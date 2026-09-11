// Copyright (c) 2026 Jamie Blanks

// The board's analogue stage after the DAC: a resonant low pass, the mixer's
// gain, and DC blocking.
//
//   in -> [ low pass 8247 Hz, Q 1.67 ] -> x 4.216 -> [ high pass 20 Hz ] -> clip
//
// The corner and Q come from the board's network (three 10k, two 330p, one 5n6
// around a unity buffer). The gain is the mixer's 68k/10k times the 0.62 of
// its output divider into 10k, folded into the low pass numerator.
//
// One shared multiplier does all twenty products, seven cycles per filter per
// channel. Coefficients are Q3.26, state Q8.32: the DC block's running sum
// passes forty before its last term cancels, and its poles at radius 0.9989
// integrate rounding error for about nine hundred samples.

module loopy_audio_filter (
	input  wire        clk_i,
	input  wire        rst_i,

	input  wire        ce_sample_i,
	input  wire signed [15:0] in_l_i,
	input  wire signed [15:0] in_r_i,

	output reg signed [15:0] out_l_o,
	output reg signed [15:0] out_r_o,

	// Savestate: the four delay registers of each filter instance and the
	// output register. Everything else is rebuilt within one sample period.
	input  wire [63:0] ss_din_i,
	input  wire [9:0]  ss_addr_i,
	input  wire        ss_wren_i,
	input  wire        ss_rst_i,
	output wire [63:0] ss_dout_o
);
	`include "ss_map.svh"
	localparam int CQ = 26;    // coefficient fraction bits; state carries 32

	// Low pass, with the 6.8 x 0.62 makeup gain folded into the numerator.
	localparam signed [28:0] TONE_B0 = 29'sd21816617;   // +0.325092926
	localparam signed [28:0] TONE_B1 = 29'sd43633234;   // +0.650185852
	localparam signed [28:0] TONE_B2 = 29'sd21816617;   // +0.325092926
	localparam signed [28:0] TONE_A1 = -29'sd93854318;  // -1.398538326
	localparam signed [28:0] TONE_A2 = 29'sd47444332;   // +0.706975637

	// DC block.
	localparam signed [28:0] DC_B0 = 29'sd67037922;     // +0.998942880
	localparam signed [28:0] DC_B1 = -29'sd134075844;   // -1.997885761
	localparam signed [28:0] DC_B2 = 29'sd67037922;     // +0.998942880
	localparam signed [28:0] DC_A1 = -29'sd134075770;   // -1.997884666
	localparam signed [28:0] DC_A2 = 29'sd66967053;     // +0.997886856

	// Four filter instances, indexed {filter, channel}: 0,1 are the low pass
	// on left and right, 2,3 the DC block on the same two.
	reg signed [39:0] x1 [4], x2 [4], y1 [4], y2 [4];
	reg signed [39:0] tone_out [2];

	reg signed [28:0] coeff;
	reg signed [39:0] operand;
	reg signed [68:0] prod;
	reg signed [39:0] acc;
	reg signed [39:0] x0;

	reg  [2:0] k;
	reg  [1:0] slot;
	reg        busy;

	wire flt = slot[1];

	// A Q3.26 coefficient times a Q8.32 state is Q11.58. Shifting off the
	// coefficient's 26 fraction bits puts it back in Q8.32, rounded so the
	// filter has no systematic bias towards zero.
	wire signed [39:0] term = $signed(prod[CQ+39 : CQ]) + {39'd0, prod[CQ-1]};

	// Stage input: the sample for the low pass, the low pass output for the DC
	// block. Full scale 32768 maps to 2^32 in Q8.32, a shift of seventeen.
	wire signed [39:0] in_scaled = slot[0] ? {{7{in_r_i[15]}}, in_r_i, 17'd0}
	                                       : {{7{in_l_i[15]}}, in_l_i, 17'd0};
	wire signed [39:0] stage_in  = flt ? tone_out[slot[0]] : in_scaled;

	wire signed [28:0] c_b0 = flt ? DC_B0 : TONE_B0;
	wire signed [28:0] c_b1 = flt ? DC_B1 : TONE_B1;
	wire signed [28:0] c_b2 = flt ? DC_B2 : TONE_B2;
	wire signed [28:0] c_a1 = flt ? DC_A1 : TONE_A1;
	wire signed [28:0] c_a2 = flt ? DC_A2 : TONE_A2;

	// Q8.32 to a full-scale 16-bit sample.
	function automatic signed [15:0] clip16(input signed [39:0] v);
		if      (v >=  40'sd4294967296) clip16 =  16'sd32767;
		else if (v <= -40'sd4294967296) clip16 = -16'sd32767;
		else                            clip16 = $signed(v[32:17]);
	endfunction

	integer i;

	always @(posedge clk_i) begin
		if (rst_i) begin
			for (i = 0; i < 4; i = i + 1) begin
				x1[i] <= $signed(ss_state[i * 160 +   0 +: 40]);
				x2[i] <= $signed(ss_state[i * 160 +  40 +: 40]);
				y1[i] <= $signed(ss_state[i * 160 +  80 +: 40]);
				y2[i] <= $signed(ss_state[i * 160 + 120 +: 40]);
			end
			tone_out[0] <= 40'sd0; tone_out[1] <= 40'sd0;
			coeff <= 29'sd0; operand <= 40'sd0; prod <= 69'sd0;
			acc <= 40'sd0; x0 <= 40'sd0;
			k <= 3'd0; slot <= 2'd0; busy <= 1'b0;
			out_l_o <= $signed(ss_state[640 +: 16]);
			out_r_o <= $signed(ss_state[656 +: 16]);
		end else begin
			prod <= coeff * operand;

			if (ce_sample_i) begin
				busy <= 1'b1;
				slot <= 2'd0;
				k    <= 3'd0;
			end else if (busy) begin
				// b0*x0, b1*x1, b2*x2, a1*y1, a2*y2, then the answer. The
				// coefficient and operand registers take a cycle and the
				// product another, so a term is on `term` two steps after it
				// was asked for.
				case (k)
				3'd0: begin
					x0      <= stage_in;
					coeff   <= c_b0;
					operand <= stage_in;
					acc     <= 40'sd0;
				end
				3'd1: begin coeff <= c_b1; operand <= x1[slot]; end
				3'd2: begin coeff <= c_b2; operand <= x2[slot]; acc <= term; end
				3'd3: begin coeff <= c_a1; operand <= y1[slot]; acc <= acc + term; end
				3'd4: begin coeff <= c_a2; operand <= y2[slot]; acc <= acc + term; end
				3'd5: begin acc <= acc - term; end
				3'd6: begin
					x2[slot] <= x1[slot]; x1[slot] <= x0;
					y2[slot] <= y1[slot]; y1[slot] <= acc - term;
					if (!flt)         tone_out[slot[0]] <= acc - term;
					else if (slot[0]) out_r_o <= clip16(acc - term);
					else              out_l_o <= clip16(acc - term);
					if (slot == 2'd3) busy <= 1'b0;
					slot <= slot + 2'd1;
				end
				default: ;
				endcase
				k <= (k == 3'd6) ? 3'd0 : k + 3'd1;
			end
		end
	end

	// ---- savestate ---------------------------------------------------------

	wire [703:0] state_back = {32'd0, out_r_o, out_l_o,
	                           y2[3], y1[3], x2[3], x1[3],
	                           y2[2], y1[2], x2[2], x1[2],
	                           y2[1], y1[1], x2[1], x1[1],
	                           y2[0], y1[0], x2[0], x1[0]};
	wire [703:0] ss_state;
	wire [63:0]  ss_d [11];

	genvar gw;
	generate
		for (gw = 0; gw < 11; gw = gw + 1) begin : g_ss
			ss_reg #(.ADDR (SSW_FILTER_BASE + gw)) u_ss (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din_i),
				.bus_addr_i (ss_addr_i),
				.bus_wren_i (ss_wren_i),
				.bus_rst_i  (ss_rst_i),
				.bus_dout_o (ss_d[gw]),
				.din_i      (state_back[gw * 64 +: 64]),
				.dout_o     (ss_state[gw * 64 +: 64])
			);
		end
	endgenerate

	assign ss_dout_o = ss_d[0] | ss_d[1] | ss_d[2] | ss_d[3] | ss_d[4] | ss_d[5]
	                 | ss_d[6] | ss_d[7] | ss_d[8] | ss_d[9] | ss_d[10];

endmodule
