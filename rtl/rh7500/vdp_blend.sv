// Screen combination: mixes the two 15-bit screen colours into the one that
// leaves the chip.
//
// Each screen arrives as a palette index and its colour. Index zero shows the
// backdrop; SBCOL makes screen B show its backdrop everywhere. A screen
// switched off at SCREEN_CTRL contributes black in modes 0-3 and is
// transparent in the over modes.
//
//   0  A plus or minus B (CSUB), per channel, clamped to 0-31
//   1  the same, halved before clamping
//   2  screen A alone
//   3  hi-res: A for the first half of each pixel, B for the second
//   4  B over A: B wins where it drew a layer; its backdrop is transparent
//   5  A over B, the same with the screens swapped
//   6,7 undefined, black
//
// The border is screen A's backdrop when screen A is on, else black.

module vdp_blend
(
	input  wire clk,
	input  wire reset,
	input  wire [2:0] phase,
	input  wire       half,        // second half of the pixel, for hi-res

	input  wire [7:0]  idx_a,
	input  wire [7:0]  idx_b,
	input  wire [14:0] col_a,      // palette lookup of idx_a
	input  wire [14:0] col_b,
	input  wire [14:0] backdrop_a,
	input  wire [14:0] backdrop_b,

	input  wire [2:0] blend_mode,
	input  wire       blend_sub,   // CSUB
	input  wire       screen_a_en,
	input  wire       screen_b_en,
	input  wire       screen_b_col, // SBCOL

	input  wire       in_render,   // inside the picture
	input  wire       render_dis,  // VIDEO_DEBUG RD
	input  wire       snow_hit,    // a CPU palette read took this pixel's lookup
	input  wire [14:0] snow_col,   // and the colour it read

	output reg  [14:0] color,      // to the video output, settled at phase 7
	output wire [14:0] screen_a,   // the two screens as they stand, for capture
	output wire [14:0] blended     // the mixed result, for capture
);

	// The palette answers in phase 5 and the result is wanted in phase 7. The
	// colours are registered in between so the colour maths, the longest run
	// in the render path, starts from a register rather than the RAM output.
	reg [14:0] q_a, q_b;
	reg [7:0]  q_ia, q_ib;
	always @(posedge clk) begin
		if (reset) begin
			q_a <= 15'd0; q_b <= 15'd0; q_ia <= 8'd0; q_ib <= 8'd0;
		end else if (phase == 3'd5) begin
			q_a  <= col_a;  q_b  <= col_b;
			q_ia <= idx_a;  q_ib <= idx_b;
		end
	end

	// What each screen shows before any mixing.
	wire [14:0] show_a = (q_ia == 8'd0) ? backdrop_a : q_a;
	wire [14:0] show_b = (q_ib == 8'd0 || screen_b_col) ? backdrop_b : q_b;

	wire [14:0] in_a = screen_a_en ? show_a : 15'd0;
	wire [14:0] in_b = screen_b_en ? show_b : 15'd0;

	assign screen_a = snow_hit ? snow_col : in_a;

	// Per-channel add or subtract with sign, optional halve, then clamp.
	function automatic [4:0] mix (input [4:0] x, input [4:0] y,
	                              input sub, input halve);
		reg signed [6:0] sum;
		begin
			sum = sub ? ($signed({2'b00, x}) - $signed({2'b00, y}))
			          : ($signed({2'b00, x}) + $signed({2'b00, y}));
			// Arithmetic shift so a negative difference still clamps to zero.
			if (halve) sum = sum >>> 1;
			if (sum < 7'sd0)       mix = 5'd0;
			else if (sum > 7'sd31) mix = 5'd31;
			else                   mix = sum[4:0];
		end
	endfunction

	wire halve = blend_mode[0];
	wire [14:0] math = {
		mix(in_a[14:10], in_b[14:10], blend_sub, halve),
		mix(in_a[9:5],   in_b[9:5],   blend_sub, halve),
		mix(in_a[4:0],   in_b[4:0],   blend_sub, halve)
	};

	// The over modes test the raw index, so a screen showing only its backdrop
	// is transparent, as is a screen that is switched off. SBCOL turns all of
	// screen B into its backdrop colour, which then counts as drawn.
	wire drew_a = screen_a_en & (q_ia != 8'd0);
	wire drew_b = screen_b_en & (screen_b_col | (q_ib != 8'd0));

	reg [14:0] mixed;
	always @* begin
		case (blend_mode)
		3'd0, 3'd1: mixed = math;
		3'd2:       mixed = in_a;
		3'd3:       mixed = in_a;      // the halves are kept apart below
		3'd4:       mixed = drew_b ? in_b : in_a;
		3'd5:       mixed = drew_a ? in_a : in_b;
		default:    mixed = 15'd0;
		endcase
	end

	wire [14:0] out_mix = snow_hit ? snow_col : mixed;

	// Capture mode 0 takes the blended output; in hi-res that is screen B alone.
	wire hires = (blend_mode == 3'd3);
	assign blended = hires ? in_b : out_mix;

	// The border takes screen A's backdrop, and render disable turns the whole
	// picture into the same thing.
	wire [14:0] border_col = screen_a_en ? backdrop_a : 15'd0;
	wire        show_pic   = in_render & ~render_dis;

	// Output settles at phase 7 and holds for the whole next pixel. Hi-res
	// keeps both screens and the half-pixel select picks between them, so both
	// halves come from the same pixel.
	reg [14:0] c_norm, c_half_a, c_half_b;
	reg        c_hires;

	always @(posedge clk) begin
		if (reset) begin
			c_norm   <= 15'd0;
			c_half_a <= 15'd0;
			c_half_b <= 15'd0;
			c_hires  <= 1'b0;
		end else if (phase == 3'd7) begin
			c_norm   <= show_pic ? out_mix : border_col;
			c_half_a <= show_pic ? in_a  : border_col;
			c_half_b <= show_pic ? in_b  : border_col;
			c_hires  <= show_pic & hires;
		end
	end

	always @* begin
		if (c_hires) color = half ? c_half_b : c_half_a;
		else         color = c_norm;
	end

endmodule
