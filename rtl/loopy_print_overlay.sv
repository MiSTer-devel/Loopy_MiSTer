// Copyright (c) 2026 Jamie Blanks

// Draws the sticker the print routine is burning over the picture, while the
// print runs and for as long afterwards as the capture holds it.
//
// The capture is 128 x 112 with three bits per colour pass, two adjacent
// horizontal pixels per eighteen-bit cache word. The print is sideways and
// lays its columns down right edge first, so stored x is the printed row and
// is read back reversed. A palette approximates source RGB from the quantized
// yellow, magenta and cyan after BIOS correction.
//
// Full doubles it over the whole 256 x 224 render area; corner draws it 1:1 in
// the top right quarter.
//
// The address runs one pixel ahead of the picture so the block RAM's word for
// a pixel is out before video_mixer samples that pixel.

module loopy_print_overlay
(
	input  wire        clk,        // clk_video
	input  wire        reset,
	input  wire        ce_pix,

	input  wire        hvisible,
	input  wire        vvisible,
	input  wire        render,

	input  wire [1:0]  mode,       // 0 off, 1 corner, 2 full
	// The BIOS default colour converter gives YMC; other conversion callbacks
	// print in other orders.
	input  wire [1:0]  ink_order,
	input  wire        show,       // the capture has something to draw

	output reg  [12:0] rd_addr,
	input  wire [17:0] rd_data,
	input  wire        rd_valid,
	output wire [6:0]  display_row,
	output wire        display_active,
	output wire        frame_start,

	input  wire [14:0] color_i,
	output wire [14:0] color_o
);
	localparam [14:0] COL_EDGE = {5'd31, 5'd12, 5'd00};

	wire corner = (mode == 2'd1);
	wire on     = (mode != 2'd0) & show;

	wire [8:0] box_x0 = corner ? 9'd128 : 9'd0;
	wire [8:0] box_w  = corner ? 9'd128 : 9'd256;
	wire [8:0] box_h  = corner ? 9'd112 : 9'd224;

	// ---- pixel position ------------------------------------------------------

	reg [8:0] x;
	reg [8:0] y;
	reg       render_q;
	reg       vvisible_q;
	reg       line_ready;

	assign frame_start = vvisible_q & ~vvisible;
	always @(posedge clk) begin
		if (reset) vvisible_q <= 1'b0;
		else       vvisible_q <= vvisible;
	end

	wire [8:0] x_next   = render ? x + 9'd1 : 9'd0;
	wire       line_end = render_q & ~render;

	always @(posedge clk) begin
		if (reset) begin
			x        <= 9'd0;
			y        <= 9'd0;
			render_q <= 1'b0;
			line_ready <= 1'b0;
		end else if (ce_pix) begin
			render_q <= render;
			x        <= x_next;
			// A late fill stays paper for this entire scanline.
			if (render && x == 9'd0) line_ready <= rd_valid;

			if (!vvisible)     y <= 9'd0;
			else if (line_end) y <= y + 9'd1;
		end
	end

	// ---- which stored pixel this screen pixel shows --------------------------

	// Corner is one buffer pixel per screen pixel, full is one per two.
	/* verilator lint_off UNUSEDSIGNAL */
	function automatic [6:0] map(input [8:0] v, input [8:0] origin, input c);
		reg [8:0] d;   // the top bit only matters outside the box
		begin
			d   = v - origin;
			map = c ? d[6:0] : d[7:1];
		end
	endfunction

	// The look-ahead picks the word, the current x picks the pixel inside it.
	wire [6:0] bx_next = map(x_next, box_x0, corner);
	wire [6:0] by      = map(y,      9'd0,   corner);
	wire [6:0] bx_now  = map(x, box_x0, corner);   // kept for the box test
	/* verilator lint_on UNUSEDSIGNAL */

	// Stored x is the printed row, laid down right edge first, so the screen's
	// left is the last column printed. Adjacent stored x values share a word.
	wire [6:0] sx_next = 7'd127 - bx_next;
	wire [6:0] sx_now = 7'd127 - bx_now;
	assign display_row = by;
	assign display_active = on & render & hvisible & vvisible & (y < box_h);

	always @(posedge clk) begin
		if (reset)       rd_addr <= 13'd0;
		else if (ce_pix) rd_addr <= {by, sx_next[6:1]};
	end

	// ---- the pixel -----------------------------------------------------------

	wire [8:0] ink = !(line_ready && rd_valid) ? 9'd0
	               : sx_now[0] ? rd_data[17:9] : rd_data[8:0];

	// Three bits per pass, in the order they were printed.
	reg [2:0] y_ink, m_ink, c_ink;
	always @* begin
		case (ink_order)
		2'd0:    {y_ink, m_ink, c_ink} = {ink[2:0], ink[5:3], ink[8:6]};  // Y M C
		2'd1:    {y_ink, m_ink, c_ink} = {ink[8:6], ink[2:0], ink[5:3]};  // M C Y
		2'd2:    {y_ink, m_ink, c_ink} = {ink[5:3], ink[8:6], ink[2:0]};  // C Y M
		default: {y_ink, m_ink, c_ink} = {ink[8:6], ink[5:3], ink[2:0]};  // C M Y
		endcase
	end

	wire [14:0] paper;
	loopy_print_palette palette (.ink({c_ink, m_ink, y_ink}), .rgb(paper));

	wire in_x   = (x >= box_x0) & (x < box_x0 + box_w);
	wire in_y   = (y < box_h);
	wire in_box = on & hvisible & vvisible & render & in_x & in_y;

	wire on_edge = in_box & ((x == box_x0) | (x == box_x0 + box_w - 9'd1)
	                       | (y == 9'd0)   | (y == box_h - 9'd1));

	assign color_o = !in_box ? color_i
	               : on_edge ? COL_EDGE : paper;

endmodule
