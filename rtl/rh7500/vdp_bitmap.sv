// The four bitmap layers.
//
// All four share the mode register, the one bitmap VRAM read port and the
// fill sequencer; only the per-layer registers, row buffers and colour latches
// are four of a kind. A bitmap layer responds to a register write one line
// later than a background layer because its row is prepared a line ahead:
//
//   1. fetch the row for the next scanline out of VRAM
//   2. walk it into a 256-pixel buffer, wrapping by SCROLLX, doing the 4bpp
//      subpalette lookup on the way
//   3. run the colour latch over buffer pixels 0 up to ENDX+1 or 255,
//      whichever is smaller
//   4. on the next scanline, output the buffer masked by STARTX and ENDX and
//      shifted by POSX
//
// Two colour-latch hardware bugs are reproduced: pixel 255 is never replaced
// by the latched colour, and the walk runs one pixel past ENDX, so a pixel
// outside the visible part of the layer can still update the latch.
//
// The row buffers are double banked, swapping at the start of every line.

module vdp_bitmap
(
	input  wire clk,
	input  wire reset,

	// Savestate scalar bus.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	input  wire        line_start,
	input  wire [8:0]  fill_y,     // VCOUNT of the line being prepared
	input  wire [8:0]  disp_y,     // VCOUNT of the line being shown
	input  wire [7:0]  disp_x,     // pixel being composed, presented at phase 0
	input  wire [2:0]  phase,

	// Registers.
	input  wire [2:0]  bm_mode,
	input  wire [15:0] bm_subpal,
	input  wire [35:0] scrollx,
	input  wire [35:0] scrolly,
	input  wire [35:0] posx,
	input  wire [35:0] posy,
	input  wire [31:0] startx,
	input  wire [31:0] endx,
	input  wire [31:0] endy,
	input  wire [3:0]  latch_en,
	input  wire [31:0] latch_thrs,

	// Bitmap VRAM render port.
	output wire [13:0] vram_addr,
	input  wire [63:0] vram_data,

	// One 8-bit palette index per layer, valid from phase 2.
	output wire [7:0]  pix0,
	output wire [7:0]  pix1,
	output wire [7:0]  pix2,
	output wire [7:0]  pix3
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_latch;
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- mode decode ---------------------------------------------------------
	// Mode 5 behaves as 0, modes 6 and 7 as 1.

	reg        is_8bpp;
	reg        split_x, split_y;
	reg [8:0]  width_mask, height_mask;
	always @* begin
		case (bm_mode)
		3'd0, 3'd5: begin is_8bpp = 1'b1; split_x = 1'b0; split_y = 1'b1;
		                  width_mask = 9'h0FF; height_mask = 9'h0FF; end
		3'd2:       begin is_8bpp = 1'b0; split_x = 1'b0; split_y = 1'b1;
		                  width_mask = 9'h1FF; height_mask = 9'h0FF; end
		3'd3:       begin is_8bpp = 1'b0; split_x = 1'b1; split_y = 1'b0;
		                  width_mask = 9'h0FF; height_mask = 9'h1FF; end
		3'd4:       begin is_8bpp = 1'b0; split_x = 1'b0; split_y = 1'b0;
		                  width_mask = 9'h1FF; height_mask = 9'h1FF; end
		default:    begin is_8bpp = 1'b1; split_x = 1'b0; split_y = 1'b0;
		                  width_mask = 9'h0FF; height_mask = 9'h1FF; end
		endcase
	end

	// ---- fill sequencer ------------------------------------------------------
	// 1024 pixel slots, 256 to each layer in turn, plus one to drain the read
	// pipeline. Bits 9 and 8 of the count are the layer.

	reg [10:0] fcnt;
	reg        bank;
	// The drain cycle must not do a pixel's work: the count has wrapped to
	// layer 0 pixel 0 and walking it again would corrupt layer 0's colour latch.
	wire       fill_slot = ~fcnt[10];
	wire       filling   = (fcnt != 11'd1025);
	wire [1:0] flayer    = fcnt[9:8];
	wire [7:0] fx        = fcnt[7:0];

	always @(posedge clk) begin
		if (reset) begin
			fcnt <= 11'd1025;
			bank <= 1'b0;
		end else if (line_start) begin
			fcnt <= 11'd0;
			bank <= ~bank;
		end else if (filling) begin
			fcnt <= fcnt + 11'd1;
		end
	end

	// ---- per-layer register slices for the layer being filled ----------------

	wire [8:0] f_scrollx = scrollx[flayer*9 +: 9];
	wire [8:0] f_scrolly = scrolly[flayer*9 +: 9];
	wire [8:0] f_posy    = posy[flayer*9 +: 9];
	wire [7:0] f_endx    = endx[flayer*8 +: 8];
	wire [7:0] f_endy    = endy[flayer*8 +: 8];
	wire [7:0] f_thrs    = latch_thrs[flayer*8 +: 8];
	wire       f_lat_en  = latch_en[flayer];
	// BM_SUBPAL holds BM0 in the top nibble, so the index runs backwards.
	wire [3:0] f_subpal  = bm_subpal[(3 - flayer)*4 +: 4];

	// A layer is skipped, colour latch included, on lines outside it.
	wire [8:0] f_rel_y   = fill_y - f_posy;
	wire       f_on_line = (f_rel_y <= {1'b0, f_endy});

	// The walk covers pixels 0 to ENDX+1, or to 255, whichever comes first.
	wire [8:0] f_last    = ({1'b0, f_endx} + 9'd1 > 9'd255) ? 9'd255
	                                                        : ({1'b0, f_endx} + 9'd1);
	wire       f_in_run  = ({1'b0, fx} <= f_last);

	// ---- address ------------------------------------------------------------

	wire [8:0] data_y_raw = fill_y + f_scrolly - f_posy;
	wire [8:0] data_y = (data_y_raw & height_mask)
	                  | (split_y ? {f_scrolly[8], 8'd0} : 9'd0);

	wire [8:0] data_x_raw = {1'b0, fx} + f_scrollx;
	wire [8:0] data_x = (data_x_raw & width_mask)
	                  | (split_x ? {f_scrollx[8], 8'd0} : 9'd0);

	wire [7:0] col_byte = is_8bpp ? data_x[7:0] : data_x[8:1];
	wire [16:0] byte_addr = {data_y, col_byte};

	assign vram_addr = byte_addr[16:3];

	// ---- fill pipeline -------------------------------------------------------
	// Stage A drives the address above; stage B, one cycle later, has the word.

	reg        b_valid;
	reg [1:0]  b_layer;
	reg [7:0]  b_x;
	reg [2:0]  b_byte;
	reg        b_nib_lo;
	reg        b_8bpp;
	reg [3:0]  b_subpal;
	reg        b_lat_en;
	reg [7:0]  b_thrs;
	reg        b_bank;

	always @(posedge clk) begin
		if (reset) begin
			b_valid <= 1'b0;
		end else begin
			b_valid  <= filling & fill_slot & f_in_run & f_on_line;
			b_layer  <= flayer;
			b_x      <= fx;
			b_byte   <= byte_addr[2:0];
			b_nib_lo <= data_x[0];
			b_8bpp   <= is_8bpp;
			b_subpal <= f_subpal;
			b_lat_en <= f_lat_en;
			b_thrs   <= f_thrs;
			b_bank   <= bank;
		end
	end

	// Big-endian packing: byte 0 of a word is in bits 63-56.
	wire [5:0] b_byte_r = 6'd7 - {3'd0, b_byte};
	wire [7:0] raw_byte = vram_data[b_byte_r * 6'd8 +: 8];
	wire [3:0] raw_nib  = b_nib_lo ? raw_byte[3:0] : raw_byte[7:4];

	// 4bpp: index 0 stays transparent, index 15 becomes the latch marker when
	// the latch is on, and anything else takes the layer's subpalette.
	wire [7:0] px_4bpp = (raw_nib == 4'd0)                   ? 8'd0
	                   : (raw_nib == 4'hF && b_lat_en)       ? 8'hFF
	                                                         : {b_subpal, raw_nib};
	wire [7:0] px_raw  = b_8bpp ? raw_byte : px_4bpp;

	// Colour latch. The threshold compares the whole byte in 8bpp and only the
	// low nibble in 4bpp. Pixel 255 is never replaced (hardware bug).
	reg  [7:0] latched [0:3];
	wire [7:0] thr_mask = b_8bpp ? 8'hFF : 8'h0F;
	wire       is_top   = (px_raw == 8'hFF);
	wire       lat_hit  = b_lat_en & is_top & (b_x != 8'hFF);
	wire       lat_upd  = b_lat_en & ~is_top
	                    & ((px_raw & thr_mask) < (b_thrs & thr_mask));
	wire [7:0] px_out   = lat_hit ? latched[b_layer] : px_raw;

	// ---- row buffers ---------------------------------------------------------

	wire [7:0] disp_idx [0:3];
	wire       disp_on  [0:3];
	wire [7:0] buf_q    [0:3];

	genvar L;
	generate
		for (L = 0; L < 4; L = L + 1) begin : g_layer
			cache_ram_dp #(
				.ADDR_WIDTH (9),
				.DATA_WIDTH (8)
			) u_buf (
				.clk_i     (clk),
				.addr_a_i  ({~bank, disp_idx[L]}),
				.wren_a_i  (1'b0),
				.wdata_a_i (8'd0),
				.q_a_o     (buf_q[L]),
				.addr_b_i  ({b_bank, b_x}),
				.wren_b_i  (b_valid & (b_layer == L[1:0])),
				.wdata_b_i (px_out),
				/* verilator lint_off PINCONNECTEMPTY */
				.q_b_o     ()
				/* verilator lint_on PINCONNECTEMPTY */
			);
		end
	endgenerate

	always @(posedge clk) begin
		if (reset) begin
			latched[0] <= ss_latch[7:0];
			latched[1] <= ss_latch[15:8];
			latched[2] <= ss_latch[23:16];
			latched[3] <= ss_latch[31:24];
		end else if (b_valid & lat_upd) begin
			latched[b_layer] <= px_raw;
		end
	end

	// ---- display -------------------------------------------------------------
	reg  [8:0] d_left  [0:3];
	reg  [8:0] d_right [0:3];
	reg  [7:0] d_shift [0:3];
	reg        d_show  [0:3];

	wire [8:0] vis_left  [0:3];
	wire [8:0] vis_right [0:3];
	wire       vis_ok    [0:3];
	wire       on_line   [0:3];

	generate
		for (L = 0; L < 4; L = L + 1) begin : g_span
			// POSX is a signed 9-bit screen coordinate, so the span is worked
			// out in 11 bits and then clamped to the 256-pixel picture.
			wire signed [10:0] px_s = {{2{posx[L*9+8]}}, posx[L*9 +: 9]};
			wire signed [10:0] l_raw = px_s + {3'd0, startx[L*8 +: 8]};
			wire signed [10:0] r_raw = px_s + {3'd0, endx[L*8 +: 8]};

			assign vis_left[L]  = (l_raw < 0) ? 9'd0 : l_raw[8:0];
			assign vis_right[L] = (r_raw > 11'sd255) ? 9'd255 : r_raw[8:0];
			assign vis_ok[L]    = (l_raw <= 11'sd255) & (r_raw >= 0)
			                    & (l_raw <= r_raw);

			wire [8:0] rel_y = disp_y - posy[L*9 +: 9];
			assign on_line[L] = (rel_y <= {1'b0, endy[L*8 +: 8]});

			assign disp_idx[L] = disp_x - d_shift[L];
			assign disp_on[L]  = d_show[L]
			                   & ({1'b0, disp_x} >= d_left[L])
			                   & ({1'b0, disp_x} <= d_right[L]);
		end
	endgenerate

	integer k;
	always @(posedge clk) begin
		if (reset) begin
			for (k = 0; k < 4; k = k + 1) begin
				d_left[k]  <= 9'd0;
				d_right[k] <= 9'd0;
				d_shift[k] <= 8'd0;
				d_show[k]  <= 1'b0;
			end
		end else if (line_start) begin
			for (k = 0; k < 4; k = k + 1) begin
				d_left[k]  <= vis_left[k];
				d_right[k] <= vis_right[k];
				d_shift[k] <= posx[k*9 +: 8];
				d_show[k]  <= vis_ok[k] & on_line[k];
			end
		end
	end



	// disp_on is delayed to match the buffer read it belongs to.
	reg disp_on_d [0:3];
	always @(posedge clk) begin
		for (k = 0; k < 4; k = k + 1) disp_on_d[k] <= disp_on[k];
	end

	// The buffers answer one cycle after the phase 0 address, so the pixel is
	// taken at phase 1 and held for the compositor.
	reg [7:0] p_hold [0:3];
	reg       p_on   [0:3];
	always @(posedge clk) begin
		if (reset) begin
			for (k = 0; k < 4; k = k + 1) begin
				p_hold[k] <= 8'd0;
				p_on[k]   <= 1'b0;
			end
		end else if (phase == 3'd1) begin
			for (k = 0; k < 4; k = k + 1) begin
				p_hold[k] <= buf_q[k];
				p_on[k]   <= disp_on_d[k];
			end
		end
	end

	assign pix0 = p_on[0] ? p_hold[0] : 8'd0;
	assign pix1 = p_on[1] ? p_hold[1] : 8'd0;
	assign pix2 = p_on[2] ? p_hold[2] : 8'd0;
	assign pix3 = p_on[3] ? p_hold[3] : 8'd0;

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"


	ss_reg #(
		.ADDR    (SSW_VDP_BITMAP),
		.DEFAULT (64'd0)
	) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({32'd0, latched[3], latched[2], latched[1], latched[0]}),
		.dout_o     (ss_latch)
	);

endmodule
