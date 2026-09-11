// The two background layers.
//
// Both share the tilemap layout, the character base and the tile VRAM slots;
// only tile size, scroll, format and subpalette differ. BG is fetched as the
// line is drawn, one pixel ahead of the compositor, so a scroll write during a
// line takes effect two pixels later on that same line.
//
// One pixel is eight clk_video cycles and the fetch fills four of them:
//
//   phase 0   BG0 tilemap address out       phase 1   entry back, latched
//   phase 1   BG1 tilemap address out       phase 2   entry back, latched
//   phase 4   BG0 character address out     phase 5   row back, latched
//   phase 5   BG1 character address out     phase 6   row back, latched
//   phase 6   BG0 pixel resolved            phase 7   BG1 pixel resolved
//
// A tile larger than one character walks an 8-wide character grid that wraps
// on each axis independently: a 16x16 tile starting at character 7 uses 7, 0,
// 15 and 8.

module vdp_bg
(
	input  wire clk,
	input  wire reset,

	input  wire [2:0] phase,
	input  wire [7:0] fetch_x,    // picture column this fetch is aimed at
	input  wire [8:0] fetch_y,    // picture line, VCOUNT

	// BG_CTRL.
	input  wire [1:0]  bg0_tsz,
	input  wire [1:0]  bg1_tsz,
	input  wire        bg0_8bpp,
	input  wire [1:0]  map_size,   // 0 64x64, 1 64x32, 2 32x64, 3 32x32
	input  wire        map_share,
	input  wire [47:0] scroll,     // BG0 X, BG0 Y, BG1 X, BG1 Y
	input  wire [31:0] subpal,     // BG0 then BG1, four nibbles each
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]  char_split,   // only seven bits can reach a 64 KB address
	/* verilator lint_on UNUSEDSIGNAL */

	// Tile VRAM render port.
	output wire [12:0] map0_addr,
	output wire [12:0] chr0_addr,
	output wire [12:0] map1_addr,
	output wire [12:0] chr1_addr,
	input  wire [63:0] rd_data,

	// One pixel per layer, held for the whole of the pixel it belongs to.
	output reg  [7:0] pix0,
	output reg        scrn0,       // 0 = screen A, 1 = screen B
	output reg  [7:0] pix1,
	output reg        scrn1,

	// Where character data starts, which the object engine needs too.
	output wire [15:0] char_base
);

	// ---- shared layout ------------------------------------------------------

	wire [2:0] map_w_log2 = (map_size[1] == 1'b0) ? 3'd6 : 3'd5;
	wire [2:0] map_h_log2 = (map_size[0] == 1'b0) ? 3'd6 : 3'd5;

	// Cells in the map, and from that where the second map and the character
	// data start. A shared layout has one map, a split layout two.
	wire [4:0] cells_log2 = {2'd0, map_w_log2} + {2'd0, map_h_log2};
	wire [15:0] map_bytes = 16'd1 << (cells_log2 + 5'd1);
	wire [15:0] map1_base = map_share ? 16'd0 : map_bytes;
	wire [15:0] data_base = map_share ? map_bytes : {map_bytes[14:0], 1'b0};
	assign char_base = data_base;

	// ---- per-layer fetch ----------------------------------------------------

	wire [15:0] map_addr_b [0:1];
	wire [15:0] chr_addr_b [0:1];
	wire [7:0]  layer_pix  [0:1];
	wire        layer_scrn [0:1];

	reg [15:0] desc [0:1];    // tilemap entry, latched from the map slot
	reg [7:0]  chr_byte [0:1];

	genvar L;
	generate
		for (L = 0; L < 2; L = L + 1) begin : g_bg
			wire [1:0] tsz = (L == 0) ? bg0_tsz : bg1_tsz;
			wire       is_8bpp = (L == 0) ? bg0_8bpp : 1'b0;   // BG1 is 4bpp only
			wire [11:0] sx = scroll[(L*2)*12 +: 12];
			wire [11:0] sy = scroll[(L*2+1)*12 +: 12];

			// Tile size is 8 to 64 pixels.
			wire [3:0] ts_log2 = {2'd0, tsz} + 4'd3;
			wire [5:0] tmask = (6'd1 << ts_log2) - 6'd1;

			// Layer coordinates wrap at the tilemap edge.
			wire [3:0]  wx_log2 = {1'b0, map_w_log2} + ts_log2;
			wire [3:0]  wy_log2 = {1'b0, map_h_log2} + ts_log2;
			// The widest map is 4096 pixels; the mask is built a bit wide and
			// the carry dropped.
			/* verilator lint_off UNUSEDSIGNAL */
			wire [12:0] wrap_x13 = (13'd1 << wx_log2) - 13'd1;
			wire [12:0] wrap_y13 = (13'd1 << wy_log2) - 13'd1;
			/* verilator lint_on UNUSEDSIGNAL */
			wire [11:0] lx = ({4'd0, fetch_x} + sx) & wrap_x13[11:0];
			wire [11:0] ly = ({3'd0, fetch_y} + sy) & wrap_y13[11:0];

			// Cell index.
			wire [11:0] col = lx >> ts_log2;
			wire [11:0] row = ly >> ts_log2;
			wire [11:0] map_cell = col | (row << map_w_log2);

			assign map_addr_b[L] = ((L == 0) ? 16'd0 : map1_base)
			                     + {3'd0, map_cell, 1'b0};

			// Entry fields.
			wire [10:0] d_chr    = desc[L][10:0];
			wire        d_scrn   = desc[L][11];
			wire [1:0]  d_subpal = desc[L][13:12];
			wire        d_flipx  = desc[L][14];
			wire        d_flipy  = desc[L][15];

			// Position inside the tile; a flip is an exclusive-or with the mask.
			wire [5:0] tile_x = (lx[5:0] & tmask) ^ (d_flipx ? tmask : 6'd0);
			wire [5:0] tile_y = (ly[5:0] & tmask) ^ (d_flipy ? tmask : 6'd0);

			// The 8-wide character grid, each axis wrapping on its own. In 4bpp
			// the split reserves rows of 8bpp characters, two grid rows each,
			// so it is added to the index and wraps with it.
			wire [7:0] split_rows = is_8bpp ? 8'd0 : {char_split[6:0], 1'b0};
			wire [2:0] chr_col = d_chr[2:0]  + tile_x[5:3];
			wire [7:0] chr_row = d_chr[10:3] + split_rows + {5'd0, tile_y[5:3]};
			wire [10:0] chr_id = {chr_row, chr_col};

			wire [16:0] chr_off = {chr_id, tile_y[2:0], tile_x[2:0]};
			assign chr_addr_b[L] = is_8bpp ? (data_base + chr_off[15:0])
			                              : (data_base + chr_off[16:1]);

			// The row byte carries one pixel in 8bpp and two in 4bpp, the left
			// one in the upper nibble.
			wire [3:0] nib = tile_x[0] ? chr_byte[L][3:0] : chr_byte[L][7:4];
			wire [3:0] sp  = subpal[L*16 + {2'd0, d_subpal} * 4 +: 4];

			assign layer_pix[L]  = is_8bpp ? chr_byte[L]
			                     : (nib == 4'd0) ? 8'd0 : {sp, nib};
			assign layer_scrn[L] = d_scrn;
		end
	endgenerate

	assign map0_addr = map_addr_b[0][15:3];
	assign map1_addr = map_addr_b[1][15:3];

	// Character address, registered one phase after its tilemap entry arrives
	// and held until the fetch slot.
	reg [15:0] chr_addr_r [0:1];
	assign chr0_addr = chr_addr_r[0][15:3];
	assign chr1_addr = chr_addr_r[1][15:3];

	// ---- slot pipeline ------------------------------------------------------
	// Big-endian words: byte 0 is bits 63-56, tilemap entry 0 is bits 63-48.

	reg [2:0] map_sel [0:1];
	reg [2:0] chr_sel [0:1];

	wire [1:0] map_word0 = map_addr_b[0][2:1];
	wire [1:0] map_word1 = map_addr_b[1][2:1];
	wire [2:0] chr_word0 = chr_addr_b[0][2:0];
	wire [2:0] chr_word1 = chr_addr_b[1][2:0];

	wire [5:0] sel_a = 6'd3 - {4'd0, map_sel[0][1:0]};
	wire [5:0] sel_b = 6'd3 - {4'd0, map_sel[1][1:0]};
	wire [5:0] sel_c = 6'd7 - {3'd0, chr_sel[0]};
	wire [5:0] sel_d = 6'd7 - {3'd0, chr_sel[1]};

	always @(posedge clk) begin
		if (reset) begin
			desc[0] <= 16'd0; desc[1] <= 16'd0;
			chr_addr_r[0] <= 16'd0; chr_addr_r[1] <= 16'd0;
			chr_byte[0] <= 8'd0; chr_byte[1] <= 8'd0;
			map_sel[0] <= 3'd0; map_sel[1] <= 3'd0;
			chr_sel[0] <= 3'd0; chr_sel[1] <= 3'd0;
			pix0 <= 8'd0; scrn0 <= 1'b0;
			pix1 <= 8'd0; scrn1 <= 1'b0;
		end else begin
			// Which slice of the word each address asked for.
			if (phase == 3'd0) map_sel[0] <= {1'b0, map_word0};
			if (phase == 3'd1) map_sel[1] <= {1'b0, map_word1};
			if (phase == 3'd4) chr_sel[0] <= chr_word0;
			if (phase == 3'd5) chr_sel[1] <= chr_word1;

			if (phase == 3'd1) desc[0]     <= rd_data[sel_a * 6'd16 +: 16];
			if (phase == 3'd2) desc[1]     <= rd_data[sel_b * 6'd16 +: 16];

			if (phase == 3'd2) chr_addr_r[0] <= chr_addr_b[0];
			if (phase == 3'd3) chr_addr_r[1] <= chr_addr_b[1];
			if (phase == 3'd5) chr_byte[0] <= rd_data[sel_c * 6'd8 +: 8];
			if (phase == 3'd6) chr_byte[1] <= rd_data[sel_d * 6'd8 +: 8];

			if (phase == 3'd6) begin pix0 <= layer_pix[0]; scrn0 <= layer_scrn[0]; end
			if (phase == 3'd7) begin pix1 <= layer_pix[1]; scrn1 <= layer_scrn[1]; end
		end
	end

endmodule
