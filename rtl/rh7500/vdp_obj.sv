// The object engine: both OBJ layers, 128 entries, two line buffers.
//
// The engine fills a line buffer during the line before it is shown. OAM
// entry 0 is the front-most object and entry 127 the back-most, whichever
// layer SPLIT puts them on; the walk runs 0 up to 127 and writes only where
// the buffer is still empty.
//
// The per-line budget is 32 character columns across both layers, one
// picture width. A 32x32 object costs four columns, an 8x8 one. When the
// budget runs out the rest of the line's objects are dropped, so the back-most
// objects are the ones lost.
//
// Tile VRAM slots are phases 2, 3, 6 and 7 inside the render window and every
// phase outside it, so the fetch waits for a slot and the emit does not.
//
// A multi-character object walks the 8-wide character grid with each axis
// wrapping on its own, as in vdp_bg. The line buffers are rebuilt from OAM
// every line.

module vdp_obj
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
	input  wire [7:0]  disp_x,
	input  wire [2:0]  phase,
	input  wire        in_window,
	input  wire        buf_hold,   // RASTER_DEBUG BH: keep the old buffers

	// OBJ_CTRL and the shared tile layout.
	input  wire [7:0]  obj_split,
	input  wire [2:0]  obj_th0,
	input  wire [2:0]  obj_th1,
	input  wire        obj_8bpp,
	input  wire [31:0] obj_subpal,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [7:0]  char_split,   // only seven bits can reach a 64 KB address
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [15:0] data_base,  // first byte of character data, from vdp_bg

	// OAM read port.
	output wire [6:0]  oam_addr,
	input  wire [31:0] oam_data,

	// Tile VRAM render port.
	output wire [12:0] chr_addr,
	input  wire [63:0] chr_data,

	// One 8-bit palette index per layer, valid from phase 2.
	output reg  [7:0]  pix0,
	output reg  [7:0]  pix1
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_walk;
	/* verilator lint_on UNUSEDSIGNAL */

	// Character columns one line may draw, across both layers.
	localparam [5:0] CHARS_PER_LINE = 6'd32;

	localparam [2:0] S_CLEAR = 3'd0;
	localparam [2:0] S_NEXT  = 3'd1;
	localparam [2:0] S_TEST  = 3'd2;
	localparam [2:0] S_FETCH = 3'd3;
	localparam [2:0] S_WORD  = 3'd4;
	localparam [2:0] S_EMIT  = 3'd5;
	localparam [2:0] S_DONE  = 3'd6;

	reg [2:0] state;
	reg [7:0] clr;          // 0..255 while clearing, also the write address
	reg [6:0] id;           // OAM entry being walked
	reg       bank;
	reg [5:0] chars_left;   // character columns still available this line

	// Per-object working set, latched in S_TEST.
	reg        o_layer;
	reg [8:0]  o_startx;
	reg [5:0]  o_tile_y;
	reg [1:0]  o_ncols;     // characters across, minus one
	reg [1:0]  o_group;     // character column being emitted
	reg [2:0]  o_ox;        // pixel within the character
	reg        o_flipx;
	reg [10:0] o_chr;
	reg [3:0]  o_subpal;
	/* verilator lint_off UNUSEDSIGNAL */
	reg [15:0] o_rowbase;   // kept for the byte select
	/* verilator lint_on UNUSEDSIGNAL */
	reg [63:0] o_word;

	wire obj_slot = ~in_window | (phase == 3'd2) | (phase == 3'd3)
	              | (phase == 3'd6) | (phase == 3'd7);

	assign oam_addr = id;

	// ---- entry decode --------------------------------------------------------

	wire [7:0]  e_chr    = oam_data[31:24];
	wire [8:0]  e_posy   = {oam_data[9], oam_data[23:16]};
	wire        e_flipy  = oam_data[15];
	wire        e_flipx  = oam_data[14];
	wire [1:0]  e_subpal = oam_data[13:12];
	wire [1:0]  e_size   = oam_data[11:10];
	wire [8:0]  e_posx   = {oam_data[8], oam_data[7:0]};

	// Height mask and the number of character columns, minus one.
	reg [5:0] e_hmask;
	reg [1:0] e_cols;
	always @* begin
		case (e_size)
		2'd0:    begin e_hmask = 6'd7;  e_cols = 2'd0; end   // 8x8
		2'd1:    begin e_hmask = 6'd15; e_cols = 2'd1; end   // 16x16
		2'd2:    begin e_hmask = 6'd31; e_cols = 2'd1; end   // 16x32
		default: begin e_hmask = 6'd31; e_cols = 2'd3; end   // 32x32
		endcase
	end

	// SPLIT is an 8-bit subtraction and only its sign bit picks the layer.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [7:0] e_test = {1'b0, id} - obj_split;
	/* verilator lint_on UNUSEDSIGNAL */
	wire       e_layer = e_test[7];

	// The object covers this line if fill_y falls in its height, including the
	// wrap at 512 from a negative position.
	wire [8:0] e_rel_y = fill_y - e_posy;
	wire       e_on    = (e_rel_y <= {3'd0, e_hmask});

	wire [5:0] e_tile_y = e_rel_y[5:0] ^ (e_flipy ? e_hmask : 6'd0);
	wire [2:0] e_th     = e_layer ? obj_th1 : obj_th0;

	// ---- character address ---------------------------------------------------
	// The column walks the object left to right on screen, flip applied, and
	// both axes wrap inside the 8-wide grid. In 4bpp the character split adds
	// two grid rows per reserved 8bpp row to the index before the wrap.
	wire [7:0] split_rows = obj_8bpp ? 8'd0 : {char_split[6:0], 1'b0};
	wire [1:0] col_sel = o_flipx ? (o_ncols - o_group) : o_group;
	wire [2:0] chr_col = o_chr[2:0]  + {1'b0, col_sel};
	/* verilator lint_off UNUSEDSIGNAL */
	wire [8:0] chr_row9 = {1'b0, o_chr[10:3]} + {1'b0, split_rows}
	                    + {6'd0, o_tile_y[5:3]};
	/* verilator lint_on UNUSEDSIGNAL */
	wire [7:0] chr_row  = chr_row9[7:0];        // the row wraps at 256, as it must
	wire [10:0] chr_id = {chr_row, chr_col};

	wire [16:0] chr_off = {chr_id, o_tile_y[2:0], 3'd0};
	wire [15:0] row_base = obj_8bpp ? (data_base + chr_off[15:0])
	                                : (data_base + chr_off[16:1]);

	// The RAM samples this in the fetch slot; the register below only keeps the
	// address for the byte select while the row is being emitted.
	assign chr_addr = row_base[15:3];

	// ---- emit ----------------------------------------------------------------

	wire [2:0] fine_x   = o_ox ^ (o_flipx ? 3'd7 : 3'd0);
	wire [2:0] byte_off = obj_8bpp ? fine_x : {1'b0, fine_x[2:1]};
	wire [2:0] byte_in  = o_rowbase[2:0] + byte_off;
	wire [5:0] byte_rev = 6'd7 - {3'd0, byte_in};
	wire [7:0] raw_byte = o_word[byte_rev * 6'd8 +: 8];
	wire [3:0] raw_nib  = fine_x[0] ? raw_byte[3:0] : raw_byte[7:4];

	wire [7:0] emit_px = obj_8bpp ? raw_byte
	                   : (raw_nib == 4'd0) ? 8'd0 : {o_subpal, raw_nib};

	// Screen position of the pixel about to be emitted. Nine bits, so an object
	// placed off the left edge wraps past 255 and is not drawn.
	wire [8:0] emit_x  = o_startx + {4'd0, o_group, o_ox};
	wire       emit_on = ~emit_x[8] & (emit_px != 8'd0);

	// A 256-bit shadow of each buffer says which pixels are taken, so the walk
	// can skip an occupied one without a read port.
	reg [255:0] taken [0:1];
	wire        px_free = ~taken[o_layer][emit_x[7:0]];
	wire        do_write = (state == S_EMIT) & emit_on & px_free;

	// ---- line buffers --------------------------------------------------------

	wire [8:0] wr_addr = (state == S_CLEAR) ? {bank, clr} : {bank, emit_x[7:0]};
	wire [7:0] wr_data = (state == S_CLEAR) ? 8'd0 : emit_px;
	wire       wr_en0  = (state == S_CLEAR) | (do_write & ~o_layer);
	wire       wr_en1  = (state == S_CLEAR) | (do_write &  o_layer);

	wire [7:0] buf_q0, buf_q1;

	cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(8)) u_buf0 (
		.clk_i     (clk),
		.addr_a_i  ({~bank, disp_x}),
		.wren_a_i  (1'b0),
		.wdata_a_i (8'd0),
		.q_a_o     (buf_q0),
		.addr_b_i  (wr_addr),
		.wren_b_i  (wr_en0),
		.wdata_b_i (wr_data),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_b_o     ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	cache_ram_dp #(.ADDR_WIDTH(9), .DATA_WIDTH(8)) u_buf1 (
		.clk_i     (clk),
		.addr_a_i  ({~bank, disp_x}),
		.wren_a_i  (1'b0),
		.wdata_a_i (8'd0),
		.q_a_o     (buf_q1),
		.addr_b_i  (wr_addr),
		.wren_b_i  (wr_en1),
		.wdata_b_i (wr_data),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_b_o     ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	always @(posedge clk) begin
		if (reset) begin
			pix0 <= 8'd0;
			pix1 <= 8'd0;
		end else if (phase == 3'd1) begin
			pix0 <= buf_q0;
			pix1 <= buf_q1;
		end
	end

	// ---- the walk ------------------------------------------------------------

	always @(posedge clk) begin
		if (reset) begin
			state    <= ss_walk[2:0];
			id       <= ss_walk[9:3];
			bank     <= ss_walk[10];
			clr      <= 8'd0;
			o_layer  <= 1'b0;
			o_startx <= 9'd0;
			o_tile_y <= 6'd0;
			o_ncols  <= 2'd0;
			o_group  <= 2'd0;
			o_ox     <= 3'd0;
			o_flipx  <= 1'b0;
			o_chr    <= 11'd0;
			o_subpal <= 4'd0;
			o_rowbase <= 16'd0;
			o_word   <= 64'd0;
			chars_left <= CHARS_PER_LINE;
			taken[0] <= 256'd0;
			taken[1] <= 256'd0;
		end else if (line_start) begin
			bank  <= ~bank;
			clr   <= 8'd0;
			id    <= 7'd0;
			chars_left <= CHARS_PER_LINE;
			state <= buf_hold ? S_NEXT : S_CLEAR;
			// The hold bit only skips the wipe: untouched pixels keep last
			// line's colour (vertical streaking), but an object that covers
			// them still wins, so the claim record is cleared either way.
			taken[0] <= 256'd0;
			taken[1] <= 256'd0;
		end else begin
			case (state)
			S_CLEAR: begin
				clr <= clr + 8'd1;
				if (clr == 8'd255) state <= S_NEXT;
			end

			S_NEXT: state <= S_TEST;      // the OAM word lands next cycle

			S_TEST: begin
				if (e_on) begin
					o_layer   <= e_layer;
					o_startx  <= e_posx;
					o_tile_y  <= e_tile_y;
					o_ncols   <= e_cols;
					o_group   <= 2'd0;
					o_ox      <= 3'd0;
					o_flipx   <= e_flipx;
					o_chr     <= {e_th, e_chr};
					o_subpal  <= obj_subpal[{e_layer, 4'd0} + {2'd0, e_subpal} * 4 +: 4];
					state     <= S_FETCH;
				end else if (id == 7'd127) begin
					state <= S_DONE;
				end else begin
					id    <= id + 7'd1;
					state <= S_NEXT;
				end
			end

			S_FETCH: if (obj_slot) begin
				o_rowbase <= row_base;
				state     <= S_WORD;
			end

			S_WORD: begin
				o_word <= chr_data;
				o_ox   <= 3'd0;
				state  <= S_EMIT;
			end

			S_EMIT: begin
				if (do_write) taken[o_layer][emit_x[7:0]] <= 1'b1;
				o_ox <= o_ox + 3'd1;
				if (o_ox == 3'd7) begin
					// One character column drawn. When the budget is gone the
					// rest of the line's objects are abandoned.
					chars_left <= chars_left - 6'd1;
					if (chars_left == 6'd1) begin
						state <= S_DONE;
					end else if (o_group == o_ncols) begin
						if (id == 7'd127) begin
							state <= S_DONE;
						end else begin
							id    <= id + 7'd1;
							state <= S_NEXT;
						end
					end else begin
						o_group <= o_group + 2'd1;
						state   <= S_FETCH;
					end
				end
			end

			default: ;                     // S_DONE: idle until the next line
			endcase
		end
	end

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"


	ss_reg #(
		.ADDR    (SSW_VDP_OBJ),
		.DEFAULT ({53'd0, 1'b0, 7'd0, S_DONE})
	) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({53'd0, bank, id, state}),
		.dout_o     (ss_walk)
	);

endmodule
