// RH-7500 register file. The controller, printer and expansion registers live
// with their own logic in vdp_io_ctrl, vdp_io_print and vdp_io_exp.
//
// The bus is the VDP's internal one: a word address, a write strobe with two
// byte enables, and a combinational read. `hit` says the address decoded to
// something and gates the open-bus read latch. Unused bits read back as zero.
//
// Twelve savestate words carry the whole file; the reset branch restores from
// them, so every power-on value lives in a word's DEFAULT.
//
// Address groups, relative to the VDP's 1 MB window:
//
//   0x58xxx  general      MODE, HCOUNT, VCOUNT, TRIGGER, RASTER_DMA_CTRL
//   0x59xxx  bitmap       scroll, position, size, mode, subpalette, colour latch
//   0x5Axxx  tile         BG control and scroll, OBJ control, character split
//   0x5Bxxx  display      blend, layer enables, screen control, backdrops, capture
//   0x5Cxxx  interrupts   enables and the IRQ0 compare values
//   0x5Exxx  bitmap mem   fast access, flash-write mask and value
//   0x5Fxxx  fill trigger word address selects the row to flash-write
//   0x60xxx  debug        sync calibration, render halt bits
//
// The four per-layer bitmap registers are selected by address bits 2-1, so
// each occupies eight bytes.

module vdp_regs
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

	// Internal register bus.
	input  wire [19:1] addr,
	input  wire        wr,
	input  wire [1:0]  be,       // 1 = high byte, 0 = low byte
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,
	output reg         hit,

	// Live raster position, for the read-only counters.
	input  wire [8:0]  hcount,
	input  wire [8:0]  vcount,

	// ---- general ----------------------------------------------------------
	output reg         mode_unk1,
	output reg         mode_cmode,   // 1 = matrix controller scan
	output reg         mode_mcnt,    // 1 = mouse counters run
	output reg         mode_unk2,
	output reg         mode_vidh,    // 1 = 240 active lines

	output reg         trig_psen,    // one-cycle pulses from TRIGGER
	output reg         trig_adc,
	output reg         trig_cap,

	output reg         rdma_en,
	output reg         rdma_tm,      // 0 = per frame, 1 = per line

	// ---- bitmap layers ----------------------------------------------------
	output reg  [35:0] bm_scrollx,   // 4 x 9, unsigned
	output reg  [35:0] bm_scrolly,
	output reg  [35:0] bm_posx,      // 4 x 9, signed
	output reg  [35:0] bm_posy,
	output reg  [31:0] bm_startx,    // 4 x 8, first visible column
	output reg  [31:0] bm_endx,      // 4 x 8, last visible column
	output reg  [31:0] bm_endy,      // 4 x 8, last visible row
	output reg  [2:0]  bm_mode,
	output reg  [15:0] bm_subpal,    // BM0 in bits 15-12 down to BM3 in 3-0
	output reg  [3:0]  bm_latch_en,
	output reg  [31:0] bm_latch_thrs,

	// ---- backgrounds and objects -----------------------------------------
	output reg  [1:0]  bg0_tsz,
	output reg  [1:0]  bg1_tsz,
	output reg         bg0_8bpp,
	output reg  [1:0]  bg_map_size,  // 0 64x64, 1 64x32, 2 32x64, 3 32x32
	output reg         bg_map_share,
	output reg  [47:0] bg_scroll,    // BG0 X, BG0 Y, BG1 X, BG1 Y, 12 bits each
	output reg  [31:0] bg_subpal,    // BG0 then BG1, four nibbles each
	output reg  [7:0]  obj_split,
	output reg  [2:0]  obj_th0,
	output reg  [2:0]  obj_th1,
	output reg         obj_8bpp,
	output reg  [31:0] obj_subpal,
	output reg  [7:0]  char_split,

	// ---- display ----------------------------------------------------------
	output reg  [2:0]  blend_mode,
	output reg  [15:0] layer_ctrl,
	output reg  [3:0]  prio,
	output reg         screen_b_col,  // SBCOL
	output reg         screen_b_en,
	output reg         screen_a_en,
	output reg         blend_sub,     // CSUB
	output reg  [14:0] backdrop_a,
	output reg  [14:0] backdrop_b,
	output reg  [1:0]  cap_mode,
	output reg  [7:0]  cap_line,
	output reg  [1:0]  cap_unk,

	// ---- interrupts -------------------------------------------------------
	output reg  [7:0]  irq_ctrl,
	output reg  [8:0]  irq0_hcmp,
	output reg  [8:0]  irq0_vcmp,

	// ---- bitmap memory ----------------------------------------------------
	output reg         bm_fast,
	output reg  [1:0]  bm_mem_unk,
	output reg  [7:0]  fill_mask,
	output reg         fill_mask_unk,
	output reg  [7:0]  fill_value,
	output reg         fill_trig,     // one-cycle pulse
	output reg  [8:0]  fill_row,

	// ---- debug ------------------------------------------------------------
	output reg  [1:0]  sync_hcal,
	output reg  [1:0]  sync_vcal,
	output reg         dbg_raster_halt,
	output reg         dbg_render_dis,
	output reg         dbg_raster_reset,
	output reg         dbg_buf_hold
);

	// MODE bit 0. Stored only so software reads back what it wrote.
	reg mode_vids;

	// Group select: address bits 19-16 pick the page and 15-12 the group.
	wire page5 = (addr[19:16] == 4'h5);
	wire page6 = (addr[19:16] == 4'h6);

	wire g_general = page5 & (addr[15:12] == 4'h8);
	wire g_bitmap  = page5 & (addr[15:12] == 4'h9);
	wire g_tile    = page5 & (addr[15:12] == 4'hA);
	wire g_display = page5 & (addr[15:12] == 4'hB);
	wire g_irq     = page5 & (addr[15:12] == 4'hC);
	wire g_bmmem   = page5 & (addr[15:12] == 4'hE);
	wire g_fill    = page5 & (addr[15:12] == 4'hF);
	wire g_debug   = page6 & (addr[15:12] == 4'h0);

	// Word offset within a 4 KB group. The bitmap registers repeat every four
	// words with the layer index in the low two.
	wire [10:0] woff = addr[11:1];
	wire [1:0]  bmi  = woff[1:0];
	wire [8:0]  bmg  = woff[10:2];

	// BG_SCROLL sits at words 1-4 and the subpalette pairs at 5-6 and 9-10.
	wire [1:0] bg_scroll_idx = woff[1:0] - 2'd1;
	wire       bg_subpal_idx = ~woff[0];
	wire       obj_subpal_idx = ~woff[0];

	// A write reaches a byte only if its enable is set.
	wire wr_lo = wr & be[0];
	wire wr_hi = wr & be[1];

	// ---- savestate words -----------------------------------------------------

	`include "ss_map.svh"

	wire [63:0] ss_w [0:11];
	wire [63:0] ss_b [0:11];
	wire [63:0] ss_q [0:11];

	assign ss_dout = ss_q[0]  | ss_q[1] | ss_q[2]  | ss_q[3]  | ss_q[4]  | ss_q[5]
	               | ss_q[6]  | ss_q[7] | ss_q[8]  | ss_q[9]  | ss_q[10] | ss_q[11];

	genvar w;
	generate
		for (w = 0; w < 12; w = w + 1) begin : g_ss
			ss_reg #(
				.ADDR    (SSW_VDP_REGS + w),
				.DEFAULT (64'd0)
			) u_ss (
				.clk_i      (ss_clk),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				.bus_rst_i  (ss_rst),
				.bus_dout_o (ss_q[w]),
				.din_i      (ss_b[w]),
				.dout_o     (ss_w[w])
			);
		end
	endgenerate

	// Live state into the words. Every register in the file appears once.
	assign ss_b[0]  = {30'd0, irq0_vcmp, irq0_hcmp, irq_ctrl, rdma_tm, rdma_en,
	                   mode_unk1, mode_cmode, mode_mcnt, mode_unk2, mode_vidh, mode_vids};
	assign ss_b[1]  = {21'd0, bm_latch_en, bm_mode, bm_scrollx};
	assign ss_b[2]  = {28'd0, bm_scrolly};
	assign ss_b[3]  = {28'd0, bm_posx};
	assign ss_b[4]  = {28'd0, bm_posy};
	assign ss_b[5]  = {bm_startx, bm_endx};
	assign ss_b[6]  = {16'd0, bm_subpal, bm_endy};
	assign ss_b[7]  = {1'd0, char_split, obj_8bpp, obj_th1, obj_th0, obj_split,
	                   bg_map_share, bg_map_size, bg0_8bpp, bg1_tsz, bg0_tsz,
	                   bm_latch_thrs};
	assign ss_b[8]  = {5'd0, blend_sub, screen_a_en, screen_b_en, screen_b_col,
	                   prio, blend_mode, bg_scroll};
	assign ss_b[9]  = {obj_subpal, bg_subpal};
	assign ss_b[10] = {6'd0, cap_unk, cap_line, cap_mode, backdrop_b, backdrop_a,
	                   layer_ctrl};
	assign ss_b[11] = {36'd0, dbg_buf_hold, dbg_raster_reset, dbg_render_dis,
	                   dbg_raster_halt, sync_vcal, sync_hcal, fill_value,
	                   fill_mask_unk, fill_mask, bm_mem_unk, bm_fast};

	always @(posedge clk) begin
		trig_psen <= 1'b0;
		trig_adc  <= 1'b0;
		trig_cap  <= 1'b0;
		fill_trig <= 1'b0;

		if (reset) begin
			{mode_unk1, mode_cmode, mode_mcnt, mode_unk2, mode_vidh, mode_vids}
			              <= ss_w[0][5:0];
			rdma_en       <= ss_w[0][6];
			rdma_tm       <= ss_w[0][7];
			irq_ctrl      <= ss_w[0][15:8];
			irq0_hcmp     <= ss_w[0][24:16];
			irq0_vcmp     <= ss_w[0][33:25];

			bm_scrollx    <= ss_w[1][35:0];
			bm_mode       <= ss_w[1][38:36];
			bm_latch_en   <= ss_w[1][42:39];
			bm_scrolly    <= ss_w[2][35:0];
			bm_posx       <= ss_w[3][35:0];
			bm_posy       <= ss_w[4][35:0];
			bm_endx       <= ss_w[5][31:0];
			bm_startx     <= ss_w[5][63:32];
			bm_endy       <= ss_w[6][31:0];
			bm_subpal     <= ss_w[6][47:32];

			bm_latch_thrs <= ss_w[7][31:0];
			bg0_tsz       <= ss_w[7][33:32];
			bg1_tsz       <= ss_w[7][35:34];
			bg0_8bpp      <= ss_w[7][36];
			bg_map_size   <= ss_w[7][38:37];
			bg_map_share  <= ss_w[7][39];
			obj_split     <= ss_w[7][47:40];
			obj_th0       <= ss_w[7][50:48];
			obj_th1       <= ss_w[7][53:51];
			obj_8bpp      <= ss_w[7][54];
			char_split    <= ss_w[7][62:55];

			bg_scroll     <= ss_w[8][47:0];
			blend_mode    <= ss_w[8][50:48];
			prio          <= ss_w[8][54:51];
			screen_b_col  <= ss_w[8][55];
			screen_b_en   <= ss_w[8][56];
			screen_a_en   <= ss_w[8][57];
			blend_sub     <= ss_w[8][58];

			bg_subpal     <= ss_w[9][31:0];
			obj_subpal    <= ss_w[9][63:32];

			layer_ctrl    <= ss_w[10][15:0];
			backdrop_a    <= ss_w[10][30:16];
			backdrop_b    <= ss_w[10][45:31];
			cap_mode      <= ss_w[10][47:46];
			cap_line      <= ss_w[10][55:48];
			cap_unk       <= ss_w[10][57:56];

			bm_fast          <= ss_w[11][0];
			bm_mem_unk       <= ss_w[11][2:1];
			fill_mask        <= ss_w[11][10:3];
			fill_mask_unk    <= ss_w[11][11];
			fill_value       <= ss_w[11][19:12];
			sync_hcal        <= ss_w[11][21:20];
			sync_vcal        <= ss_w[11][23:22];
			dbg_raster_halt  <= ss_w[11][24];
			dbg_render_dis   <= ss_w[11][25];
			dbg_raster_reset <= ss_w[11][26];
			dbg_buf_hold     <= ss_w[11][27];

			// Only live between the trigger and the fill.
			fill_row <= 9'd0;

		end else if (wr) begin
			// ---- 0x58xxx general ------------------------------------------
			if (g_general) begin
				case (woff)
				11'h000: if (wr_lo) begin
					mode_vids  <= wdata[0];
					mode_vidh  <= wdata[1];
					mode_unk2  <= wdata[2];
					mode_mcnt  <= wdata[3];
					mode_cmode <= wdata[4];
					mode_unk1  <= wdata[5];
				end
				11'h003: if (wr_lo) begin       // TRIGGER, write only
					trig_cap  <= wdata[0];
					trig_adc  <= wdata[1];
					trig_psen <= wdata[2];
				end
				11'h004: if (wr_lo) begin
					rdma_en <= wdata[0];
					rdma_tm <= wdata[1];
				end
				default: ;
				endcase
			end

			// ---- 0x59xxx bitmap layers ------------------------------------
			if (g_bitmap) begin
				case (bmg)
				9'd0: begin                                    // BM_SCROLLX
					if (wr_lo) bm_scrollx[bmi*9 +: 8] <= wdata[7:0];
					if (wr_hi) bm_scrollx[bmi*9 + 8]  <= wdata[8];
				end
				9'd1: begin                                    // BM_SCROLLY
					if (wr_lo) bm_scrolly[bmi*9 +: 8] <= wdata[7:0];
					if (wr_hi) bm_scrolly[bmi*9 + 8]  <= wdata[8];
				end
				9'd2: begin                                    // BM_POSX
					if (wr_lo) bm_posx[bmi*9 +: 8] <= wdata[7:0];
					if (wr_hi) bm_posx[bmi*9 + 8]  <= wdata[8];
				end
				9'd3: begin                                    // BM_POSY
					if (wr_lo) bm_posy[bmi*9 +: 8] <= wdata[7:0];
					if (wr_hi) bm_posy[bmi*9 + 8]  <= wdata[8];
				end
				9'd4: begin                                    // BM_WIDTH
					if (wr_lo) bm_endx[bmi*8 +: 8]   <= wdata[7:0];
					if (wr_hi) bm_startx[bmi*8 +: 8] <= wdata[15:8];
				end
				9'd5: if (wr_lo) bm_endy[bmi*8 +: 8] <= wdata[7:0];
				9'd6: if (wr_lo) bm_mode <= wdata[2:0];        // BM_CTRL
				9'd8: begin                                    // BM_SUBPAL
					if (wr_lo) bm_subpal[7:0]  <= wdata[7:0];
					if (wr_hi) bm_subpal[15:8] <= wdata[15:8];
				end
				9'd10: begin                                   // BM_COL_LATCH
					if (wr_lo) bm_latch_thrs[bmi*8 +: 8] <= wdata[7:0];
					if (wr_hi) bm_latch_en[bmi]          <= wdata[8];
				end
				default: ;
				endcase
			end

			// ---- 0x5Axxx backgrounds and objects --------------------------
			if (g_tile) begin
				case (woff)
				11'h000: if (wr_lo) begin                      // BG_CTRL
					bg_map_share <= wdata[0];
					bg_map_size  <= wdata[2:1];
					bg0_8bpp     <= wdata[3];
					bg1_tsz      <= wdata[5:4];
					bg0_tsz      <= wdata[7:6];
				end
				11'h001, 11'h002, 11'h003, 11'h004: begin      // BG_SCROLL
					if (wr_lo) bg_scroll[bg_scroll_idx*12 +: 8]     <= wdata[7:0];
					if (wr_hi) bg_scroll[bg_scroll_idx*12 + 8 +: 4] <= wdata[11:8];
				end
				11'h005, 11'h006: begin                        // BG_SUBPAL
					if (wr_lo) bg_subpal[bg_subpal_idx*16 +: 8]     <= wdata[7:0];
					if (wr_hi) bg_subpal[bg_subpal_idx*16 + 8 +: 8] <= wdata[15:8];
				end
				11'h008: begin                                 // OBJ_CTRL
					if (wr_lo) obj_split <= wdata[7:0];
					if (wr_hi) begin
						obj_th1  <= wdata[10:8];
						obj_th0  <= wdata[13:11];
						obj_8bpp <= wdata[14];
					end
				end
				11'h009, 11'h00A: begin                        // OBJ_SUBPAL
					if (wr_lo) obj_subpal[obj_subpal_idx*16 +: 8]     <= wdata[7:0];
					if (wr_hi) obj_subpal[obj_subpal_idx*16 + 8 +: 8] <= wdata[15:8];
				end
				11'h010: if (wr_lo) char_split <= wdata[7:0];  // CHAR_SPLIT
				default: ;
				endcase
			end

			// ---- 0x5Bxxx display ------------------------------------------
			if (g_display) begin
				case (woff)
				11'h000: if (wr_lo) blend_mode <= wdata[2:0];
				11'h001: begin                                 // LAYER_CTRL
					if (wr_lo) layer_ctrl[7:0]  <= wdata[7:0];
					if (wr_hi) layer_ctrl[15:8] <= wdata[15:8];
				end
				11'h002: if (wr_lo) begin                      // SCREEN_CTRL
					prio         <= wdata[3:0];
					screen_b_col <= wdata[4];
					screen_b_en  <= wdata[5];
					screen_a_en  <= wdata[6];
					blend_sub    <= wdata[7];
				end
				11'h003: begin                                 // BACKDROP_B
					if (wr_lo) backdrop_b[7:0]  <= wdata[7:0];
					if (wr_hi) backdrop_b[14:8] <= wdata[14:8];
				end
				11'h004: begin                                 // BACKDROP_A
					if (wr_lo) backdrop_a[7:0]  <= wdata[7:0];
					if (wr_hi) backdrop_a[14:8] <= wdata[14:8];
				end
				11'h005: begin                                 // CAPTURE_CTRL
					if (wr_lo) cap_line <= wdata[7:0];
					if (wr_hi) begin
						cap_mode <= wdata[9:8];
						cap_unk  <= wdata[11:10];
					end
				end
				default: ;
				endcase
			end

			// ---- 0x5Cxxx interrupts ---------------------------------------
			if (g_irq) begin
				case (woff)
				11'h000: if (wr_lo) irq_ctrl <= wdata[7:0];
				11'h001: begin                                 // IRQ0_HCMP
					if (wr_lo) irq0_hcmp[7:0] <= wdata[7:0];
					if (wr_hi) irq0_hcmp[8]   <= wdata[8];
				end
				11'h002: begin                                 // IRQ0_VCMP
					if (wr_lo) irq0_vcmp[7:0] <= wdata[7:0];
					if (wr_hi) irq0_vcmp[8]   <= wdata[8];
				end
				default: ;
				endcase
			end

			// ---- 0x5Exxx bitmap memory control ----------------------------
			if (g_bmmem) begin
				case (woff)
				11'h000: if (wr_lo) begin                      // BM_MEM_CTRL
					bm_fast    <= wdata[0];
					bm_mem_unk <= wdata[2:1];
				end
				11'h001: begin                                 // BM_FILL_MASK
					if (wr_lo) fill_mask     <= wdata[7:0];
					if (wr_hi) fill_mask_unk <= wdata[8];
				end
				11'h002: if (wr_lo) fill_value <= wdata[7:0];  // BM_FILL_VALUE
				default: ;
				endcase
			end

			// ---- 0x5Fxxx flash-write trigger ------------------------------
			// The address is the row; the data is ignored. A byte write still
			// selects a word address, so either enable triggers it.
			if (g_fill) begin
				fill_trig <= 1'b1;
				fill_row  <= woff[8:0];
			end

			// ---- 0x60xxx debug --------------------------------------------
			if (g_debug) begin
				case (woff)
				11'h000: if (wr_lo) begin                      // SYNC_CALIBRATE
					sync_hcal <= wdata[1:0];
					sync_vcal <= wdata[3:2];
				end
				11'h001: if (wr_hi) begin                      // VIDEO_DEBUG
					dbg_raster_reset <= wdata[9];
					dbg_render_dis   <= wdata[10];
					dbg_raster_halt  <= wdata[11];
				end
				11'h002: if (wr_lo) dbg_buf_hold <= wdata[7]; // RASTER_DEBUG
				default: ;
				endcase
			end
		end
	end

	// ---- read path -----------------------------------------------------------
	// Write-only registers (TRIGGER, the fill trigger, sync and debug) decode
	// as unmapped and leave the read latch alone.

	always @* begin
		rdata = 16'd0;
		hit   = 1'b0;

		if (g_general) begin
			case (woff)
			11'h000: begin
				// Bit 2 is kept but reads back 0.
				hit = 1'b1;
				rdata = {10'd0, mode_unk1, mode_cmode, mode_mcnt,
				         1'b0, mode_vidh, mode_vids};
			end
			11'h001: begin hit = 1'b1; rdata = {7'd0, hcount}; end
			11'h002: begin hit = 1'b1; rdata = {7'd0, vcount}; end
			11'h004: begin hit = 1'b1; rdata = {14'd0, rdma_tm, rdma_en}; end
			default: ;
			endcase
		end

		if (g_bitmap) begin
			case (bmg)
			9'd0: begin hit = 1'b1; rdata = {7'd0, bm_scrollx[bmi*9 +: 9]}; end
			9'd1: begin hit = 1'b1; rdata = {7'd0, bm_scrolly[bmi*9 +: 9]}; end
			9'd2: begin hit = 1'b1; rdata = {7'd0, bm_posx[bmi*9 +: 9]}; end
			9'd3: begin hit = 1'b1; rdata = {7'd0, bm_posy[bmi*9 +: 9]}; end
			9'd4: begin hit = 1'b1; rdata = {bm_startx[bmi*8 +: 8], bm_endx[bmi*8 +: 8]}; end
			9'd5: begin hit = 1'b1; rdata = {8'd0, bm_endy[bmi*8 +: 8]}; end
			9'd6: begin hit = 1'b1; rdata = {13'd0, bm_mode}; end
			9'd8: begin hit = 1'b1; rdata = bm_subpal; end
			9'd10: begin hit = 1'b1;
				rdata = {7'd0, bm_latch_en[bmi], bm_latch_thrs[bmi*8 +: 8]}; end
			default: ;
			endcase
		end

		if (g_tile) begin
			case (woff)
			11'h000: begin hit = 1'b1;
				rdata = {8'd0, bg0_tsz, bg1_tsz, bg0_8bpp, bg_map_size, bg_map_share}; end
			11'h001, 11'h002, 11'h003, 11'h004: begin hit = 1'b1;
				rdata = {4'd0, bg_scroll[bg_scroll_idx*12 +: 12]}; end
			11'h005, 11'h006: begin hit = 1'b1;
				rdata = bg_subpal[bg_subpal_idx*16 +: 16]; end
			11'h008: begin hit = 1'b1;
				rdata = {1'b0, obj_8bpp, obj_th0, obj_th1, obj_split}; end
			11'h009, 11'h00A: begin hit = 1'b1;
				rdata = obj_subpal[obj_subpal_idx*16 +: 16]; end
			11'h010: begin hit = 1'b1; rdata = {8'd0, char_split}; end
			default: ;
			endcase
		end

		if (g_display) begin
			case (woff)
			11'h000: begin hit = 1'b1; rdata = {13'd0, blend_mode}; end
			11'h001: begin hit = 1'b1; rdata = layer_ctrl; end
			11'h002: begin hit = 1'b1;
				rdata = {8'd0, blend_sub, screen_a_en, screen_b_en,
				         screen_b_col, prio}; end
			11'h003: begin hit = 1'b1; rdata = {1'b0, backdrop_b}; end
			11'h004: begin hit = 1'b1; rdata = {1'b0, backdrop_a}; end
			11'h005: begin hit = 1'b1;
				rdata = {4'd0, cap_unk, cap_mode, cap_line}; end
			default: ;
			endcase
		end

		if (g_irq) begin
			case (woff)
			11'h000: begin hit = 1'b1; rdata = {8'd0, irq_ctrl}; end
			11'h001: begin hit = 1'b1; rdata = {7'd0, irq0_hcmp}; end
			11'h002: begin hit = 1'b1; rdata = {7'd0, irq0_vcmp}; end
			default: ;
			endcase
		end

		if (g_bmmem) begin
			case (woff)
			11'h000: begin hit = 1'b1; rdata = {13'd0, bm_mem_unk, bm_fast}; end
			11'h001: begin hit = 1'b1; rdata = {7'd0, fill_mask_unk, fill_mask}; end
			11'h002: begin hit = 1'b1; rdata = {8'd0, fill_value}; end
			default: ;
			endcase
		end
	end

endmodule
