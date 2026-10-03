// RH-7500 raster counters and sync generation. Everything is counted in VDP
// cycles; ce_vdp is one in two clk_video.
//
// A line is 1365 VDP cycles on NTSC, counted from the start of sync:
//
//   hcyc     0        sync              100
//          100        back porch        100
//          200        left border        52
//          252        active window    1029   (1024 render, 5 border)
//         1281        right border       52
//         1333        front porch        32
//
// HCOUNT counts pixels, 0..257 then -84..-1. HCOUNT 257 lasts one cycle:
//
//   HCOUNT   0..256   4 cycles each  = 1028      the active window
//   HCOUNT   257      1 cycle        =    1
//   HCOUNT -84..-1    4 cycles each  =  336      everything outside it
//                                     -----
//                                      1365
//
// Vertical: 9 border, 3 short, 3 long, 13 short sync, 9 border, 224 active,
// plus two lines in the top blanking, 263 in all. VCOUNT is 0 at the first
// active line, -39..-1 through blanking, and steps where HCOUNT resets at the
// right border, 84 cycles before vline does.
//
// HCAL shifts sync right in steps of two pixels and VCAL shifts vertical sync
// down in steps of two lines, relative to the measured runs at HCAL 2.

module vdp_raster
(
	input  wire clk,
	input  wire reset,
	input  wire ce_vdp,

	// Savestate scalar bus.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,
	output wire        ss_ready,

	input  wire mode_240,   // 0: 224 active lines. 1: 240

	input  wire [1:0] hcal, // SYNC_CALIBRATE, 2 is the measured default
	input  wire [1:0] vcal,

	// INTERRUPT_CTRL and the IRQ0 compare values.
	input  wire       nmi_en,
	input  wire       irq0_en,
	input  wire       irq0_vcmp_en,
	input  wire [8:0] irq0_hcmp,
	input  wire [8:0] irq0_vcmp,

	// RASTER_DMA_CTRL.
	input  wire       rdma_en,
	input  wire       rdma_line,   // 0 per frame, 1 per line

	output reg  [10:0] hcyc,        // VDP cycle within the line, 0 at the start of sync
	output reg  [9:0]  vline,       // line within the frame, 0 at the first sync line
	output wire [8:0]  hcount,      // the VDP's own HCOUNT
	output wire [8:0]  vcount,      // the VDP's own VCOUNT
	output reg  [8:0]  fetch_y,     // vcount a cycle late, for the fetch path

	output wire ce_pix,       // one per HCOUNT step, so one per line is short
	output wire ce_pix_half,  // ce_pix plus the half-pixel, for hi-res blending
	output wire [2:0] phase,  // clk_video cycle within the pixel, 0 at ce_pix
	output wire hsync,
	output wire vsync,
	output wire hvisible,     // left border, active window or right border
	output wire vvisible,     // top border, active lines or bottom border
	output wire hrender,      // the render window's horizontal half
	output wire vrender,      // its vertical half
	output wire render,       // the 1024-cycle window the layers draw into
	output wire fetch_win,    // render, moved one pixel earlier
	output wire border,       // visible but outside the render window

	output wire line_start,   // first cycle of a line
	output wire frame_start,  // first cycle of a frame

	// VCOUNT steps: the bitmap layers take their scroll for the next line.
	output wire vstep,

	// The VDP is about to use, or is using, the bitmap VRAM for itself.
	output wire bm_hold_rd,
	output wire bm_hold_wr,

	output wire nmi_n,        // 16-cycle low pulse, last active line, HCOUNT -84
	output wire irq0_n,       // 16-cycle low pulse on the raster compare
	output wire raster_dma    // DREQ0/IRQ1/PA13 request, high in blanking
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_pos;
	/* verilator lint_on UNUSEDSIGNAL */

	// Horizontal runs, in VDP cycles. They sum to 1365.
	localparam int unsigned H_WINDOW     = 1029;  // 1024 render + 5 border
	localparam int unsigned H_RENDER_LEN = 1024;

	localparam int unsigned H_SYNC = 100, H_BPORCH = 100, H_LBORDER = 52,
	                        H_RBORDER = 52,  H_FPORCH = 32;

	// Pixel slots in the negative HCOUNT run: blanking divided by four.
	localparam int unsigned H_NEG = 84;   // (1365 - 1029) / 4

	// Vertical runs, in lines. With 224 or 240 active they sum to 263.
	localparam int unsigned V_SHORT1 = 3, V_LONG = 3, V_SHORT2 = 13;
	localparam int unsigned V_UNACCOUNTED = 2;   // documented runs sum to 261 of 263
	localparam int unsigned V_BORDER_224 = 9, V_BORDER_240 = 1;
	localparam int unsigned V_ACTIVE_224 = 224, V_ACTIVE_240 = 240;

	wire [10:0] h_sync    = H_SYNC[10:0];
	wire [10:0] h_bporch  = H_BPORCH[10:0];
	wire [10:0] h_lborder = H_LBORDER[10:0];
	wire [10:0] h_rborder = H_RBORDER[10:0];
	wire [10:0] h_fporch  = H_FPORCH[10:0];
	wire [8:0]  h_neg     = H_NEG[8:0];

	// First cycle of each run.
	wire [10:0] h_bporch_at  = h_sync;
	wire [10:0] h_lborder_at = h_bporch_at  + h_bporch;
	wire [10:0] h_window_at  = h_lborder_at + h_lborder;
	wire [10:0] h_rborder_at = h_window_at  + H_WINDOW[10:0];
	wire [10:0] h_fporch_at  = h_rborder_at + h_rborder;
	wire [10:0] h_total      = h_fporch_at  + h_fporch;

	wire [9:0] v_border = mode_240 ? V_BORDER_240[9:0] : V_BORDER_224[9:0];
	wire [9:0] v_active = mode_240 ? V_ACTIVE_240[9:0] : V_ACTIVE_224[9:0];
	wire [9:0] v_unacc  = V_UNACCOUNTED[9:0];

	// First line of each run.
	wire [9:0] v_long_at    = V_SHORT1[9:0];
	wire [9:0] v_short2_at  = v_long_at    + V_LONG[9:0];
	wire [9:0] v_unacc_at   = v_short2_at  + V_SHORT2[9:0];
	wire [9:0] v_tborder_at = v_unacc_at   + v_unacc;
	wire [9:0] v_active_at  = v_tborder_at + v_border;
	wire [9:0] v_bborder_at = v_active_at  + v_active;
	wire [9:0] v_total      = v_bborder_at + v_border;

	// ---- counters -----------------------------------------------------------

	always @(posedge clk) begin
		if (reset) begin
			hcyc  <= ss_pos[10:0];
			vline <= ss_pos[20:11];
		end else if (ce_vdp) begin
			if (hcyc == h_total - 11'd1) begin
				hcyc <= 11'd0;
				vline <= (vline == v_total - 10'd1) ? 10'd0 : vline + 10'd1;
			end else begin
				hcyc <= hcyc + 11'd1;
			end
		end
	end

	assign line_start  = ce_vdp & (hcyc == 11'd0);
	assign frame_start = line_start & (vline == 10'd0);

	wire in_window = (hcyc >= h_window_at) && (hcyc < h_rborder_at);

	// The pixel phase restarts wherever HCOUNT does: the window and the right
	// border. `phase` is {pixel quarter, half of the VDP cycle} so the render
	// schedule gets clk_video cycles.
	reg [1:0] pix_phase;
	reg       vdp_half;
	wire pix_phase_reset = (hcyc == h_window_at) || (hcyc == h_rborder_at);
	assign ce_pix      = ce_vdp & (pix_phase == 2'd0 || pix_phase_reset);
	assign ce_pix_half = ce_vdp & (pix_phase == 2'd0 || pix_phase == 2'd2
	                               || pix_phase_reset);
	assign phase = {pix_phase, vdp_half};

	// A savestate may stop the video side only at the top of the frame, where
	// every line buffer, fetch queue and object walk is empty.
	assign ss_ready = (hcyc == 11'd0) && (vline == 10'd0)
	                  && (pix_phase == 2'd0) && !vdp_half;

	always @(posedge clk) begin
		if (reset) begin
			pix_phase <= ss_pos[22:21];
			vdp_half  <= 1'b0;
		end else begin
			vdp_half <= ~ce_vdp;
			if (ce_vdp) begin
				if (pix_phase_reset) pix_phase <= 2'd1;
				else                 pix_phase <= pix_phase + 2'd1;
			end
		end
	end

	// HCOUNT. Inside the window it is the pixel index; outside, it counts up
	// from -h_neg, and that run crosses the end of the line, so the offset
	// wraps with it.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [10:0] h_win_off = hcyc - h_window_at;                 // low bits are sub-pixel
	wire [11:0] h_neg_off = (hcyc >= h_rborder_at)
		? {1'b0, hcyc - h_rborder_at}
		: ({1'b0, hcyc} + {1'b0, h_total} - {1'b0, h_rborder_at});
	/* verilator lint_on UNUSEDSIGNAL */
	assign hcount = in_window ? h_win_off[10:2]
	                          : ((9'd0 - h_neg) + h_neg_off[10:2]);

	// VCOUNT steps at the right border, where HCOUNT resets, which is 84 VDP
	// cycles before vline steps. So for the tail of every line it is already
	// showing the next line's number.
	wire [9:0] vline_adj = (hcyc >= h_rborder_at)
		? ((vline == v_total - 10'd1) ? 10'd0 : vline + 10'd1)
		: vline;

	wire [9:0] v_from_active = (vline_adj >= v_active_at)
		? (vline_adj - v_active_at)
		: (vline_adj + v_total - v_active_at);
	assign vcount = (v_from_active < v_active)
		? v_from_active[8:0]
		: (v_from_active[8:0] - v_total[8:0]);

	wire v_in_active = (v_from_active < v_active);

	// VCOUNT plus the background scroll, wrap and shift is the longest path in
	// the chip. It changes once a line, so the fetch takes it a cycle late
	// from a register.
	always @(posedge clk) begin
		if (reset) fetch_y <= 9'd0;
		else       fetch_y <= vcount;
	end

	// ---- sync ---------------------------------------------------------------
	// HCAL shifts sync right in steps of two pixels, VCAL down in steps of two
	// lines, both relative to HCAL 2. The offset can go negative, so it is
	// taken modulo the line.

	wire [10:0] h_shift = {6'd0, hcal, 3'd0} - 11'd16;          // (hcal - 2) * 8
	wire [10:0] hs_at   = (h_shift[10] ? (h_total + h_shift) : h_shift);
	wire [11:0] hs_off  = (hcyc >= hs_at) ? {1'b0, hcyc - hs_at}
	                                      : ({1'b0, hcyc} + {1'b0, h_total} - {1'b0, hs_at});
	assign hsync = (hs_off < {1'b0, h_sync});

	wire [9:0] vs_at    = v_long_at + {7'd0, vcal, 1'b0};
	assign vsync = (vline >= vs_at) && (vline < vs_at + V_LONG[9:0]);

	assign hvisible = (hcyc >= h_lborder_at) && (hcyc < h_fporch_at);
	assign vvisible = (vline >= v_tborder_at);

	// The render window's two halves, each wholly inside its visible run. The
	// video output needs them apart for horizontal and vertical blank.
	assign hrender = (hcyc >= h_window_at)
		&& (hcyc <  h_window_at + H_RENDER_LEN[10:0]);
	assign vrender = (vline >= v_active_at) && (vline < v_bborder_at);

	assign render = hrender && vrender;
	assign border = hvisible && vvisible && !render;

	// The background layers are fetched one pixel ahead of the compositor, so
	// the first pixel of a line is fetched during the last pixel of the left
	// border and the tile VRAM slot schedule follows the fetch.
	assign fetch_win = hvisible && vvisible
		&& (hcyc >= h_window_at - 11'd4)
		&& (hcyc <  h_window_at + H_RENDER_LEN[10:0] - 11'd4)
		&& (vline >= v_active_at) && (vline < v_bborder_at);

	// ---- interrupts and the DMA signal --------------------------------------
	// Every VDP interrupt is a 16-VDP-cycle low pulse and the CPU takes the
	// falling edge. NMI sits where VCOUNT goes negative on the last active
	// line; IRQ0 sits wherever the compare says.

	wire last_active_line = (vline == v_bborder_at - 10'd1);
	wire nmi_win = last_active_line
		&& (hcyc >= h_rborder_at) && (hcyc < h_rborder_at + 11'd16);
	assign nmi_n = ~(nmi_en & nmi_win);

	// The compare hits on the first cycle of the matching HCOUNT, which is the
	// cycle where hcount changes to it. The pulse then runs 16 cycles from
	// there, so it is generated from a counter rather than a window.
	wire hcmp_hit = (hcount == irq0_hcmp)
		& (in_window ? (h_win_off[1:0] == 2'd0) : (h_neg_off[1:0] == 2'd0));
	wire vcmp_hit = ~irq0_vcmp_en | (vcount == irq0_vcmp);
	wire irq0_hit = irq0_en & hcmp_hit & vcmp_hit;

	reg [4:0] irq0_cnt;
	always @(posedge clk) begin
		if (reset)            irq0_cnt <= 5'd0;
		else if (ce_vdp) begin
			if (irq0_hit)          irq0_cnt <= 5'd16;
			else if (irq0_cnt != 0) irq0_cnt <= irq0_cnt - 5'd1;
		end
	end
	assign irq0_n = ~(irq0_cnt != 5'd0);

	// DREQ0/IRQ1/PA13. Per line the pin is high from HCOUNT -17 of an active
	// line to pixel 254, and requests the rest of the time. Per
	// frame it falls where VCOUNT steps into blanking and rises where it steps
	// to the first active line, since a level-sensed IRQ1 on it stops as line
	// 0 begins. Disabled, the pin stays high, so enabling it in blanking is an
	// edge.
	wire rdma_line_hi = (hcyc >= h_window_at - 11'd68) && (hcyc < h_rborder_at - 11'd6);
	assign raster_dma = rdma_en & ~(rdma_line ? (v_in_active & rdma_line_hi) : v_in_active);

	// ---- bitmap VRAM cycles the VDP keeps -------------------------------------
	// Each picture line opens, where HCOUNT resets, with seven of the VDP's own
	// bitmap VRAM cycles, four VDP clocks long and twelve apart, out to about
	// HCOUNT -63; none in vertical blanking. A CPU cycle may not start if it
	// would run into one: a read 4 clocks ahead, a write 3. Count and spacing
	// are fitted to measured CPU waits by position.
	localparam int unsigned BMS_N = 7, BMS_P = 12, BMS_W = 4;
	localparam int unsigned BMS_LRD = 4, BMS_LWR = 3;

	// Clocks since HCOUNT -84, plus 16 so a lead never goes below zero.
	wire [11:0] bm_pos = h_neg_off + 12'd16;
	reg hold_rd_c, hold_wr_c;
	integer st_k;
	always @* begin
		hold_rd_c = 1'b0;
		hold_wr_c = 1'b0;
		for (st_k = 0; st_k < BMS_N; st_k = st_k + 1) begin
			if (({20'd0, bm_pos} + BMS_LRD >= 16 + st_k * BMS_P)
			    && ({20'd0, bm_pos} < 16 + st_k * BMS_P + BMS_W))
				hold_rd_c = 1'b1;
			if (({20'd0, bm_pos} + BMS_LWR >= 16 + st_k * BMS_P)
			    && ({20'd0, bm_pos} < 16 + st_k * BMS_P + BMS_W))
				hold_wr_c = 1'b1;
		end
	end
	wire bm_line = ~in_window & (v_from_active < v_active);
	assign vstep = ce_vdp & (hcyc == h_rborder_at);
	assign bm_hold_rd = bm_line & hold_rd_c;
	assign bm_hold_wr = bm_line & hold_wr_c;

	// ---- savestate ----------------------------------------------------------

	`include "ss_map.svh"

	ss_reg #(
		.ADDR    (SSW_VDP_RASTER),
		.DEFAULT (64'd0)
	) u_ss_pos (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({41'd0, pix_phase, vline, hcyc}),
		.dout_o     (ss_pos)
	);

endmodule
