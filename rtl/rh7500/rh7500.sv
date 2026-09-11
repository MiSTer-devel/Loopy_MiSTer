// RH-7500: the Loopy's video display processor and, on the same die, most of
// the machine's memory-mapped IO.
//
// Pins modelled: the /CS4 bus (A19-A0, D15-D0 split into in, out and output
// enable, /RD, /WRH, /WRL, /WAIT), /NMI, /IRQ0, /IRQ2 and the raster DMA
// request, the controller port, the printer sensor and drive lines, the
// analogue mux inputs, the three expansion strobes, the sound control latch,
// and 15-bit RGB with syncs. Bitmap and tile VRAM are block RAM inside this
// module.
//
// The whole chip runs on clk_video with ce_vdp marking a VDP clock. A pixel is
// four VDP cycles, eight clk_video cycles, counted by `phase`:
//
//   phase 0  layer buffers read           phase 4  palette lookup issued
//   phase 1  layer pixels latched         phase 5  palette colours back
//   phase 2  priority, first two levels   phase 6  BG0 pixel resolved
//   phase 3  priority resolved            phase 7  blend, output latched
//
// The picture leaves the chip one pixel after the raster, so the sync and
// blanking signals are delayed by one pixel to match.

module rh7500
(
	input  wire clk,          // clk_video, 42.954545 MHz
	input  wire reset,
	input  wire ce_vdp,       // every second clk_video

	// Savestate scalar bus.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// Savestate bulk walk, one byte per request, on the video clock. The chip
	// is stopped, so the walk takes the memories' CPU ports outright; a restore
	// holds the chip in reset for the whole transfer, so the walk has its own.
	input  wire        ss_mem_reset,
	input  wire        ss_mem_valid,
	input  wire [2:0]  ss_mem_type,
	input  wire [16:0] ss_mem_addr,
	input  wire        ss_mem_we,
	input  wire [7:0]  ss_mem_wdata,
	output reg         ss_mem_ack,
	output reg  [7:0]  ss_mem_rdata,
	// The chip is at the top of a frame, where a savestate stops it.
	output wire        ss_ready,

	// CPU bus, area 4.
	input  wire        cs_n,
	input  wire        rd_n,
	input  wire        wrh_n,
	input  wire        wrl_n,
	input  wire [19:0] a,
	input  wire [15:0] d_i,
	output wire [15:0] d_o,
	output wire        d_oe,
	output wire        wait_n,

	// To the CPU.
	output wire        nmi_n,
	output wire        irq0_n,
	output wire        irq2_n,
	output wire        raster_dma,

	// Controller port.
	output wire [5:0]  ctrl_out,
	input  wire [7:0]  ctrl_in,

	// Printer, and the board's region jumper.
	input  wire [2:0]  print_sct,
	input  wire [2:0]  print_opto,
	input  wire        region_ntsc,
	output wire [3:0]  print_motor,
	output wire [3:0]  print_head,
	output wire [15:0] print_data,
	output wire        print_data_wr,
	output wire        print_enable,

	// Cartridge expansion.
	output wire [2:0]  exp_strobe_n,
	output wire [3:0]  exp_fast,
	output wire [11:0] sound_ctrl,

	// Video.
	output wire        ce_pix,
	output wire [14:0] rgb,
	output wire        hsync,
	output wire        vsync,
	output wire        hvisible,
	output wire        vvisible,
	output wire        hrender,
	output wire        vrender,
	output wire        render
);

	// ---- bus interface -------------------------------------------------------

	wire [19:1] cpu_addr;
	wire [15:0] cpu_wdata;
	wire [1:0]  cpu_be;
	wire        cpu_wr, cpu_rd, cpu_exp_busy;
	wire        sel_bitmap, sel_tile, sel_oam, sel_pal, sel_cap;
	wire        sel_reg, sel_fill, sel_io, sel_sound;
	wire [2:0]  sel_exp;
	wire [15:0] bus_rdata;
	wire        bus_hit;
	wire        bm_fast;

	// A flash write owns the bitmap memory's CPU port while it runs.
	wire        fill_busy;
	wire fill_stall = fill_busy & (sel_bitmap | sel_fill);

	vdp_cpu_if u_cpu (
		.clk        (clk),
		.reset      (reset),
		.ce_vdp     (ce_vdp),
		.cs_n       (cs_n),
		.rd_n       (rd_n),
		.wrh_n      (wrh_n),
		.wrl_n      (wrl_n),
		.a          (a),
		.d_i        (d_i),
		.d_o        (d_o),
		.d_oe       (d_oe),
		.wait_n     (wait_n),
		.bm_fast    (bm_fast),
		.stall      (fill_stall),
		.cpu_addr   (cpu_addr),
		.cpu_wdata  (cpu_wdata),
		.cpu_be     (cpu_be),
		.cpu_wr     (cpu_wr),
		.cpu_rd     (cpu_rd),
		// Nothing outside the expansion strobes needs the whole access.
		/* verilator lint_off PINCONNECTEMPTY */
		.cpu_busy   (),
		/* verilator lint_on PINCONNECTEMPTY */
		.cpu_exp_busy (cpu_exp_busy),
		.exp_fast   (exp_fast),
		.sel_bitmap (sel_bitmap),
		.sel_tile   (sel_tile),
		.sel_oam    (sel_oam),
		.sel_pal    (sel_pal),
		.sel_cap    (sel_cap),
		.sel_reg    (sel_reg),
		.sel_fill   (sel_fill),
		.sel_io     (sel_io),
		.sel_sound  (sel_sound),
		.sel_exp    (sel_exp),
		.rdata      (bus_rdata),
		.rdata_hit  (bus_hit)
	);

	// ---- registers -----------------------------------------------------------

	wire mode_unk1, mode_cmode, mode_mcnt, mode_unk2, mode_vidh;
	wire trig_psen, trig_adc, trig_cap;
	wire rdma_en, rdma_tm;
	wire [35:0] bm_scrollx, bm_scrolly, bm_posx, bm_posy;
	wire [31:0] bm_startx, bm_endx, bm_endy;
	wire [2:0]  bm_mode;
	wire [15:0] bm_subpal;
	wire [3:0]  bm_latch_en;
	wire [31:0] bm_latch_thrs;
	wire [1:0]  bg0_tsz, bg1_tsz, bg_map_size;
	wire        bg0_8bpp, bg_map_share;
	wire [47:0] bg_scroll;
	wire [31:0] bg_subpal, obj_subpal;
	wire [7:0]  obj_split, char_split;
	wire [2:0]  obj_th0, obj_th1;
	wire        obj_8bpp;
	wire [2:0]  blend_mode;
	wire [15:0] layer_ctrl;
	wire [3:0]  prio;
	wire        screen_b_col, screen_b_en, screen_a_en, blend_sub;
	wire [14:0] backdrop_a, backdrop_b;
	wire [1:0]  cap_mode, cap_unk;
	wire [7:0]  cap_line;
	wire [7:0]  irq_ctrl;
	wire [8:0]  irq0_hcmp, irq0_vcmp;
	wire [1:0]  bm_mem_unk;
	wire [7:0]  fill_mask, fill_value;
	wire        fill_mask_unk, fill_trig;
	wire [8:0]  fill_row;
	wire [1:0]  sync_hcal, sync_vcal;
	wire        dbg_raster_halt, dbg_render_dis, dbg_raster_reset, dbg_buf_hold;

	wire [15:0] reg_rdata;
	wire        reg_hit;
	wire [63:0] ss_regs;

	wire [8:0] hcount, vcount, fetch_y;

	vdp_regs u_regs (
		.clk              (clk),
		.reset            (reset),
		.ss_clk           (ss_clk),
		.ss_din           (ss_din),
		.ss_addr          (ss_addr),
		.ss_wren          (ss_wren),
		.ss_rst           (ss_rst),
		.ss_dout          (ss_regs),
		.addr             (cpu_addr),
		.wr               (cpu_wr & sel_reg),
		.be               (cpu_be),
		.wdata            (cpu_wdata),
		.rdata            (reg_rdata),
		.hit              (reg_hit),
		.hcount           (hcount),
		.vcount           (vcount),
		.mode_unk1        (mode_unk1),
		.mode_cmode       (mode_cmode),
		.mode_mcnt        (mode_mcnt),
		.mode_unk2        (mode_unk2),
		.mode_vidh        (mode_vidh),
		.trig_psen        (trig_psen),
		.trig_adc         (trig_adc),
		.trig_cap         (trig_cap),
		.rdma_en          (rdma_en),
		.rdma_tm          (rdma_tm),
		.bm_scrollx       (bm_scrollx),
		.bm_scrolly       (bm_scrolly),
		.bm_posx          (bm_posx),
		.bm_posy          (bm_posy),
		.bm_startx        (bm_startx),
		.bm_endx          (bm_endx),
		.bm_endy          (bm_endy),
		.bm_mode          (bm_mode),
		.bm_subpal        (bm_subpal),
		.bm_latch_en      (bm_latch_en),
		.bm_latch_thrs    (bm_latch_thrs),
		.bg0_tsz          (bg0_tsz),
		.bg1_tsz          (bg1_tsz),
		.bg0_8bpp         (bg0_8bpp),
		.bg_map_size      (bg_map_size),
		.bg_map_share     (bg_map_share),
		.bg_scroll        (bg_scroll),
		.bg_subpal        (bg_subpal),
		.obj_split        (obj_split),
		.obj_th0          (obj_th0),
		.obj_th1          (obj_th1),
		.obj_8bpp         (obj_8bpp),
		.obj_subpal       (obj_subpal),
		.char_split       (char_split),
		.blend_mode       (blend_mode),
		.layer_ctrl       (layer_ctrl),
		.prio             (prio),
		.screen_b_col     (screen_b_col),
		.screen_b_en      (screen_b_en),
		.screen_a_en      (screen_a_en),
		.blend_sub        (blend_sub),
		.backdrop_a       (backdrop_a),
		.backdrop_b       (backdrop_b),
		.cap_mode         (cap_mode),
		.cap_line         (cap_line),
		.cap_unk          (cap_unk),
		.irq_ctrl         (irq_ctrl),
		.irq0_hcmp        (irq0_hcmp),
		.irq0_vcmp        (irq0_vcmp),
		.bm_fast          (bm_fast),
		.bm_mem_unk       (bm_mem_unk),
		.fill_mask        (fill_mask),
		.fill_mask_unk    (fill_mask_unk),
		.fill_value       (fill_value),
		.fill_trig        (fill_trig),
		.fill_row         (fill_row),
		.sync_hcal        (sync_hcal),
		.sync_vcal        (sync_vcal),
		.dbg_raster_halt  (dbg_raster_halt),
		.dbg_render_dis   (dbg_render_dis),
		.dbg_raster_reset (dbg_raster_reset),
		.dbg_buf_hold     (dbg_buf_hold)
	);

	// INTERRUPT_CTRL spreads each enable over several bits, all of which must
	// be set.
	wire nmi_en  = irq_ctrl[2];
	wire irq0_en = irq_ctrl[7] & irq_ctrl[1];
	wire irq2_en = irq_ctrl[6] & irq_ctrl[3] & irq_ctrl[0];
	wire irq0_vcmp_en = irq_ctrl[5];
	wire irq2_src     = irq_ctrl[4];    // 0 capture ready, 1 ADC ready

	// ---- raster --------------------------------------------------------------

	wire [10:0] hcyc;
	wire [9:0]  vline;
	wire        ce_pix_1, ce_pix_half;
	wire [2:0]  phase;
	wire        r_hsync, r_vsync, r_hvisible, r_vvisible, r_render, r_border;
	wire        r_hrender, r_vrender;
	wire        r_fetch;
	wire        line_start, frame_start;
	wire [63:0] ss_raster;

	vdp_raster u_raster (
		.clk          (clk),
		.reset        (reset),
		.ce_vdp       (ce_vdp),
		.ss_clk       (ss_clk),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_raster),
		.ss_ready     (ss_ready),
		.mode_240     (mode_vidh),
		.hcal         (sync_hcal),
		.vcal         (sync_vcal),
		.nmi_en       (nmi_en),
		.irq0_en      (irq0_en),
		.irq0_vcmp_en (irq0_vcmp_en),
		.irq0_hcmp    (irq0_hcmp),
		.irq0_vcmp    (irq0_vcmp),
		.rdma_en      (rdma_en),
		.rdma_line    (rdma_tm),
		.hcyc         (hcyc),
		.vline        (vline),
		.hcount       (hcount),
		.vcount       (vcount),
		.fetch_y      (fetch_y),
		.ce_pix       (ce_pix_1),
		.ce_pix_half  (ce_pix_half),
		.phase        (phase),
		.hsync        (r_hsync),
		.vsync        (r_vsync),
		.hvisible     (r_hvisible),
		.vvisible     (r_vvisible),
		.hrender      (r_hrender),
		.vrender      (r_vrender),
		.render       (r_render),
		.fetch_win    (r_fetch),
		.border       (r_border),
		.line_start   (line_start),
		.frame_start  (frame_start),
		.nmi_n        (nmi_n),
		.irq0_n       (irq0_n),
		.raster_dma   (raster_dma)
	);

	// Half way across the picture, for the ADC's fastest automatic rate.
	wire half_line = ce_pix_1 & (hcount == 9'd128);

	wire [7:0] disp_x = hcount[7:0];
	wire [8:0] fill_y = vcount + 9'd1;

	// Column the background fetch is aimed at, registered like fetch_y because
	// it starts a long run into the tile memory. Loaded in the last phase of a
	// pixel, where HCOUNT still reads that pixel, so it looks two ahead.
	reg [7:0] fetch_x;
	always @(posedge clk) begin
		if (reset)                fetch_x <= 8'd0;
		else if (phase == 3'd7)   fetch_x <= hcount[7:0] + 8'd2;
	end

	// ---- memories ------------------------------------------------------------

	wire [13:0] bm_rd_addr;
	wire [63:0] bm_rd_data;
	wire [15:0] bm_cpu_rdata;

	// ---- savestate bulk walk -------------------------------------------------

	`include "ss_map.svh"

	// Two cycles: apply the address, then take what the RAM answers.
	reg ss_mem_apply;
	always @(posedge clk) begin
		ss_mem_ack <= 1'b0;
		if (ss_mem_reset) begin
			ss_mem_apply <= 1'b0;
		end else if (!ss_mem_apply) begin
			ss_mem_apply <= ss_mem_valid & ~ss_mem_ack;
		end else begin
			ss_mem_apply <= 1'b0;
			ss_mem_ack   <= 1'b1;
		end
	end

	wire ss_bm   = ss_mem_apply && (ss_mem_type == SS_MEM_VRAM);
	wire ss_tile = ss_mem_apply && (ss_mem_type == SS_MEM_TILERAM);
	wire ss_oam  = ss_mem_apply && (ss_mem_type == SS_MEM_OAM);
	wire ss_pal  = ss_mem_apply && (ss_mem_type == SS_MEM_PALETTE);

	wire [7:0] ss_bm_q, ss_tile_q;

	// OAM is 32 bits big-endian, the palette 16, so both byte enables count
	// down from the top the same way the bus does.
	wire [3:0] ss_oam_be = 4'b1000 >> ss_mem_addr[1:0];
	wire [1:0] ss_pal_be = ss_mem_addr[0] ? 2'b01 : 2'b10;
	reg  [1:0] ss_oam_sel_q;
	reg        ss_pal_sel_q;
	always @(posedge clk) begin
		ss_oam_sel_q <= ss_mem_addr[1:0];
		ss_pal_sel_q <= ss_mem_addr[0];
	end

	vdp_bm_vram u_bmvram (
		.clk        (clk),
		.reset      (reset),
		.cpu_sel    (sel_bitmap),
		.cpu_wr     (cpu_wr),
		.cpu_addr   (cpu_addr[16:1]),
		.cpu_be     (cpu_be),
		.cpu_wdata  (cpu_wdata),
		.cpu_rdata  (bm_cpu_rdata),
		.fill_trig  (fill_trig),
		.fill_row   (fill_row),
		.fill_mask  (fill_mask),
		.fill_value (fill_value),
		.fill_busy  (fill_busy),
		.rd_addr    (bm_rd_addr),
		.rd_data    (bm_rd_data),
		.ss_sel     (ss_bm),
		.ss_addr    (ss_mem_addr),
		.ss_we      (ss_bm & ss_mem_we),
		.ss_wdata   (ss_mem_wdata),
		.ss_rdata   (ss_bm_q)
	);

	wire [12:0] bg0_map_addr, bg0_chr_addr, bg1_map_addr, bg1_chr_addr;
	wire [12:0] obj_chr_addr;
	wire [63:0] tile_rd_data;
	wire [15:0] tile_cpu_rdata;

	vdp_tile_vram u_tilevram (
		.clk          (clk),
		.reset        (reset),
		.cpu_sel      (sel_tile),
		.cpu_wr       (cpu_wr),
		.cpu_rd       (cpu_rd),
		.cpu_addr     (cpu_addr[15:1]),
		.cpu_wdata    (cpu_wdata),
		.cpu_rdata    (tile_cpu_rdata),
		.phase        (phase),
		.in_window    (r_fetch),
		.bg0_map_addr (bg0_map_addr),
		.bg0_chr_addr (bg0_chr_addr),
		.bg1_map_addr (bg1_map_addr),
		.bg1_chr_addr (bg1_chr_addr),
		.obj_addr     (obj_chr_addr),
		.rd_data      (tile_rd_data),
		.ss_sel       (ss_tile),
		.ss_addr      (ss_mem_addr[15:0]),
		.ss_we        (ss_tile & ss_mem_we),
		.ss_wdata     (ss_mem_wdata),
		.ss_rdata     (ss_tile_q)
	);

	// OAM is 128 entries of 32 bits and the CPU sees two halfwords of each, so
	// its byte lanes are placed by address bit 1.
	wire [6:0]  oam_rd_addr;
	wire [31:0] oam_rd_data, oam_rd_raw;
	wire [31:0] oam_cpu_q;
	wire [3:0]  oam_be = cpu_addr[1] ? {2'b00, cpu_be} : {cpu_be, 2'b00};
	wire [15:0] oam_cpu_rdata = cpu_addr[1] ? oam_cpu_q[15:0] : oam_cpu_q[31:16];

	wire [6:0]  oam_a_addr  = ss_oam ? ss_mem_addr[8:2] : cpu_addr[8:2];
	wire        oam_a_wren  = ss_oam ? ss_mem_we : (sel_oam & cpu_wr);
	wire [3:0]  oam_a_be    = ss_oam ? ss_oam_be : oam_be;
	wire [31:0] oam_a_wdata = ss_oam ? {4{ss_mem_wdata}} : {2{cpu_wdata}};

	cache_ram_dp_be #(.ADDR_WIDTH (7), .DATA_WIDTH (32)) u_oam (
		.clk_i     (clk),
		.addr_a_i  (oam_a_addr),
		.wren_a_i  (oam_a_wren),
		.be_a_i    (oam_a_be),
		.wdata_a_i (oam_a_wdata),
		.q_a_o     (oam_cpu_q),
		.addr_b_i  (oam_rd_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (4'd0),
		.wdata_b_i (32'd0),
		.q_b_o     (oam_rd_raw)
	);

	// A CPU write to the entry the object walk is reading would otherwise
	// return undefined data from the M10K.
	vdp_rdw_fwd #(.ADDR_WIDTH (7), .BYTES (4)) u_oam_fwd (
		.clk     (clk),
		.reset   (reset),
		.wr_addr (oam_a_addr),
		.wr_en   (oam_a_wren),
		.wr_be   (oam_a_be),
		.wr_data (oam_a_wdata),
		.rd_addr (oam_rd_addr),
		.rd_raw  (oam_rd_raw),
		.rd_data (oam_rd_data)
	);

	// The palette is duplicated so both screens can look up at once. The CPU
	// writes both copies and reads the first.
	wire [7:0]  idx_a, idx_b;
	// Palette entries are RGB555 in a 16-bit word; bit 15 is dropped.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [15:0] pal_q_a, pal_q_b;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [15:0] pal_cpu_q;
	wire [15:0] pal_raw_a, pal_raw_b;

	wire [7:0]  pal_a_addr  = ss_pal ? ss_mem_addr[8:1] : cpu_addr[8:1];
	wire        pal_a_wren  = ss_pal ? ss_mem_we : (sel_pal & cpu_wr);
	wire [1:0]  pal_a_be    = ss_pal ? ss_pal_be : 2'b11;
	wire [15:0] pal_a_wdata = ss_pal ? {2{ss_mem_wdata}} : cpu_wdata;

	cache_ram_dp_be #(.ADDR_WIDTH (8), .DATA_WIDTH (16)) u_pal_a (
		.clk_i     (clk),
		.addr_a_i  (pal_a_addr),
		.wren_a_i  (pal_a_wren),
		.be_a_i    (pal_a_be),
		.wdata_a_i (pal_a_wdata),
		.q_a_o     (pal_cpu_q),
		.addr_b_i  (idx_a),
		.wren_b_i  (1'b0),
		.be_b_i    (2'd0),
		.wdata_b_i (16'd0),
		.q_b_o     (pal_raw_a)
	);

	cache_ram_dp_be #(.ADDR_WIDTH (8), .DATA_WIDTH (16)) u_pal_b (
		.clk_i     (clk),
		.addr_a_i  (pal_a_addr),
		.wren_a_i  (pal_a_wren),
		.be_a_i    (pal_a_be),
		.wdata_a_i (pal_a_wdata),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  (idx_b),
		.wren_b_i  (1'b0),
		.be_b_i    (2'd0),
		.wdata_b_i (16'd0),
		.q_b_o     (pal_raw_b)
	);

	// Raster effects write the palette mid-screen, so a write and a pixel's
	// lookup often collide. Both copies take the new colour on that pixel.
	vdp_rdw_fwd #(.ADDR_WIDTH (8), .BYTES (2)) u_pal_a_fwd (
		.clk     (clk),
		.reset   (reset),
		.wr_addr (pal_a_addr),
		.wr_en   (pal_a_wren),
		.wr_be   (pal_a_be),
		.wr_data (pal_a_wdata),
		.rd_addr (idx_a),
		.rd_raw  (pal_raw_a),
		.rd_data (pal_q_a)
	);

	vdp_rdw_fwd #(.ADDR_WIDTH (8), .BYTES (2)) u_pal_b_fwd (
		.clk     (clk),
		.reset   (reset),
		.wr_addr (pal_a_addr),
		.wr_en   (pal_a_wren),
		.wr_be   (pal_a_be),
		.wr_data (pal_a_wdata),
		.rd_addr (idx_b),
		.rd_raw  (pal_raw_b),
		.rd_data (pal_q_b)
	);

	// What the walk reads back, one cycle after the address went out.
	always @* begin
		case (ss_mem_type)
		SS_MEM_VRAM:    ss_mem_rdata = ss_bm_q;
		SS_MEM_TILERAM: ss_mem_rdata = ss_tile_q;
		SS_MEM_OAM:     ss_mem_rdata =
			(ss_oam_sel_q == 2'd0) ? oam_cpu_q[31:24] :
			(ss_oam_sel_q == 2'd1) ? oam_cpu_q[23:16] :
			(ss_oam_sel_q == 2'd2) ? oam_cpu_q[15:8]  : oam_cpu_q[7:0];
		SS_MEM_PALETTE: ss_mem_rdata = ss_pal_sel_q ? pal_cpu_q[7:0] : pal_cpu_q[15:8];
		default:        ss_mem_rdata = 8'd0;
		endcase
	end

	wire [7:0]  cap_wr_addr;
	wire        cap_wr_en;
	wire [1:0]  cap_wr_be;
	wire [15:0] cap_wr_data, cap_cpu_q, cap_cpu_raw;

	cache_ram_dp_be #(.ADDR_WIDTH (8), .DATA_WIDTH (16)) u_capbuf (
		.clk_i     (clk),
		.addr_a_i  (cpu_addr[8:1]),
		.wren_a_i  (1'b0),
		.be_a_i    (2'd0),
		.wdata_a_i (16'd0),
		.q_a_o     (cap_cpu_raw),
		.addr_b_i  (cap_wr_addr),
		.wren_b_i  (cap_wr_en),
		.be_b_i    (cap_wr_be),
		.wdata_b_i (cap_wr_data),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_b_o     ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	// The capture is the writer and the CPU the reader. A read while a line is
	// still landing gets the word just captured.
	vdp_rdw_fwd #(.ADDR_WIDTH (8), .BYTES (2)) u_cap_fwd (
		.clk     (clk),
		.reset   (reset),
		.wr_addr (cap_wr_addr),
		.wr_en   (cap_wr_en),
		.wr_be   (cap_wr_be),
		.wr_data (cap_wr_data),
		.rd_addr (cpu_addr[8:1]),
		.rd_raw  (cap_cpu_raw),
		.rd_data (cap_cpu_q)
	);

	// ---- layers --------------------------------------------------------------

	wire [7:0] bm_pix [0:3];
	wire [63:0] ss_bitmap;

	vdp_bitmap u_bitmap (
		.clk        (clk),
		.reset      (reset),
		.ss_clk     (ss_clk),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_bitmap),
		.line_start (line_start),
		.fill_y     (fill_y),
		.disp_y     (vcount),
		.disp_x     (disp_x),
		.phase      (phase),
		.bm_mode    (bm_mode),
		.bm_subpal  (bm_subpal),
		.scrollx    (bm_scrollx),
		.scrolly    (bm_scrolly),
		.posx       (bm_posx),
		.posy       (bm_posy),
		.startx     (bm_startx),
		.endx       (bm_endx),
		.endy       (bm_endy),
		.latch_en   (bm_latch_en),
		.latch_thrs (bm_latch_thrs),
		.vram_addr  (bm_rd_addr),
		.vram_data  (bm_rd_data),
		.pix0       (bm_pix[0]),
		.pix1       (bm_pix[1]),
		.pix2       (bm_pix[2]),
		.pix3       (bm_pix[3])
	);

	wire [7:0] bg_pix0, bg_pix1;
	wire       bg_scrn0, bg_scrn1;
	wire [15:0] char_base;

	vdp_bg u_bg (
		.clk        (clk),
		.reset      (reset),
		.phase      (phase),
		.fetch_x    (fetch_x),
		.fetch_y    (fetch_y),
		.bg0_tsz    (bg0_tsz),
		.bg1_tsz    (bg1_tsz),
		.bg0_8bpp   (bg0_8bpp),
		.map_size   (bg_map_size),
		.map_share  (bg_map_share),
		.scroll     (bg_scroll),
		.subpal     (bg_subpal),
		.char_split (char_split),
		.map0_addr  (bg0_map_addr),
		.chr0_addr  (bg0_chr_addr),
		.map1_addr  (bg1_map_addr),
		.chr1_addr  (bg1_chr_addr),
		.rd_data    (tile_rd_data),
		.pix0       (bg_pix0),
		.scrn0      (bg_scrn0),
		.pix1       (bg_pix1),
		.scrn1      (bg_scrn1),
		.char_base  (char_base)
	);

	wire [7:0] obj_pix0, obj_pix1;
	wire [63:0] ss_obj;

	vdp_obj u_obj (
		.clk        (clk),
		.reset      (reset),
		.ss_clk     (ss_clk),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_obj),
		.line_start (line_start),
		.fill_y     (fill_y),
		.disp_x     (disp_x),
		.phase      (phase),
		.in_window  (r_fetch),
		.buf_hold   (dbg_buf_hold),
		.obj_split  (obj_split),
		.obj_th0    (obj_th0),
		.obj_th1    (obj_th1),
		.obj_8bpp   (obj_8bpp),
		.obj_subpal (obj_subpal),
		.char_split (char_split),
		.data_base  (char_base),
		.oam_addr   (oam_rd_addr),
		.oam_data   (oam_rd_data),
		.chr_addr   (obj_chr_addr),
		.chr_data   (tile_rd_data),
		.pix0       (obj_pix0),
		.pix1       (obj_pix1)
	);

	// ---- compositor ----------------------------------------------------------

	vdp_priority u_prio (
		.clk        (clk),
		.reset      (reset),
		.phase      (phase),
		.prio       (prio),
		.layer_ctrl (layer_ctrl),
		.bm0        (bm_pix[0]),
		.bm1        (bm_pix[1]),
		.bm2        (bm_pix[2]),
		.bm3        (bm_pix[3]),
		.bg0        (bg_pix0),
		.bg1        (bg_pix1),
		.bg0_scrn   (bg_scrn0),
		.bg1_scrn   (bg_scrn1),
		.obj0       (obj_pix0),
		.obj1       (obj_pix1),
		.idx_a      (idx_a),
		.idx_b      (idx_b)
	);

	wire [14:0] screen_a_col, blended_col;

	vdp_blend u_blend (
		.clk          (clk),
		.reset        (reset),
		.phase        (phase),
		.half         (phase[2]),
		.idx_a        (idx_a),
		.idx_b        (idx_b),
		.col_a        (pal_q_a[14:0]),
		.col_b        (pal_q_b[14:0]),
		.backdrop_a   (backdrop_a),
		.backdrop_b   (backdrop_b),
		.blend_mode   (blend_mode),
		.blend_sub    (blend_sub),
		.screen_a_en  (screen_a_en),
		.screen_b_en  (screen_b_en),
		.screen_b_col (screen_b_col),
		.in_render    (r_render),
		.render_dis   (dbg_render_dis),
		.color        (rgb),
		.screen_a     (screen_a_col),
		.blended      (blended_col)
	);

	wire        cap_ready;
	wire [63:0] ss_capture;

	vdp_capture u_capture (
		.clk        (clk),
		.reset      (reset),
		.ss_clk     (ss_clk),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_capture),
		.trig_cap   (trig_cap),
		.cap_mode   (cap_mode),
		.cap_line   (cap_line),
		.vcount     (vcount),
		.disp_x     (disp_x),
		.phase      (phase),
		.in_render  (r_render),
		.line_start (line_start),
		.idx_a      (idx_a),
		.screen_a   (screen_a_col),
		.blended    (blended_col),
		.wr_addr    (cap_wr_addr),
		.wr_en      (cap_wr_en),
		.wr_be      (cap_wr_be),
		.wr_data    (cap_wr_data),
		.ready      (cap_ready)
	);

	// ---- IO ------------------------------------------------------------------

	wire [15:0] ioc_rdata, iop_rdata, ioe_rdata;
	wire        ioc_hit, iop_hit, ioe_hit;
	wire [63:0] ss_ioctrl, ss_ioprint, ss_ioexp;
	wire        adc_ready;

	vdp_io_ctrl u_ioctrl (
		.clk        (clk),
		.reset      (reset),
		.ss_clk     (ss_clk),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_ioctrl),
		.addr       (cpu_addr[11:1]),
		.wr         (cpu_wr & sel_io),
		.rd         (cpu_rd & sel_io),
		.be         (cpu_be),
		.wdata      (cpu_wdata),
		.rdata      (ioc_rdata),
		.hit        (ioc_hit),
		.mode_cmode (mode_cmode),
		.mode_mcnt  (mode_mcnt),
		.line_start (line_start),
		.ce_pix     (ce_pix_1),
		.vcount     (vcount),
		.ctrl_out   (ctrl_out),
		.ctrl_in    (ctrl_in)
	);

	vdp_io_print u_ioprint (
		.clk         (clk),
		.reset       (reset),
		.ss_clk      (ss_clk),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_ioprint),
		.addr        (cpu_addr[11:1]),
		.wr          (cpu_wr & sel_io),
		.be          (cpu_be),
		.wdata       (cpu_wdata),
		.rdata       (iop_rdata),
		.hit         (iop_hit),
		.trig_psen   (trig_psen),
		.trig_adc    (trig_adc),
		.line_start  (line_start),
		.half_line   (half_line),
		.ce_vdp      (ce_vdp),
		.sct         (print_sct),
		.opto        (print_opto),
		.region_ntsc (region_ntsc),
		.motor_phase (print_motor),
		.head_ctrl   (print_head),
		.head_data   (print_data),
		.head_data_wr (print_data_wr),
		.printer_enable (print_enable),
		.adc_ready   (adc_ready)
	);

	vdp_io_exp u_ioexp (
		.clk          (clk),
		.reset        (reset),
		.ss_clk       (ss_clk),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_ioexp),
		.addr         (cpu_addr[11:1]),
		.wr           (cpu_wr & sel_io),
		.be           (cpu_be),
		.wdata        (cpu_wdata),
		.rdata        (ioe_rdata),
		.hit          (ioe_hit),
		.sel_sound    (sel_sound),
		.sel_exp      (sel_exp),
		.cpu_wr       (cpu_wr),
		.cpu_busy     (cpu_exp_busy),
		.cpu_wdata    (cpu_wdata),
		.sound_ctrl   (sound_ctrl),
		.exp_strobe_n (exp_strobe_n),
		.exp_fast     (exp_fast)
	);

	// ---- read mux ------------------------------------------------------------
	// Memories answer one cycle after the strobe, registers straight away;
	// both settle inside the shortest WAIT, so one mux on the held selects
	// serves for either.

	// The IO blocks only decode the low twelve bits of an address, so their
	// answers are qualified by the page select before being wire-ORed.
	wire [15:0] io_rdata = sel_io ? (ioc_rdata | iop_rdata | ioe_rdata) : 16'd0;
	wire        io_hit   = sel_io & (ioc_hit | iop_hit | ioe_hit);

	assign bus_rdata = sel_bitmap ? bm_cpu_rdata
	                 : sel_tile   ? tile_cpu_rdata
	                 : sel_oam    ? oam_cpu_rdata
	                 : sel_pal    ? pal_cpu_q
	                 : sel_cap    ? cap_cpu_q
	                 : ((sel_reg ? reg_rdata : 16'd0) | io_rdata);

	assign bus_hit = sel_bitmap | sel_tile | sel_oam | sel_pal | sel_cap
	               | (sel_reg & reg_hit) | io_hit;

	// ---- IRQ2 ----------------------------------------------------------------
	// One source at a time, and the same 16-VDP-cycle low pulse every VDP
	// interrupt uses.

	wire irq2_hit = irq2_en & (irq2_src ? adc_ready : cap_ready);
	reg [4:0] irq2_cnt;
	always @(posedge clk) begin
		if (reset)              irq2_cnt <= 5'd0;
		else if (irq2_hit)      irq2_cnt <= 5'd16;
		else if (ce_vdp && irq2_cnt != 5'd0) irq2_cnt <= irq2_cnt - 5'd1;
	end
	assign irq2_n = ~(irq2_cnt != 5'd0);

	// ---- video output --------------------------------------------------------
	// The colour is a pixel behind the raster, so the position signals are
	// delayed to match.

	assign ce_pix = (blend_mode == 3'd3) ? ce_pix_half : ce_pix_1;

	reg d_hsync, d_vsync, d_hvis, d_vvis, d_hrend, d_vrend, d_render;
	always @(posedge clk) begin
		if (reset) begin
			d_hsync <= 1'b0; d_vsync <= 1'b0;
			d_hvis  <= 1'b0; d_vvis  <= 1'b0; d_render <= 1'b0;
			d_hrend <= 1'b0; d_vrend <= 1'b0;
		end else if (ce_pix_1) begin
			d_hsync  <= r_hsync;
			d_vsync  <= r_vsync;
			d_hvis   <= r_hvisible;
			d_vvis   <= r_vvisible;
			d_hrend  <= r_hrender;
			d_vrend  <= r_vrender;
			d_render <= r_render;
		end
	end

	assign hsync    = d_hsync;
	assign vsync    = d_vsync;
	assign hvisible = d_hvis;
	assign vvisible = d_vvis;
	assign hrender  = d_hrend;
	assign vrender  = d_vrend;
	assign render   = d_render;

	// ---- savestate wire-OR ---------------------------------------------------

	assign ss_dout = ss_regs | ss_raster | ss_bitmap | ss_obj | ss_capture
	               | ss_ioctrl | ss_ioprint | ss_ioexp;

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	// Stored but unused: the MODE, memory-control, flash-mask and capture
	// unknowns, two raster debug bits, and the raster's own counters.
	wire _unused = &{1'b0, mode_unk1, mode_unk2, bm_mem_unk, fill_mask_unk,
	                 cap_unk, dbg_raster_halt, dbg_raster_reset, hcyc, vline,
	                 frame_start, r_border};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

endmodule
