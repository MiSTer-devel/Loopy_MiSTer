// Copyright (c) 2026 Jamie Blanks

// The JCM631-MA1M board: every chip instantiated once and wired by the name
// the schematic uses.
//
//   LSI302  SH7021      CPU, 32 KB mask ROM (the BIOS), 1 KB RAM
//   LSI301  HM514260    512 KB work DRAM on the CPU's own DRAM strobes
//   LSI51   RH-7500     VDP, controller port, printer IO, ADC, expansion
//           cart slot   90-pin connector: ROM on /CS6, SRAM on /CS2
//
// Port A carries most of the bus control. The BIOS sets PACR1:PACR2 to
// 0x0C02:0xBF99, which is the arrangement the board is wired for:
//
//   PA0  /CS4  -> RH-7500          PA8  DET, high with a cartridge seated
//   PA1  /RAS  -> HM514260         PA9  free GPIO to the cartridge
//   PA2  /CS6  -> cartridge ROM    PA10 GPIO out, board net unknown
//   PA3  /WAIT <- RH-7500          PA11 GPIO in, board net unknown
//   PA4  /WRL                      PA12 /IRQ0 <- RH-7500
//   PA5  /WRH                      PA13 DREQ0 <- RH-7500 raster DMA
//   PA6  /RD                       PA14 /IRQ2 <- RH-7500
//   PA7  free GPIO to the cart     PA15 free GPIO to the cartridge
//
// Dedicated pins: /CS2 to the cartridge SRAM, CS1/CASH and CS3/CASL as CASH
// and CASL (CASCR = 0xAFFF), WDTOVF to the cartridge's /PRST.
//
// Until the BIOS writes PACR the bus pins are GPIO inputs pulled high, the
// inactive level for every strobe; the `pin` helper models that. The RH-7500
// has its own crystal, so its /WAIT and interrupt lines come in through two
// flops. `ext_mem_bridge` takes `reset_mem_*` from a cold start only, since
// content downloads hold the machine in reset while writing through it.

module loopy_mainboard (
	input  wire        clk_sys,
	input  wire        reset_sys,
	input  wire        ce_cpu_r,        // one per SH7021 state, already frozen
	input  wire        ce_cpu_f,        // by mem_stall and the savestate pause
	input  wire        ce_4m,           // RH-7501 X201, the MIDI receiver
	input  wire        ce_sample,       // X202 / 256, one output sample
	input  wire        clk_video,
	input  wire        reset_video,
	input  wire        ce_vdp,
	input  wire        clk_ram,
	// The memory path keeps working while the console is held in reset,
	// because that is when content arrives.
	input  wire        reset_mem_sys,
	input  wire        reset_mem_ram,
	// The savestate transfer path's own resets, one per domain, driven only
	// by a cold start: a restore holds the machine in reset for the transfer.
	input  wire        reset_ss_sys,
	input  wire        reset_ss_video,

	// The BIOS image, into the SH7021's mask ROM.
	input  wire        rom_wr_en,
	input  wire [12:0] rom_wr_addr,
	input  wire [31:0] rom_wr_data,

	// What the loader worked out about the cartridge.
	input  wire        cart_det,
	input  wire [21:1] cart_rom_mask,
	input  wire        cart_rom_split,
	input  wire [20:1] cart_rom_hi_mask,
	input  wire [16:0] cart_sram_mask,

	// The loader's own path into SDRAM. The sound section watches it go past.
	input  wire        loading,
	input  wire        ld_req,
	input  wire [25:0] ld_addr,
	input  wire [15:0] ld_din,
	output wire        ld_busy,

	// SDRAM controller.
	output wire        p0_req,
	output wire        p0_we,
	output wire [25:0] p0_addr,
	output wire [63:0] p0_din,
	output wire [7:0]  p0_byte_en,
	input  wire [63:0] p0_dout,
	input  wire        p0_busy,
	input  wire        p0_ready,
	output wire        p1_req,
	output wire [25:0] p1_addr,
	input  wire [63:0] p1_dout,
	input  wire        p1_busy,
	input  wire        p1_ready,
	output wire        p2_req,
	output wire [25:0] p2_addr,
	input  wire [63:0] p2_dout,
	input  wire        p2_busy,
	input  wire        p2_ready,

	// Back to loopy_clocks: hold the CPU while a memory model is waiting.
	output wire        mem_stall,

	// Video, straight out of the RH-7500.
	output wire        ce_pix,
	output wire [14:0] rgb,
	output wire        hsync,
	output wire        vsync,
	output wire        hvisible,
	output wire        vvisible,
	output wire        hrender,
	output wire        vrender,
	output wire        render,

	// The seal printer: the mechanism is only fitted when the menu says so,
	// and the capture buffer is read from the video overlay at the top.
	input  wire        print_cassette,
	output wire        print_clear,
	output wire        print_cap_wr,
	output wire [6:0]  print_cap_y,
	output wire [6:0]  print_cap_x,
	output wire [1:0]  print_cap_pass,
	output wire [2:0]  print_cap_ink,
	output wire        print_show,

	// Controller port and sound control.
	output wire [5:0]  ctrl_out,
	input  wire [7:0]  ctrl_in,
	output wire [11:0] sound_ctrl,

	// The AV connector's audio, the synth and the cartridge expansion mixed.
	output wire signed [15:0] aout_l,
	output wire signed [15:0] aout_r,
	output wire        snd_idle,        // at a sample boundary, nothing in flight

	// Cartridge SRAM's second port: the save slot and the savestate bulk stream.
	input  wire [13:0] sram_b_addr,
	input  wire        sram_b_wren,
	input  wire [1:0]  sram_b_be,
	input  wire [15:0] sram_b_wdata,
	output wire [15:0] sram_b_q,
	output wire        sram_dirty,

	// Savestate scalar bus and the work-DRAM flush.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,
	input  wire        ss_flush,
	output wire        ss_idle,

	// Savestate bulk walk, one byte per request.
	input  wire [24:0] ss_ram_addr,
	input  wire [2:0]  ss_ram_type,
	input  wire        ss_ram_rd,
	input  wire        ss_ram_wr,
	input  wire [7:0]  ss_ram_wdata,
	output wire [7:0]  ss_ram_rdata,
	output wire        ss_ram_ready,

	// Where the machine may be stopped, and the line cache invalidate a
	// restore needs because it wrote SDRAM behind the cache.
	output wire        ss_cpu_ready,
	output wire        ss_vdp_ready,    // clk_video
	input  wire        ss_cache_rst
);

	// --------------------------------------------------------- the SH7021
	wire [21:0] a;
	wire [15:0] cpu_d_o, cpu_d_i;
	wire        cpu_d_oe;
	wire        cs0_n, cs2_n, cash_n, casl_n, wdtovf_n;
	wire [15:0] pa_o, pa_oe, pb_o, pb_oe;
	reg  [15:0] pa_i, pb_i;
	wire [63:0] ss_dout_cpu, ss_dout_vdp;
	wire signed [11:0] exp_aout;
	wire        exp_aout_en;


	sh7021 u_cpu (
		.clk_i         (clk_sys),
		.ce_i          (ce_cpu_r),
		.ceh_i         (ce_cpu_f),
		.res_n_i       (~reset_sys),
		// MD 010: on-chip ROM enabled, area 0 is that ROM.
		.md_i          (3'b010),
		.nmi_i         (nmi_n_s),
		.a_o           (a),
		.ad_i          (cpu_d_i),
		.ad_o          (cpu_d_o),
		.ad_oe_o       (cpu_d_oe),
		.cs0_n_o       (cs0_n),
		.cs2_n_o       (cs2_n),
		.cs1_cash_n_o  (cash_n),
		.cs3_casl_n_o  (casl_n),
		.wdtovf_n_o    (wdtovf_n),
		.pa_i          (pa_i),
		.pa_o          (pa_o),
		.pa_oe_o       (pa_oe),
		.pb_i          (pb_i),
		.pb_o          (pb_o),
		.pb_oe_o       (pb_oe),
		.rom_wr_addr_i (rom_wr_addr),
		.rom_wr_en_i   (rom_wr_en),
		.rom_wr_data_i (rom_wr_data),
		.ss_din        (ss_din),
		.ss_addr       (ss_addr),
		.ss_wren       (ss_wren),
		.ss_rst        (ss_rst),
		.ss_dout       (ss_dout_cpu),
		.ss_ram_addr   (ssb_ram_addr),
		.ss_ram_we     (ssb_ram_we),
		.ss_ram_din    (ssb_ram_wdata),
		.ss_ram_dout   (ssb_ram_q),
		.ss_ready      (ss_cpu_ready)
	);

	// A bus pin the pin function controller has not handed to the bus is an
	// input, and the board pulls it up.
	function automatic logic pin(input [15:0] o, input [15:0] oe, input [3:0] n);
		pin = oe[n] ? o[n] : 1'b1;
	endfunction

	wire cs4_n  = pin(pa_o, pa_oe, 0);      // PA0, the RH-7500
	wire ras_n  = pin(pa_o, pa_oe, 1);      // PA1, the DRAM
	wire cs6_n  = pin(pa_o, pa_oe, 2);      // PA2, the cartridge ROM
	wire wrl_n  = pin(pa_o, pa_oe, 4);
	wire wrh_n  = pin(pa_o, pa_oe, 5);
	wire rd_n   = pin(pa_o, pa_oe, 6);

	// --------------------------------------------------------- the DRAM
	wire [15:0] dram_d;
	wire        dram_d_oe, dram_stall;
	wire        dram_req, dram_we;
	wire [18:3] dram_line;
	wire [63:0] dram_din, dram_dout;
	wire [7:0]  dram_be;
	wire        dram_busy, dram_done;

	// The board wires the chip's A8-A0 to the CPU's A9-A1, which is what makes
	// the nine-bit row shift the BIOS programs (DCR.MXC = 01) line up.
	hm514260 u_dram (
		.clk_i      (clk_sys),
		// A restore writes the work DRAM straight into SDRAM, behind the
		// cache, so the cache has to forget what it thinks it holds.
		.rst_i      (reset_sys | ss_cache_rst),
		.a_i        (a[9:1]),
		.ras_n_i    (ras_n),
		.cash_n_i   (cash_n),
		.casl_n_i   (casl_n),
		.we_n_i     (wrl_n),
		.oe_n_i     (rd_n),
		.d_i        (bus_d),
		.d_o        (dram_d),
		.d_oe_o     (dram_d_oe),
		.stall_o    (dram_stall),
		.ss_flush_i (ss_flush),
		.ss_idle_o  (ss_idle),
		.mem_req_o  (dram_req),
		.mem_we_o   (dram_we),
		.mem_line_o (dram_line),
		.mem_din_o  (dram_din),
		.mem_be_o   (dram_be),
		.mem_dout_i (dram_dout),
		.mem_busy_i (dram_busy),
		.mem_done_i (dram_done)
	);

	// -------------------------------------------------------- the RH-7500
	wire [15:0] vdp_d;
	wire        vdp_d_oe, vdp_wait_n;
	wire        vdp_nmi_n, vdp_irq0_n, vdp_irq2_n, vdp_raster_dma;
	wire [2:0]  exp_strobe_n;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [3:0]  exp_fast;                   // expansion fast strobes
	/* verilator lint_on UNUSEDSIGNAL */
	wire [3:0]  print_head;                 // PRINT_HEAD_CTRL, the row enable
	wire [3:0]  print_motor;
	wire [15:0] print_data;
	wire        print_data_wr;
	wire        print_enable;
	wire [2:0]  print_sct, print_opto;
	wire        print_step;

	rh7500 u_vdp (
		.clk          (clk_video),
		.reset        (reset_video),
		.ce_vdp       (ce_vdp),
		.ss_clk       (clk_sys),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ss_dout_vdp),
		.ss_mem_reset (reset_ss_video),
		.ss_mem_valid (ssv_valid),
		.ss_mem_type  (ssv_data[28:26]),
		.ss_mem_addr  (ssv_data[25:9]),
		.ss_mem_we    (ssv_data[8]),
		.ss_mem_wdata (ssv_data[7:0]),
		.ss_mem_ack   (ssv_ack),
		.ss_mem_rdata (ssv_resp),
		.ss_ready     (ss_vdp_ready),
		.cs_n         (cs4_n),
		.rd_n         (rd_n),
		.wrh_n        (wrh_n),
		.wrl_n        (wrl_n),
		.a            (a[19:0]),
		.d_i          (bus_d),
		.d_o          (vdp_d),
		.d_oe         (vdp_d_oe),
		.wait_n       (vdp_wait_n),
		.nmi_n        (vdp_nmi_n),
		.irq0_n       (vdp_irq0_n),
		.irq2_n       (vdp_irq2_n),
		.raster_dma   (vdp_raster_dma),
		.ctrl_out     (ctrl_out),
		.ctrl_in      (ctrl_in),
		// The mechanism answers the sensors; stock NTSC jumper.
		.print_sct    (print_sct),
		.print_opto   (print_opto),
		.region_ntsc  (1'b1),
		.print_motor  (print_motor),
		.print_head   (print_head),
		.print_data   (print_data),
		.print_data_wr (print_data_wr),
		.print_enable (print_enable),
		.exp_strobe_n (exp_strobe_n),
		.exp_fast     (exp_fast),
		.sound_ctrl   (sound_ctrl),
		.ce_pix       (ce_pix),
		.rgb          (rgb),
		.hsync        (hsync),
		.vsync        (vsync),
		.hvisible     (hvisible),
		.vvisible     (vvisible),
		.hrender      (hrender),
		.vrender      (vrender),
		.render       (render)
	);

	// ------------------------------------------------------- the seal printer

	wire [63:0] ss_dout_print, ss_dout_pcap;

	loopy_printer u_printer (
		.clk         (clk_video),
		.reset       (reset_video),
		.ce          (ce_vdp),
		.ss_clk      (clk_sys),
		.ss_din      (ss_din),
		.ss_addr     (ss_addr),
		.ss_wren     (ss_wren),
		.ss_rst      (ss_rst),
		.ss_dout     (ss_dout_print),
		.cassette    (print_cassette),
		.motor_phase (print_motor),
		.sct         (print_sct),
		.opto        (print_opto),
		.step_o      (print_step)
	);

	// The hold counts frames, so the capture wants one tick per vertical sync.
	reg vsync_q;
	always @(posedge clk_video) begin
		if (reset_video) vsync_q <= 1'b0;
		else             vsync_q <= vsync;
	end
	wire frame_tick = vsync & ~vsync_q;

	loopy_print_capture u_print_capture (
		.clk        (clk_video),
		.reset      (reset_video),
		.job_enable (print_enable),
		.ss_clk     (clk_sys),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ss_dout_pcap),
		.head_ctrl  (print_head),
		.data_wr    (print_data_wr),
		.data       (print_data),
		.frame_tick (frame_tick),
		.step_i     (print_step),
		.clear      (print_clear),
		.cap_wr     (print_cap_wr),
		.cap_y      (print_cap_y),
		.cap_x      (print_cap_x),
		.cap_pass   (print_cap_pass),
		.cap_ink    (print_cap_ink),
		.show       (print_show)
	);

	// The VDP runs from its own crystal, so its control lines to the CPU get
	// two flops. The data bus goes straight through: it is stable long before
	// /WAIT is released, and /WAIT is what the CPU waits for.
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] wait_sync, nmi_sync, irq0_sync, irq2_sync, dma_sync;
	always @(posedge clk_sys) begin
		wait_sync <= {wait_sync[0], vdp_wait_n};
		nmi_sync  <= {nmi_sync[0],  vdp_nmi_n};
		irq0_sync <= {irq0_sync[0], vdp_irq0_n};
		irq2_sync <= {irq2_sync[0], vdp_irq2_n};
		dma_sync  <= {dma_sync[0],  vdp_raster_dma};
	end

	wire nmi_n_s = nmi_sync[1];

	// ---------------------------------------------------- the cartridge
	wire [15:0] cart_d;
	wire        cart_d_oe, cart_stall;
	wire        cart_rom_req;
	wire [21:3] cart_rom_line;
	wire [63:0] cart_rom_dout;
	wire        cart_rom_busy, cart_rom_done;
	wire        pb7_nar, pb7_nar_oe;

	loopy_cart_slot u_cart (
		.clk_sys        (clk_sys),
		.rst            (reset_sys),
		.cart_det       (cart_det),
		.a              (a[21:0]),
		.d_in           (bus_d),
		.d_out          (cart_d),
		.d_oe           (cart_d_oe),
		.rom_n          (cs6_n),
		.sram_n         (cs2_n),
		.rd_n           (rd_n),
		.wrl_n          (wrl_n),
		.rom_mask       (cart_rom_mask),
		.rom_split      (cart_rom_split),
		.rom_hi_mask    (cart_rom_hi_mask),
		.sram_mask      (cart_sram_mask),
		.stall          (cart_stall),
		.sram_dirty     (sram_dirty),
		.rom_req        (cart_rom_req),
		.rom_line       (cart_rom_line),
		.rom_dout       (cart_rom_dout),
		.rom_busy       (cart_rom_busy),
		.rom_done       (cart_rom_done),
		.sram_b_addr    (ssb_sram_own ? ssb_sram_addr  : sram_b_addr),
		.sram_b_wren    (ssb_sram_own ? ssb_sram_we    : sram_b_wren),
		.sram_b_be      (ssb_sram_own ? ssb_sram_be    : sram_b_be),
		.sram_b_wdata   (ssb_sram_own ? ssb_sram_wdata : sram_b_wdata),
		.sram_b_q       (sram_b_q),
		.exp1_n         (exp_strobe_n[0]),
		.pb1_reset_n    (pb_o[1] & pb_oe[1]),
		.pb5_cmd        (pb_o[5] & pb_oe[5]),
		.pb8_ch         (pb_o[8] & pb_oe[8]),
		.pb10_st        (pb_o[10] & pb_oe[10]),
		.pb7_nar        (pb7_nar),
		.pb7_nar_oe     (pb7_nar_oe),
		.exp_aout       (exp_aout),
		.exp_aout_en    (exp_aout_en)
	);

	// --------------------------------------------------------- the buses
	// One data bus with pull-ups: whoever is driving wins, and nothing driving
	// reads as all ones.
	wire [15:0] bus_d = cpu_d_oe  ? cpu_d_o
	                  : vdp_d_oe  ? vdp_d
	                  : dram_d_oe ? dram_d
	                  : cart_d_oe ? cart_d : 16'hFFFF;

	assign cpu_d_i = vdp_d_oe  ? vdp_d
	               : dram_d_oe ? dram_d
	               : cart_d_oe ? cart_d : 16'hFFFF;

	// Port A inputs. Everything the CPU does not drive is pulled up, except
	// the four pins the VDP owns and DET.
	always @* begin
		pa_i     = 16'hFFFF;
		pa_i[3]  = wait_sync[1];       // /WAIT
		pa_i[8]  = cart_det;           // DET, tied to VCC in every cartridge
		pa_i[11] = 1'b1;               // board net unknown
		pa_i[12] = irq0_sync[1];       // /IRQ0
		// DREQ0 is low-active at the CPU and the VDP's raster signal is high
		// while the picture is being drawn, so the request falls in blanking.
		// Polarity is inferred, unmeasured.
		pa_i[13] = ~dma_sync[1];
		pa_i[14] = irq2_sync[1];       // /IRQ2
	end

	// Port B inputs: pull-ups, and PB7 when a cartridge drives it.
	always @* begin
		pb_i    = 16'hFFFF;
		pb_i[7] = pb7_nar_oe ? pb7_nar : 1'b1;
	end

	// ------------------------------------------------- memory and stalls
	assign mem_stall = dram_stall | cart_stall;

	// ---------------------------------------------------- the sound section
	//
	// Cartridge audio and the synth mix here, as at the AV connector on the
	// board. SOUND_CTRL arrives on the VDP clock and changes a few times a
	// frame at most, so a change detector feeds the handshake to cross it.
	wire [11:0] snd_ctl_video = sound_ctrl;

	reg  [11:0] snd_ctl_sent;
	reg  [11:0] snd_ctl_sys;
	wire        snd_cdc_busy, snd_dst_valid;
	wire [11:0] snd_dst_data;
	wire        snd_send = (snd_ctl_video != snd_ctl_sent) & ~snd_cdc_busy;

	always @(posedge clk_video) begin
		if (reset_video)   snd_ctl_sent <= 12'd0;
		else if (snd_send) snd_ctl_sent <= snd_ctl_video;
	end

	cdc_handshake #(.WIDTH (12), .RESP_WIDTH (1)) u_snd_cdc (
		.src_clk   (clk_video),
		.src_reset (reset_video),
		.src_req   (snd_send),
		.src_data  (snd_ctl_video),
		/* verilator lint_off PINCONNECTEMPTY */
		.src_busy  (snd_cdc_busy),
		.src_done  (),
		.src_resp  (),
		/* verilator lint_on PINCONNECTEMPTY */
		.dst_clk   (clk_sys),
		.dst_reset (reset_sys),
		.dst_valid (snd_dst_valid),
		.dst_data  (snd_dst_data),
		.dst_ack   (snd_dst_valid),
		.dst_resp  (1'b0)
	);

	always @(posedge clk_sys) begin
		if (reset_sys)          snd_ctl_sys <= 12'd0;
		else if (snd_dst_valid) snd_ctl_sys <= snd_dst_data;
	end

	// The wave ROM's half of the loader's stream, for the pitch table mirror:
	// the 512 KB at ext_mem_bridge's WAVE_BASE, 0x2000000.
	wire wave_ld_wr = ld_req & (ld_addr[25:19] == 7'h40);

	wire        wave_req, wave_busy, wave_done;
	wire [18:3] wave_line;
	wire [63:0] wave_dout;
	wire signed [15:0] synth_l, synth_r;
	wire [63:0] ss_dout_snd;

	loopy_sound u_sound (
		.clk_i             (clk_sys),
		.rst_i             (reset_sys),
		.ce_4m_i           (ce_4m),
		.ce_sample_i       (ce_sample),
		.snd_ctl_i         (snd_ctl_sys),
		// SCI1's TxD is PB11 once the pin function controller has selected it;
		// until then the board's pull-up holds the line in its idle mark state.
		.midi_rxd_i        (pin(pb_o, pb_oe, 11)),
		.wave_req_o        (wave_req),
		.wave_line_o       (wave_line),
		.wave_dout_i       (wave_dout),
		.wave_busy_i       (wave_busy),
		.wave_done_i       (wave_done),
		.ld_wr_i           (wave_ld_wr),
		.ld_addr_i         (ld_addr[18:0]),
		.ld_data_i         (ld_din),
		.aout_l_o          (synth_l),
		.aout_r_o          (synth_r),
		.idle_o            (snd_idle),
		.ss_din_i          (ss_din),
		.ss_addr_i         (ss_addr),
		.ss_wren_i         (ss_wren),
		.ss_rst_i          (ss_rst),
		.ss_dout_o         (ss_dout_snd),
		// Savestate walk into the synth's working RAM.
		.ssb_rst_i         (reset_ss_sys),
		.ssb_rd_i          (ssb_voice_rd),
		.ssb_wr_i          (ssb_voice_wr),
		.ssb_addr_i        (ssb_voice_addr),
		.ssb_wdata_i       (ssb_voice_wdata),
		/* verilator lint_off PINCONNECTEMPTY */
		.ssb_q_o           (ssb_voice_q),
		.ssb_ready_o       (ssb_voice_ready)
		/* verilator lint_on PINCONNECTEMPTY */
	);

	wire signed [15:0] motor_mix;
	loopy_printer_audio u_printer_audio (
		.clk         (clk_sys),
		.reset       (reset_sys),
		.ce_sample   (ce_sample),
		.enable      (print_enable & print_cassette),
		.motor_phase (print_motor),
		.sample      (motor_mix)
	);

	// The cartridge's analogue output joins the synth's after the amplifier,
	// at the AV connector. The relative level is a guess: a quarter of full
	// scale puts speech over music without burying it.
	localparam int EXP_SHIFT = 2;
	wire signed [15:0] exp_mix = exp_aout_en ? {{(4 - EXP_SHIFT){exp_aout[11]}},
	                                            exp_aout, {EXP_SHIFT{1'b0}}}
	                                         : 16'sd0;
	wire signed [16:0] mix_l = {synth_l[15], synth_l} + {exp_mix[15], exp_mix}
	                         + {motor_mix[15], motor_mix};
	wire signed [16:0] mix_r = {synth_r[15], synth_r} + {exp_mix[15], exp_mix}
	                         + {motor_mix[15], motor_mix};

	function automatic signed [15:0] sat16(input signed [16:0] v);
		if      (v >  17'sd32767) sat16 =  16'sd32767;
		else if (v < -17'sd32767) sat16 = -16'sd32767;
		else                      sat16 = v[15:0];
	endfunction

	assign aout_l = sat16(mix_l);
	assign aout_r = sat16(mix_r);

	ext_mem_bridge u_bridge (
		.clk_sys    (clk_sys),
		.reset_sys  (reset_mem_sys),
		.clk_ram    (clk_ram),
		.reset_ram  (reset_mem_ram),
		.dram_req   (dram_req),
		.dram_we    (dram_we),
		.dram_line  (dram_line),
		.dram_din   (dram_din),
		.dram_be    (dram_be),
		.dram_dout  (dram_dout),
		.dram_busy  (dram_busy),
		.dram_done  (dram_done),
		.ss_own     (ssb_dram_own),
		.ss_req     (ssb_dram_req),
		.ss_we      (ssb_dram_we),
		.ss_line    (ssb_dram_line),
		.ss_din     (ssb_dram_din),
		.ss_be      (ssb_dram_be),
		.ss_dout    (ssb_dram_dout),
		.ss_done    (ssb_dram_done),
		.rom_req    (cart_rom_req),
		.rom_line   (cart_rom_line),
		.rom_dout   (cart_rom_dout),
		.rom_busy   (cart_rom_busy),
		.rom_done   (cart_rom_done),
		.wave_req   (wave_req),
		.wave_line  (wave_line),
		.wave_dout  (wave_dout),
		.wave_busy  (wave_busy),
		.wave_done  (wave_done),
		.loading    (loading),
		.ld_req     (ld_req),
		.ld_addr    (ld_addr),
		.ld_din     (ld_din),
		.ld_busy    (ld_busy),
		/* verilator lint_off PINCONNECTEMPTY */
		.ld_done    (),
		/* verilator lint_on PINCONNECTEMPTY */
		.p0_req     (p0_req),
		.p0_we      (p0_we),
		.p0_addr    (p0_addr),
		.p0_din     (p0_din),
		.p0_byte_en (p0_byte_en),
		.p0_dout    (p0_dout),
		.p0_busy    (p0_busy),
		.p0_ready   (p0_ready),
		.p1_req     (p1_req),
		.p1_addr    (p1_addr),
		.p1_dout    (p1_dout),
		.p1_busy    (p1_busy),
		.p1_ready   (p1_ready),
		.p2_req     (p2_req),
		.p2_addr    (p2_addr),
		.p2_dout    (p2_dout),
		.p2_busy    (p2_busy),
		.p2_ready   (p2_ready)
	);

	// ---- savestate bulk walk -------------------------------------------------

	wire [9:0]  ssb_ram_addr;
	wire        ssb_ram_we;
	wire [7:0]  ssb_ram_wdata, ssb_ram_q;
	wire        ssb_sram_own, ssb_sram_we;
	wire [13:0] ssb_sram_addr;
	wire [1:0]  ssb_sram_be;
	wire [15:0] ssb_sram_wdata;
	wire        ssb_voice_rd, ssb_voice_wr, ssb_voice_ready;
	wire [10:0] ssb_voice_addr;
	wire [7:0]  ssb_voice_wdata, ssb_voice_q;
	wire        ssb_dram_own, ssb_dram_req, ssb_dram_we;
	wire [18:3] ssb_dram_line;
	wire [63:0] ssb_dram_din, ssb_dram_dout;
	wire [7:0]  ssb_dram_be;
	wire        ssb_dram_done;

	// The RH-7500's memories are on the video clock, so each byte goes through
	// the same handshake everything else in this core uses to change domain.
	// The engine allows at least seven clocks per byte, which is more than the
	// round trip costs.
	wire        ssv_req, ssv_done;
	wire [7:0]  ssv_src_resp;
	wire        ssv_valid, ssv_ack;
	wire [28:0] ssv_data;
	wire [7:0]  ssv_resp;

	wire [28:0] ssv_src_data = {ss_ram_type, ss_ram_addr[16:0], ss_ram_wr, ss_ram_wdata};

	cdc_handshake #(.WIDTH (29), .RESP_WIDTH (8)) u_ss_vdp (
		.src_clk   (clk_sys),
		.src_reset (reset_ss_sys),
		.src_req   (ssv_req),
		.src_data  (ssv_src_data),
		/* verilator lint_off PINCONNECTEMPTY */
		.src_busy  (),
		/* verilator lint_on PINCONNECTEMPTY */
		.src_done  (ssv_done),
		.src_resp  (ssv_src_resp),
		.dst_clk   (clk_video),
		.dst_reset (reset_ss_video),
		.dst_valid (ssv_valid),
		.dst_data  (ssv_data),
		.dst_ack   (ssv_ack),
		.dst_resp  (ssv_resp)
	);

	ss_bulk u_ss_bulk (
		.clk           (clk_sys),
		.reset         (reset_ss_sys),
		.addr_i        (ss_ram_addr),
		.type_i        (ss_ram_type),
		.rd_i          (ss_ram_rd),
		.wr_i          (ss_ram_wr),
		.wdata_i       (ss_ram_wdata),
		.rdata_o       (ss_ram_rdata),
		.ready_o       (ss_ram_ready),

		.dram_own_o    (ssb_dram_own),
		.dram_req_o    (ssb_dram_req),
		.dram_we_o     (ssb_dram_we),
		.dram_line_o   (ssb_dram_line),
		.dram_din_o    (ssb_dram_din),
		.dram_be_o     (ssb_dram_be),
		.dram_dout_i   (ssb_dram_dout),
		.dram_done_i   (ssb_dram_done),

		.vdp_req_o     (ssv_req),
		.vdp_done_i    (ssv_done),
		.vdp_rdata_i   (ssv_src_resp),

		.sram_own_o    (ssb_sram_own),
		.sram_addr_o   (ssb_sram_addr),
		.sram_we_o     (ssb_sram_we),
		.sram_be_o     (ssb_sram_be),
		.sram_wdata_o  (ssb_sram_wdata),
		.sram_q_i      (sram_b_q),

		.ram_addr_o    (ssb_ram_addr),
		.ram_we_o      (ssb_ram_we),
		.ram_wdata_o   (ssb_ram_wdata),
		.ram_q_i       (ssb_ram_q),

		.voice_rd_o    (ssb_voice_rd),
		.voice_wr_o    (ssb_voice_wr),
		.voice_addr_o  (ssb_voice_addr),
		.voice_wdata_o (ssb_voice_wdata),
		.voice_q_i     (ssb_voice_q),
		.voice_ready_i (ssb_voice_ready)
	);

	assign ss_dout = ss_dout_cpu | ss_dout_vdp | ss_dout_snd
	               | ss_dout_print | ss_dout_pcap;

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	// /CS0 goes nowhere on this board, WDTOVF reaches the cartridge's /PRST,
	// the VDP decodes A19-A0 only, EXP2 and EXP3 are unpopulated, and PB7 is
	// an input the cartridge drives.
	wire unused = &{1'b0, cs0_n, wdtovf_n, a[21:20], pa_o[15:7],
	                pb_o[15:12], pb_o[9], pb_o[7:0], pa_oe[15:7],
	                pb_oe[15:12], pb_oe[9], pb_oe[7:0],
	                exp_strobe_n[2:1]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
endmodule
