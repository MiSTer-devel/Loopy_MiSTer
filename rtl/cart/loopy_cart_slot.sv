// Copyright (c) 2026 Jamie Blanks

// The cartridge slot.
//
// Bus side of the 90-pin connector: A21-A1 and D15-D0, /ROM (/CS6), /SRAM
// (/CS2), /RD and /WRL, reaching `cart_rom` and `cart_sram`. The ROM sits on
// the full 16-bit bus and the SRAM on D7-D0 only, so a byte read of SRAM comes
// back on the low lane.
//
// Expansion side: the VDP's /EXP1-3 strobes and the port B GPIOs a cart may
// drive. The only known expansion board is Wanwan Aijou Monogatari's: a
// 74HC273A latch, an OKI MSM6653A-457 and a BA15218 op amp. Its speech ROM is
// built into the core, so nothing outside the slot needs to know the board
// exists.
//
// DET (PA8) is tied to VCC in every cartridge and says only that something is
// seated. Whether it carries the expansion board is decided here by watching
// for the write into expansion area 1 that only such a board receives. Until
// then the expansion chip is held in reset, NAR is not driven onto PB7, and
// the audio pin is silent, like an ordinary cartridge.

module loopy_cart_slot #(
	parameter int OSC_CE_NUM   = 45056,    // 4.096 MHz from clk_sys = 2835/88 MHz
	parameter int OSC_CE_DEN   = 354375,
	parameter int FS_DIV_SHIFT = 0,
	parameter int TICK_DIV     = 65536,
	// Speech ROM images.
	parameter     BANK_MIF     = "rtl/cart/wanwan_bank.mif",
	parameter     BANK_HEX     = "rtl/cart/wanwan_bank.hex"
) (
	input  logic               clk_sys,
	input  logic               rst,

	input  logic               cart_det,         // DET, PA8, high when seated

	// The bus side of the connector.
	input  logic [21:0]        a,
	// D15-D8 reach the connector but nothing on any known board reads them:
	// the SRAM is eight bits on D7-D0 and the expansion latch takes D6-D0.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [15:0]        d_in,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [15:0]        d_out,
	output logic               d_oe,
	input  logic               rom_n,            // /ROM,  the CPU's /CS6
	input  logic               sram_n,           // /SRAM, the CPU's /CS2
	input  logic               rd_n,
	input  logic               wrl_n,

	// Sizes the loader worked out from the image and the header, and whether
	// the board carries two mask ROMs rather than one.
	input  logic [21:1]        rom_mask,
	input  logic               rom_split,
	input  logic [20:1]        rom_hi_mask,
	input  logic [16:0]        sram_mask,

	input  logic               rd_take,          // the CPU takes a read word this cycle
	output logic               stall,
	output logic               sram_dirty,

	// Cartridge ROM lines out of SDRAM.
	output logic               rom_req,
	output logic [21:3]        rom_line,
	input  logic [63:0]        rom_dout,
	input  logic               rom_busy,
	input  logic               rom_done,

	// Cartridge SRAM, second port: the save slot and the savestate stream.
	input  logic [13:0]        sram_b_addr,
	input  logic               sram_b_wren,
	input  logic [1:0]         sram_b_be,
	input  logic [15:0]        sram_b_wdata,
	output logic [15:0]        sram_b_q,

	// /EXP1, from vdp_io_exp.  Slow mode on this cart: a 181 ns active-low
	// strobe.  A 74HC273A is positive-edge clocked, so the latch takes D6-D0
	// on the trailing edge, when the CPU's data is settled.
	input  logic               exp1_n,

	// SH7021 port B, cart side.  PB7 is an input to the CPU, driven only by a
	// cart that has something to say on it.
	input  logic               pb1_reset_n,
	input  logic               pb5_cmd,
	input  logic               pb8_ch,
	input  logic               pb10_st,
	output logic               pb7_nar,
	output logic               pb7_nar_oe,

	// Analog audio back to the mainboard's AUDL/AUDR mix.
	output logic signed [11:0] exp_aout,
	output logic               exp_aout_en
);


	// ------------------------------------------------------- ROM and SRAM
	wire [15:0] rom_d;
	wire        rom_oe;
	wire [7:0]  sram_d;
	wire        sram_oe;

	cart_rom u_rom (
		.clk_i      (clk_sys),
		.rst_i      (rst),
		.a_i        (a[21:1]),
		.cs_n_i     (rom_n | ~cart_det),
		.rd_n_i     (rd_n),
		.d_o        (rom_d),
		.d_oe_o     (rom_oe),
		.mask_i     (rom_mask),
		.split_i    (rom_split),
		.hi_mask_i  (rom_hi_mask),
		.sample_i   (rd_take),
		.stall_o    (stall),
		.mem_req_o  (rom_req),
		.mem_line_o (rom_line),
		.mem_dout_i (rom_dout),
		.mem_busy_i (rom_busy),
		.mem_done_i (rom_done)
	);

	cart_sram #(.ADDR_W (15)) u_sram (
		.clk_i       (clk_sys),
		.rst_i       (rst),
		.a_i         (a[16:0]),
		.cs_n_i      (sram_n | ~cart_det),
		.rd_n_i      (rd_n),
		.wr_n_i      (wrl_n),
		.d_i         (d_in[7:0]),
		.d_o         (sram_d),
		.d_oe_o      (sram_oe),
		.size_mask_i (sram_mask),
		.dirty_o     (sram_dirty),
		.b_addr_i    (sram_b_addr),
		.b_wren_i    (sram_b_wren),
		.b_be_i      (sram_b_be),
		.b_wdata_i   (sram_b_wdata),
		.b_q_o       (sram_b_q)
	);

	// The ROM answers on all sixteen lines; the SRAM only on the low eight,
	// and the rest of the bus floats to the board's pull-ups.
	assign d_out = rom_oe ? rom_d : {8'hFF, sram_d};
	assign d_oe  = rom_oe | sram_oe;

	// Trailing edge of /EXP1, synchronised into the fabric clock.
	logic [1:0] exp1_sync;

	always_ff @(posedge clk_sys)
		exp1_sync <= {exp1_sync[0], exp1_n};

	wire exp1_wr = ~exp1_sync[1] & exp1_sync[0];

	// The first write into expansion area 1 is the board announcing itself.
	// The 74HC273A clocks on any expansion write while a cart is seated, so
	// that same byte also reaches the chip.
	logic exp_fitted;

	always_ff @(posedge clk_sys)
		if (rst)                     exp_fitted <= 1'b0;
		else if (exp1_wr & cart_det) exp_fitted <= 1'b1;

	wire exp_en = cart_det & exp_fitted;

	logic               nar;
	logic signed [11:0] aout;

	// The chip's speech ROM is built into the core, so the board carries it.
	logic        bank_req, bank_ready;
	logic [23:0] bank_addr, bank_size;
	logic [7:0]  bank_data;

	cart_exp_bank_rom #(
		.MEM_INIT_FILE (BANK_MIF),
		.SIM_INIT_FILE (BANK_HEX)
	) u_bank_rom (
		.clk_sys (clk_sys),
		.req     (bank_req),
		.addr    (bank_addr),
		.data    (bank_data),
		.ready   (bank_ready),
		.size    (bank_size)
	);

	cart_exp_wanwan #(
		.OSC_CE_NUM   (OSC_CE_NUM),
		.OSC_CE_DEN   (OSC_CE_DEN),
		.FS_DIV_SHIFT (FS_DIV_SHIFT),
		.TICK_DIV     (TICK_DIV)
	) u_exp_audio (
		.clk_sys    (clk_sys),
		.exp1_wr    (exp1_wr & cart_det),
		.exp1_data  (d_in[6:0]),
		.pb_reset_n (pb1_reset_n & exp_en),
		.pb_cmd     (pb5_cmd),
		.pb_ch      (pb8_ch),
		.pb_st      (pb10_st),
		.pb_nar     (nar),
		.aout       (aout),
		// BUSY exists on the chip but no connector pin carries it.
		/* verilator lint_off PINCONNECTEMPTY */
		.busy_n     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.bank_size  (bank_size),
		.rom_req    (bank_req),
		.rom_addr   (bank_addr),
		.rom_data   (bank_data),
		.rom_ready  (bank_ready)
	);

	assign pb7_nar        = nar;
	assign pb7_nar_oe     = exp_en;
	assign exp_aout       = exp_en ? aout : 12'sd0;
	assign exp_aout_en    = exp_en;

endmodule
