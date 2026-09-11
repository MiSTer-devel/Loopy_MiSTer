// Copyright (c) 2026 Jamie Blanks

// The Wanwan Aijou Monogatari cart's expansion audio hardware: a 74HC273A
// latch, the OKI MSM6653A-457, and the phrase bank that stands in for the
// chip's undumped mask ROM.
//
//   VDP /EXP1 write ──► 74HC273A ──► I6-I0 ──┐
//   SH7021 PB1  RESET ───────────────────────┤
//   SH7021 PB5  CMD   ───────────────────────┼──► MSM6653A ──► AOUT
//   SH7021 PB8  CH    ───────────────────────┤        │
//   SH7021 PB10 ST    ───────────────────────┘        └──► phrase bank ──► ROM
//   SH7021 PB7  NAR   ◄───────────────────────────────┘
//
// The latch ignores every address line: any byte write into expansion area 1
// clocks it. /EXP1 is in slow mode for this cart (a 181 ns strobe), the BIOS
// default.

module cart_exp_wanwan #(
	parameter int OSC_CE_NUM   = 45056,    // 4.096 MHz from clk_sys = 2835/88 MHz
	parameter int OSC_CE_DEN   = 354375,
	parameter int FS_DIV_SHIFT = 0,
	parameter int TICK_DIV     = 65536
) (
	input  logic               clk_sys,

	// Expansion area 1 byte write, the /EXP1 strobe
	input  logic               exp1_wr,
	input  logic [6:0]         exp1_data,   // only D6-D0 reach the latch

	// SH7021 port B
	input  logic               pb_reset_n,   // PB1
	input  logic               pb_cmd,       // PB5
	input  logic               pb_ch,        // PB8
	input  logic               pb_st,        // PB10
	output logic               pb_nar,       // PB7, an input to the CPU

	output logic signed [11:0] aout,
	output logic               busy_n,

	// Phrase bank image, somewhere the cart can read bytes from
	input  logic [23:0]        bank_size,
	output logic               rom_req,
	output logic [23:0]        rom_addr,
	input  logic [7:0]         rom_data,
	input  logic               rom_ready
);

	// 74HC273A on D6-D0. No address decode and no reset path from the CPU,
	// only the cart's power-on clear.
	logic [6:0] latch;

	always_ff @(posedge clk_sys)
		if (exp1_wr) latch <= exp1_data;

	logic [6:0]  phrase_req;
	logic        phrase_ch;
	logic        phrase_req_stb;
	logic [23:0] phrase_start, phrase_len;
	logic [2:0]  phrase_rate;
	logic        phrase_valid;

	logic        chip_rom_req;
	logic        chip_rom_ch;
	logic [23:0] chip_rom_addr;
	logic [7:0]  chip_rom_data;
	logic        chip_rom_ready;

	cart_exp_msm6653 #(
		.OSC_CE_NUM   (OSC_CE_NUM),
		.OSC_CE_DEN   (OSC_CE_DEN),
		.FS_DIV_SHIFT (FS_DIV_SHIFT),
		.TICK_DIV     (TICK_DIV)
	) u_chip (
		.clk_sys        (clk_sys),
		.reset_pin_n    (pb_reset_n),
		.cmd_pin        (pb_cmd),
		.st_pin         (pb_st),
		.ch_pin         (pb_ch),
		.id_pins        (latch),
		.nar            (pb_nar),
		.busy_n         (busy_n),
		.aout           (aout),
		.phrase_req     (phrase_req),
		.phrase_ch      (phrase_ch),
		.phrase_req_stb (phrase_req_stb),
		.phrase_start   (phrase_start),
		.phrase_len     (phrase_len),
		.phrase_rate    (phrase_rate),
		.phrase_valid   (phrase_valid),
		.rom_req        (chip_rom_req),
		.rom_ch         (chip_rom_ch),
		.rom_addr       (chip_rom_addr),
		.rom_data       (chip_rom_data),
		.rom_ready      (chip_rom_ready)
	);

	cart_exp_msm6653_bank u_bank (
		.clk_sys        (clk_sys),
		.bank_size      (bank_size),
		.phrase_req     (phrase_req),
		.phrase_ch      (phrase_ch),
		.phrase_req_stb (phrase_req_stb),
		.phrase_start   (phrase_start),
		.phrase_len     (phrase_len),
		.phrase_rate    (phrase_rate),
		.phrase_valid   (phrase_valid),
		.chip_rom_req   (chip_rom_req),
		.chip_rom_ch    (chip_rom_ch),
		.chip_rom_addr  (chip_rom_addr),
		.chip_rom_data  (chip_rom_data),
		.chip_rom_ready (chip_rom_ready),
		.rom_req        (rom_req),
		.rom_addr       (rom_addr),
		.rom_data       (rom_data),
		.rom_ready      (rom_ready)
	);

endmodule
