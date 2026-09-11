// Expansion strobes and the sound control latch.
//
// The top of the VDP address space belongs to cartridge hardware. An access
// there pulls one of three select lines on the cart connector:
//
//   0x80000-0x9FFFF  sound control latch, write only, mirrored
//   0xA0000-0xBFFFF  /EXP1
//   0xC0000-0xDFFFF  /EXP2
//   0xE0000-0xFFFFF  /EXP3
//
// EXP_TIMING gives each area (sound control is area 0) a fast mode, write
// only and two CPU cycles, or a slow mode, read-write and just under three.

module vdp_io_exp
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

	// Register bus slice, for EXP_TIMING at 0x5D020.
	input  wire [11:1] addr,
	input  wire        wr,
	input  wire [1:0]  be,
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,
	output reg         hit,

	// Accesses decoded by vdp_cpu_if.
	input  wire        sel_sound,
	input  wire [2:0]  sel_exp,
	input  wire        cpu_wr,
	input  wire        cpu_busy,
	input  wire [15:0] cpu_wdata,

	output reg  [11:0] sound_ctrl,
	output wire [2:0]  exp_strobe_n,   // /EXP1 in bit 0, active low
	output wire [3:0]  exp_fast        // per area, sound control is bit 0
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_exp;
	/* verilator lint_on UNUSEDSIGNAL */

	reg [3:0] timing;
	assign exp_fast = timing;

	// The strobe follows the whole expansion window rather than pulsing, since
	// the cartridge samples it in the slower CPU clock domain. `cpu_busy` ends
	// before WAIT is released so the trailing edge, which clocks the cart's
	// 74HC273A, lands while the CPU is still driving D.
	reg [2:0] strobe;
	assign exp_strobe_n = ~strobe;

	always @(posedge clk) begin
		if (reset) begin
			sound_ctrl <= ss_exp[11:0];
			timing     <= ss_exp[15:12];
			strobe     <= 3'd0;
		end else begin
			strobe <= sel_exp & {3{cpu_busy}};

			if (sel_sound & cpu_wr) sound_ctrl <= cpu_wdata[11:0];

			if (wr & be[0] & (addr == 11'h010)) timing <= wdata[3:0];
		end
	end

	always @* begin
		rdata = 16'd0;
		hit   = 1'b0;
		if (addr == 11'h010) begin
			hit   = 1'b1;
			rdata = {12'd0, timing};
		end
	end

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"


	ss_reg #(.ADDR (SSW_VDP_IOEXP), .DEFAULT (64'd0)) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({48'd0, timing, sound_ctrl}),
		.dout_o     (ss_exp)
	);

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused = &{1'b0, be[1], wdata[15:4], cpu_wdata[15:12]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

endmodule
