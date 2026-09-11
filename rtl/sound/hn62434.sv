// Copyright (c) 2026 Jamie Blanks

// LSI352, the HN62434 512K x 8 mask ROM on the synth's private bus: patch
// tables and every PCM sample.
//
// The image lives in SDRAM at 0x480000 and is read eight bytes at a time
// through the memory bridge's wave port. The pitch increment table and the
// control region are mirrored in block RAM, filled by watching the loader's
// writes, so the sampler and sequencer get them in one cycle.

module hn62434 (
	input  wire        clk_i,

	// Line read. Pulse rd_i for one cycle while busy_o is low; done_o pulses
	// once with dout_o valid.
	input  wire        rd_i,
	input  wire [18:3] line_i,
	output wire [63:0] dout_o,
	output wire        busy_o,
	output wire        done_o,

	// Pitch increment table, ROM offset 0x1600 + index * 2. One cycle.
	input  wire [10:0] pitch_idx_i,
	output wire [15:0] pitch_q_o,

	// The control region, byte addressed. One cycle, like the pitch table.
	input  wire [15:0] ctl_addr_i,
	output wire [7:0]  ctl_q_o,

	// The loader's word writes into SDRAM, watched for the mirror.
	input  wire        ld_wr_i,
	input  wire [18:0] ld_addr_i,      // offset within the wave image
	input  wire [15:0] ld_data_i,

	// The memory bridge's wave port.
	output wire        wave_req_o,
	output wire [18:3] wave_line_o,
	input  wire [63:0] wave_dout_i,
	input  wire        wave_busy_i,
	input  wire        wave_done_i
);
	localparam [18:0] PITCH_BASE = 19'h1600;
	localparam [18:0] PITCH_END  = 19'h2200;

	assign wave_req_o  = rd_i;
	assign wave_line_o = line_i;
	assign dout_o      = wave_dout_i;
	assign busy_o      = wave_busy_i;
	assign done_o      = wave_done_i;

	// ---- pitch table mirror ------------------------------------------------

	wire mirror_wr = ld_wr_i & ~ld_addr_i[0] &
	                 (ld_addr_i >= PITCH_BASE) & (ld_addr_i < PITCH_END);
	wire [10:0] mirror_wa = ld_addr_i[11:1] - PITCH_BASE[11:1];

	// Port A takes the loader's writes, port B answers the sampler.
	cache_ram_dp #(.ADDR_WIDTH (11), .DATA_WIDTH (16)) u_pitch (
		.clk_i     (clk_i),
		.addr_a_i  (mirror_wa),
		.wren_a_i  (mirror_wr),
		.wdata_a_i (ld_data_i),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  (pitch_idx_i),
		.wren_b_i  (1'b0),
		.wdata_b_i (16'd0),
		.q_b_o     (pitch_q_o)
	);

	// ---- control region mirror ---------------------------------------------
	//
	// Every control table ends below 0xB612 and sample data starts there, so
	// mirroring 0x0000-0xBFFF covers every envelope, descriptor and keymap.
	localparam [18:0] CTL_END = 19'hC000;

	wire        ctl_wr = ld_wr_i & ~ld_addr_i[0] & (ld_addr_i < CTL_END);
	wire [14:0] ctl_wa = ld_addr_i[15:1];

	wire [15:0] ctl_word;
	reg         ctl_lsb;
	always @(posedge clk_i) ctl_lsb <= ctl_addr_i[0];
	assign ctl_q_o = ctl_lsb ? ctl_word[15:8] : ctl_word[7:0];

	cache_ram_dp #(.ADDR_WIDTH (15), .DATA_WIDTH (16)) u_ctl (
		.clk_i     (clk_i),
		.addr_a_i  (ctl_wa),
		.wren_a_i  (ctl_wr),
		.wdata_a_i (ld_data_i),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  (ctl_addr_i[15:1]),
		.wren_b_i  (1'b0),
		.wdata_b_i (16'd0),
		.q_b_o     (ctl_word)
	);

endmodule
