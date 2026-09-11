// Copyright (c) 2026 Jamie Blanks

// The board's sound section, wired as on the schematic:
//
//   SH7021 SCI1 TxD ---- MIDI 31250 baud ----+
//                                            v
//   VDP SOUND_CTRL ----> RH-7501 --panel----> CDT109 --frame--> uPD6379 --> amp
//                        (LSI201)             (LSI351)          (IC251)
//                                                |
//                                          HN62434 (LSI352)
//
// The loader tap feeds the HN62434's block RAM mirrors as the wave image is
// written into SDRAM.

module loopy_sound (
	input  wire        clk_i,
	input  wire        rst_i,
	input  wire        ce_4m_i,
	input  wire        ce_sample_i,

	// SOUND_CTRL, already across from the VDP's clock domain.
	input  wire [11:0] snd_ctl_i,
	// SCI1's transmit pin.
	input  wire        midi_rxd_i,

	// The memory bridge's wave port.
	output wire        wave_req_o,
	output wire [18:3] wave_line_o,
	input  wire [63:0] wave_dout_i,
	input  wire        wave_busy_i,
	input  wire        wave_done_i,

	// The loader's writes into the wave region.
	input  wire        ld_wr_i,
	input  wire [18:0] ld_addr_i,
	input  wire [15:0] ld_data_i,

	output wire signed [15:0] aout_l_o,
	output wire signed [15:0] aout_r_o,

	// High when the whole section is at a sample boundary with nothing in
	// flight, which is what a savestate waits for.
	output wire        idle_o,

	// The savestate walk's window onto the synth's working RAM. It shares the
	// bulk region with the SH7021's on-chip RAM.
	input  wire        ssb_rst_i,
	input  wire        ssb_rd_i,
	input  wire        ssb_wr_i,
	input  wire [10:0] ssb_addr_i,
	input  wire [7:0]  ssb_wdata_i,
	output wire [7:0]  ssb_q_o,
	output wire        ssb_ready_o,

	// Savestate.
	input  wire [63:0] ss_din_i,
	input  wire [9:0]  ss_addr_i,
	input  wire        ss_wren_i,
	input  wire        ss_rst_i,
	output wire [63:0] ss_dout_o
);
	wire       cmd_valid, cmd_ack;
	wire [1:0] cmd_kind, cmd_arg;
	wire [2:0] slider_a, slider_r;
	wire       midi_gate, rh_idle, synth_idle;
	wire [63:0] ss_dout_rh, ss_dout_synth, ss_dout_filter;

	rh7501 u_rh7501 (
		.clk_i       (clk_i),
		.rst_i       (rst_i),
		.snd_ctl_i   (snd_ctl_i),
		.cmd_valid_o (cmd_valid),
		.cmd_kind_o  (cmd_kind),
		.cmd_arg_o   (cmd_arg),
		.cmd_ack_i   (cmd_ack),
		.slider_a_o  (slider_a),
		.slider_r_o  (slider_r),
		.midi_gate_o (midi_gate),
		.idle_o      (rh_idle),
		.ss_din_i    (ss_din_i),
		.ss_addr_i   (ss_addr_i),
		.ss_wren_i   (ss_wren_i),
		.ss_rst_i    (ss_rst_i),
		.ss_dout_o   (ss_dout_rh)
	);

	wire        rom_rd, rom_busy, rom_done;
	wire [18:3] rom_line;
	wire [63:0] rom_dout;
	wire [10:0] pitch_idx;
	wire [15:0] pitch_q;
	wire [15:0] ctl_addr;
	wire  [7:0] ctl_q;

	hn62434 u_hn62434 (
		.clk_i       (clk_i),
		.rd_i        (rom_rd),
		.line_i      (rom_line),
		.dout_o      (rom_dout),
		.busy_o      (rom_busy),
		.done_o      (rom_done),
		.pitch_idx_i (pitch_idx),
		.pitch_q_o   (pitch_q),
		.ctl_addr_i  (ctl_addr),
		.ctl_q_o     (ctl_q),
		.ld_wr_i     (ld_wr_i),
		.ld_addr_i   (ld_addr_i),
		.ld_data_i   (ld_data_i),
		.wave_req_o  (wave_req_o),
		.wave_line_o (wave_line_o),
		.wave_dout_i (wave_dout_i),
		.wave_busy_i (wave_busy_i),
		.wave_done_i (wave_done_i)
	);

	wire signed [15:0] synth_l, synth_r;
	wire lrck;

	cdt109 u_cdt109 (
		.clk_i             (clk_i),
		.rst_i             (rst_i),
		.ce_4m_i           (ce_4m_i),
		.ce_sample_i       (ce_sample_i),
		.rxd_i             (midi_rxd_i),
		.midi_gate_i       (midi_gate),
		.cmd_valid_i       (cmd_valid),
		.cmd_kind_i        (cmd_kind),
		.cmd_arg_i         (cmd_arg),
		.cmd_ack_o         (cmd_ack),
		.slider_a_i        (slider_a),
		.slider_r_i        (slider_r),
		.rom_rd_o          (rom_rd),
		.rom_line_o        (rom_line),
		.rom_dout_i        (rom_dout),
		.rom_busy_i        (rom_busy),
		.rom_done_i        (rom_done),
		.pitch_idx_o       (pitch_idx),
		.pitch_q_i         (pitch_q),
		.ctl_addr_o        (ctl_addr),
		.ctl_q_i           (ctl_q),
		.lrck_o            (lrck),
		.out_l_o           (synth_l),
		.out_r_o           (synth_r),
		.idle_o            (synth_idle),
		.ssb_rst_i         (ssb_rst_i),
		.ssb_rd_i          (ssb_rd_i),
		.ssb_wr_i          (ssb_wr_i),
		.ssb_addr_i        (ssb_addr_i),
		.ssb_wdata_i       (ssb_wdata_i),
		.ssb_q_o           (ssb_q_o),
		.ssb_ready_o       (ssb_ready_o),
		.ss_din_i          (ss_din_i),
		.ss_addr_i         (ss_addr_i),
		.ss_wren_i         (ss_wren_i),
		.ss_rst_i          (ss_rst_i),
		.ss_dout_o         (ss_dout_synth)
	);

	wire signed [15:0] dac_l, dac_r;

	upd6379 u_dac (
		.clk_i    (clk_i),
		.rst_i    (rst_i),
		.lrck_i   (lrck),
		.l_i      (synth_l),
		.r_i      (synth_r),
		.aout_l_o (dac_l),
		.aout_r_o (dac_r)
	);

	// The DAC latches on the frame pulse, so the analogue stage runs one cycle
	// behind it and sees the sample that just landed.
	reg lrck_d;
	always @(posedge clk_i) lrck_d <= rst_i ? 1'b0 : lrck;

	loopy_audio_filter u_filter (
		.clk_i       (clk_i),
		.rst_i       (rst_i),
		.ce_sample_i (lrck_d),
		.in_l_i      (dac_l),
		.in_r_i      (dac_r),
		.out_l_o     (aout_l_o),
		.out_r_o     (aout_r_o),
		.ss_din_i    (ss_din_i),
		.ss_addr_i   (ss_addr_i),
		.ss_wren_i   (ss_wren_i),
		.ss_rst_i    (ss_rst_i),
		.ss_dout_o   (ss_dout_filter)
	);

	assign idle_o    = rh_idle & synth_idle;
	assign ss_dout_o = ss_dout_rh | ss_dout_synth | ss_dout_filter;

endmodule
