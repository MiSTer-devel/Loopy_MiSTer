// Copyright (c) 2026 Jamie Blanks

// LSI351, the NEC CDT109: sample-based synthesizer, a uPD937GD-006
// derivative with 32 voice slots, four channels and 110 patches.
//
//   MIDI in  -> cdt109_midi    receiver and message parser
//   panel    -> cdt109_seq     channels, allocator, envelopes
//                cdt109_voices  the 32-slot sampler and its RAM
//                cdt109_rom     the ROM bus and the wave line buffers
//
// The sequencer and the sampler take turns with the voice RAM, so the only
// arbitration here is which of them reports a voice pointer to the ROM block.

module cdt109 (
	input  wire        clk_i,
	input  wire        rst_i,
	input  wire        ce_4m_i,        // the RH-7501's X201, for the receiver
	input  wire        ce_sample_i,    // and its X202 divided by 256

	input  wire        rxd_i,          // MIDI in, from SCI1's TxD
	input  wire        midi_gate_i,    // the panel's MIDI mode

	// The RH-7501's virtual panel.
	input  wire        cmd_valid_i,
	input  wire [1:0]  cmd_kind_i,
	input  wire [1:0]  cmd_arg_i,
	output wire        cmd_ack_o,
	input  wire [2:0]  slider_a_i,
	input  wire [2:0]  slider_r_i,

	// The HN62434's bus.
	output wire        rom_rd_o,
	output wire [18:3] rom_line_o,
	input  wire [63:0] rom_dout_i,
	input  wire        rom_busy_i,
	input  wire        rom_done_i,
	output wire [10:0] pitch_idx_o,
	input  wire [15:0] pitch_q_i,
	output wire [15:0] ctl_addr_o,
	input  wire [7:0]  ctl_q_i,

	// To the uPD6379.
	output wire        lrck_o,
	output wire signed [15:0] out_l_o,
	output wire signed [15:0] out_r_o,

	output wire        idle_o,

	// The savestate walk's window onto the voice RAM.
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
	`include "cdt109_defs.svh"

	// ---- MIDI --------------------------------------------------------------

	wire       ev_valid, ev_ack, midi_idle;
	wire [1:0] ev_channel;
	wire [2:0] ev_kind;
	wire [7:0] ev_value;
	wire [3:0] midi_en;
	wire [63:0] ss_dout_midi, ss_dout_seq;

	// In keyboard mode and while the demo tune plays, MIDI bytes are dropped
	// at the parser.
	cdt109_midi u_midi (
		.clk_i         (clk_i),
		.rst_i         (rst_i),
		.ce_4m_i       (ce_4m_i),
		.rxd_i         (rxd_i),
		.midi_en_i     (midi_gate_i ? midi_en : 4'd0),
		.ev_valid_o    (ev_valid),
		.ev_channel_o  (ev_channel),
		.ev_kind_o     (ev_kind),
		.ev_value_o    (ev_value),
		.ev_ack_i      (ev_ack),
		.idle_o        (midi_idle),
		.ss_din_i      (ss_din_i),
		.ss_addr_i     (ss_addr_i),
		.ss_wren_i     (ss_wren_i),
		.ss_rst_i      (ss_rst_i),
		.ss_dout_o     (ss_dout_midi)
	);

	// ---- the two engines ----------------------------------------------------

	wire        smp_run, smp_busy, smp_done;
	wire signed [15:0] smp_l, smp_r;
	wire [63:0] vch;
	wire [31:0] bend;
	wire  [2:0] slider_a_q, slider_r_q;

	wire  [4:0] seq_raddr;
	wire [319:0] ram_q;
	wire        seq_wr;
	wire  [4:0] seq_waddr;
	wire [319:0] seq_wdata;

	wire        seq_rd, seq_rom_done, seq_rom_busy;
	wire [18:0] seq_addr;
	wire  [7:0] seq_data;
	wire        seq_upd;
	wire  [4:0] seq_upd_voice;
	wire [15:0] seq_upd_cur, seq_upd_next;
	wire [31:0] cur_ready;
	wire        seq_idle, rom_idle;

	cdt109_seq u_seq (
		.clk_i           (clk_i),
		.rst_i           (rst_i),
		.ce_sample_i     (ce_sample_i),
		.out_l_o         (out_l_o),
		.out_r_o         (out_r_o),
		.cmd_valid_i     (cmd_valid_i),
		.cmd_kind_i      (cmd_kind_i),
		.cmd_arg_i       (cmd_arg_i),
		.cmd_ack_o       (cmd_ack_o),
		.ev_valid_i      (ev_valid),
		.ev_channel_i    (ev_channel),
		.ev_kind_i       (ev_kind),
		.ev_value_i      (ev_value),
		.ev_ack_o        (ev_ack),
		.midi_en_o       (midi_en),
		.slider_a_i      (slider_a_i),
		.slider_r_i      (slider_r_i),
		.slider_a_o      (slider_a_q),
		.slider_r_o      (slider_r_q),
		.smp_run_o       (smp_run),
		.smp_busy_i      (smp_busy),
		.smp_done_i      (smp_done),
		.smp_l_i         (smp_l),
		.smp_r_i         (smp_r),
		.vch_o           (vch),
		.bend_o          (bend),
		.ram_raddr_o     (seq_raddr),
		.ram_q_i         (ram_q),
		.ram_wr_o        (seq_wr),
		.ram_waddr_o     (seq_waddr),
		.ram_wdata_o     (seq_wdata),
		.rom_rd_o        (seq_rd),
		.rom_addr_o      (seq_addr),
		.rom_data_i      (seq_data),
		.rom_done_i      (seq_rom_done),
		.rom_busy_i      (seq_rom_busy),
		.upd_o           (seq_upd),
		.upd_voice_o     (seq_upd_voice),
		.upd_line_cur_o  (seq_upd_cur),
		.upd_line_next_o (seq_upd_next),
		.cur_ready_i     (cur_ready),
		.rom_idle_i      (rom_idle),
		.idle_o          (seq_idle),
		.ss_din_i        (ss_din_i),
		.ss_addr_i       (ss_addr_i),
		.ss_wren_i       (ss_wren_i),
		.ss_rst_i        (ss_rst_i),
		.ss_dout_o       (ss_dout_seq)
	);

	wire        smp_sel;
	wire  [4:0] smp_voice;
	wire  [1:0] smp_word;
	wire        smp_hit;
	wire [15:0] smp_data;
	wire        vo_upd;
	wire  [4:0] vo_upd_voice;
	wire [15:0] vo_upd_cur, vo_upd_next;
	wire        vo_upd_cross;

	cdt109_voices u_voices (
		.clk_i             (clk_i),
		.rst_i             (rst_i),
		.run_i             (smp_run),
		.busy_o            (smp_busy),
		.done_o            (smp_done),
		.out_l_o           (smp_l),
		.out_r_o           (smp_r),
		.vch_i             (vch),
		.bend_i            (bend),
		.slider_a_i        (SLIDER_LEVEL[slider_a_q]),
		.slider_r_i        (SLIDER_LEVEL[slider_r_q]),
		.seq_raddr_i       (seq_raddr),
		.seq_q_o           (ram_q),
		.seq_wr_i          (seq_wr),
		.seq_waddr_i       (seq_waddr),
		.seq_wdata_i       (seq_wdata),
		.smp_sel_o         (smp_sel),
		.smp_voice_o       (smp_voice),
		.smp_word_o        (smp_word),
		.smp_hit_i         (smp_hit),
		.smp_data_i        (smp_data),
		.upd_o             (vo_upd),
		.upd_voice_o       (vo_upd_voice),
		.upd_line_cur_o    (vo_upd_cur),
		.upd_line_next_o   (vo_upd_next),
		.upd_cross_o       (vo_upd_cross),
		.pitch_idx_o       (pitch_idx_o),
		.pitch_q_i         (pitch_q_i),
		.ssb_rst_i         (ssb_rst_i),
		.ssb_rd_i          (ssb_rd_i),
		.ssb_wr_i          (ssb_wr_i),
		.ssb_addr_i        (ssb_addr_i),
		.ssb_wdata_i       (ssb_wdata_i),
		.ssb_q_o           (ssb_q_o),
		.ssb_ready_o       (ssb_ready_o)
	);

	// Only one of the two ever has the voice RAM, so only one ever has a
	// pointer to report. A note start is the sequencer's; a line crossing is
	// the sampler's.
	wire        upd       = vo_upd | seq_upd;
	wire  [4:0] upd_voice = vo_upd ? vo_upd_voice : seq_upd_voice;
	wire [15:0] upd_cur   = vo_upd ? vo_upd_cur   : seq_upd_cur;
	wire [15:0] upd_next  = vo_upd ? vo_upd_next  : seq_upd_next;

	cdt109_rom u_rom (
		.clk_i           (clk_i),
		.rst_i           (rst_i),
		.smp_sel_i       (smp_sel),
		.smp_voice_i     (smp_voice),
		.smp_word_i      (smp_word),
		.smp_hit_o       (smp_hit),
		.smp_word_o      (smp_data),
		.upd_i           (upd),
		.upd_voice_i     (upd_voice),
		.upd_line_cur_i  (upd_cur),
		.upd_line_next_i (upd_next),
		.upd_cross_i     (vo_upd & vo_upd_cross),
		.upd_reset_i     (~vo_upd & seq_upd),
		.cur_ready_o     (cur_ready),
		.seq_rd_i        (seq_rd),
		.seq_addr_i      (seq_addr),
		.seq_data_o      (seq_data),
		.seq_done_o      (seq_rom_done),
		.seq_busy_o      (seq_rom_busy),
		.ctl_addr_o      (ctl_addr_o),
		.ctl_q_i         (ctl_q_i),
		.idle_o          (rom_idle),
		.rom_rd_o        (rom_rd_o),
		.rom_line_o      (rom_line_o),
		.rom_dout_i      (rom_dout_i),
		.rom_busy_i      (rom_busy_i),
		.rom_done_i      (rom_done_i)
	);

	// The queue publishes its sample on the sample clock, so the DAC's frame
	// pulse is one cycle behind it.
	reg lrck;
	always @(posedge clk_i) lrck <= rst_i ? 1'b0 : ce_sample_i;
	assign lrck_o    = lrck;
	assign idle_o    = seq_idle & midi_idle & rom_idle & ~smp_busy;
	assign ss_dout_o = ss_dout_midi | ss_dout_seq;

endmodule
