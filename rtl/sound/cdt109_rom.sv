// Copyright (c) 2026 Jamie Blanks

// The CDT109's side of the wave ROM bus: which line to fetch next, and the
// line buffers the eight bytes land in. Sequencer table reads are answered
// from the HN62434's control mirror; only the sampler reaches SDRAM.
//
// Each voice owns two eight-byte halves: the line it is playing and the line
// it will play next. "Next" follows the loop, so a looping sample has its
// loop point ready before the pointer wraps:
//
//   ptr ...  3C 3D 3E 3F | 40 41 42 43 | ...        end = 41, loop = 20
//            current line| next line   |
//                          ^ ptr passes end here and jumps to 20, so the line
//                            worth having ready is the one holding 20
//
// A pointer advances at most one word per output sample, so a prefetch has
// four sample periods to land. One access is outstanding at a time; a voice
// waiting on its current line beats a prefetch.

module cdt109_rom (
	input  wire        clk_i,
	// A savestate restore comes through this reset; the line buffers are
	// simply refetched.
	input  wire        rst_i,

	// ---- sampler, one voice per slot ----
	// Presented in the slot's second stage; hit is answered that same cycle,
	// the word one cycle later.
	input  wire        smp_sel_i,
	input  wire [4:0]  smp_voice_i,
	input  wire [1:0]  smp_word_i,      // which word of the line, ptr[1:0]
	output wire        smp_hit_o,
	output wire [15:0] smp_word_o,

	// Where the pointer ended up, two stages later. cross: the two halves
	// swapped roles. reset: the pointer jumped and both halves are stale.
	input  wire        upd_i,
	input  wire [4:0]  upd_voice_i,
	input  wire [15:0] upd_line_cur_i,
	input  wire [15:0] upd_line_next_i,
	input  wire        upd_cross_i,
	input  wire        upd_reset_i,

	// Set once the voice's current line is in its buffer, so a note start can
	// wait for it.
	output wire [31:0] cur_ready_o,

	// ---- sequencer, one byte at a time, out of the control mirror ----
	input  wire        seq_rd_i,
	input  wire [18:0] seq_addr_i,
	output reg  [7:0]  seq_data_o,
	output reg         seq_done_o,
	output wire        seq_busy_o,
	output wire [15:0] ctl_addr_o,
	input  wire [7:0]  ctl_q_i,

	output wire        idle_o,

	// ---- the mask ROM ----
	output reg         rom_rd_o,
	output reg  [18:3] rom_line_o,
	input  wire [63:0] rom_dout_i,
	input  wire        rom_busy_i,
	input  wire        rom_done_i
);
	// ---- per-voice bookkeeping --------------------------------------------

	reg [31:0] valid_0, valid_1;    // the half holds the line it should
	reg [31:0] want_0,  want_1;     // and someone has asked for the ones missing
	reg [31:0] cur_sel;             // which half is the line being played

	// The two line numbers each voice wants, written every slot by whoever
	// owns the pointer. Kept in a memory so the 32-way mux does not sit behind
	// the priority encoder.
	reg  [4:0]  shadow_wa;
	reg         shadow_wr;
	reg  [31:0] shadow_wd;
	reg  [4:0]  shadow_ra;
	wire [31:0] shadow_q;

	cache_ram_dp #(.ADDR_WIDTH (5), .DATA_WIDTH (32)) u_shadow (
		.clk_i     (clk_i),
		.addr_a_i  (shadow_wa),
		.wren_a_i  (shadow_wr),
		.wdata_a_i (shadow_wd),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  (shadow_ra),
		.wren_b_i  (1'b0),
		.wdata_b_i (32'd0),
		.q_b_o     (shadow_q)
	);

	reg [15:0] sched_line;

	wire       smp_half = cur_sel[smp_voice_i];
	assign smp_hit_o    = smp_sel_i & (smp_half ? valid_1[smp_voice_i] : valid_0[smp_voice_i]);
	assign cur_ready_o  = (valid_1 & cur_sel) | (valid_0 & ~cur_sel);

	// ---- line buffer -------------------------------------------------------

	// Samples are 12 bits in the top of each 16-bit ROM word, so a line is
	// kept as four 12-bit fields.
	reg  [1:0]  smp_word_sel;
	wire [47:0] buf_q;
	reg         buf_wr;
	reg  [5:0]  buf_wa;
	reg  [47:0] buf_wd;

	cache_ram_dp #(.ADDR_WIDTH (6), .DATA_WIDTH (48)) u_buf (
		.clk_i     (clk_i),
		.addr_a_i  (buf_wa),
		.wren_a_i  (buf_wr),
		.wdata_a_i (buf_wd),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  ({smp_voice_i, smp_half}),
		.wren_b_i  (1'b0),
		.wdata_b_i (48'd0),
		.q_b_o     (buf_q)
	);

	always @(posedge clk_i) smp_word_sel <= smp_word_i;
	assign smp_word_o = {buf_q[smp_word_sel * 12 +: 12], 4'd0};

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, seq_addr_i[18:16], rom_dout_i[51:48],
	                rom_dout_i[35:32], rom_dout_i[19:16], rom_dout_i[3:0]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

	// ---- what to fetch next ------------------------------------------------

	reg        busy;              // an access is out and has not landed
	reg  [4:0] req_voice;
	reg        req_half;
	reg        req_kill;
	reg        req_pre;

	wire [31:0] req_bit    = 32'd1 << req_voice;
	wire [31:0] inflight_0 = (busy & ~req_half) ? req_bit : 32'd0;
	wire [31:0] inflight_1 = (busy &  req_half) ? req_bit : 32'd0;

	wire [31:0] pend_0 = want_0 & ~valid_0 & ~inflight_0;
	wire [31:0] pend_1 = want_1 & ~valid_1 & ~inflight_1;

	wire [31:0] need_block = (pend_0 & ~cur_sel) | (pend_1 &  cur_sel);
	wire [31:0] need_pre   = (pend_0 &  cur_sel) | (pend_1 & ~cur_sel);

	// Lowest set bit, as a balanced tree so the path is five levels deep
	// instead of thirty-two.
	function automatic [5:0] first_set(input [31:0] v);
		logic [4:0] i;
		logic [15:0] w16;
		logic [7:0]  w8;
		logic [3:0]  w4;
		/* verilator lint_off UNUSEDSIGNAL */
		logic [1:0]  w2;             // only bit 0 decides the last level
		/* verilator lint_on UNUSEDSIGNAL */
		begin
			i[4] = ~(|v[15:0]);
			w16  = i[4] ? v[31:16] : v[15:0];
			i[3] = ~(|w16[7:0]);
			w8   = i[3] ? w16[15:8] : w16[7:0];
			i[2] = ~(|w8[3:0]);
			w4   = i[2] ? w8[7:4] : w8[3:0];
			i[1] = ~(|w4[1:0]);
			w2   = i[1] ? w4[3:2] : w4[1:0];
			i[0] = ~w2[0];
			first_set = {~(|v), i};
		end
	endfunction

	wire [5:0] pick_block = first_set(need_block);
	wire [5:0] pick_pre   = first_set(need_pre);

	// ---- the sequencer's read ----------------------------------------------
	//
	// Answered from the control mirror: the address goes out with the request
	// and the byte comes back the next cycle.
	reg        seq_pending;
	reg [15:0] seq_addr_q;
	assign ctl_addr_o = seq_rd_i ? seq_addr_i[15:0] : seq_addr_q;

	assign seq_busy_o = seq_pending;
	assign idle_o = ~busy & ~seq_pending & (need_block == 32'd0) & (need_pre == 32'd0);

	// ---- scheduler ---------------------------------------------------------

	localparam [2:0] SC_IDLE = 3'd0, SC_LOOK1 = 3'd1, SC_LOOK2 = 3'd2,
	                 SC_ADDR = 3'd3, SC_WAIT = 3'd4;
	reg [2:0] sc;

	// A note start on the voice whose access is in flight has to reach the
	// same cycle the access lands, or stale data would be marked valid.
	wire kill_now = upd_i & upd_reset_i & busy & (req_voice == upd_voice_i);

	always @(posedge clk_i) begin
		if (rst_i) begin
			valid_0 <= 32'd0; valid_1 <= 32'd0;
			want_0  <= 32'd0; want_1  <= 32'd0;
			cur_sel <= 32'd0;
			busy <= 1'b0; req_voice <= 5'd0; req_half <= 1'b0;
			req_kill <= 1'b0; req_pre <= 1'b0;
			sc <= SC_IDLE;
			rom_rd_o <= 1'b0; rom_line_o <= 16'd0;
			seq_pending <= 1'b0; seq_addr_q <= 16'd0;
			seq_data_o <= 8'd0; seq_done_o <= 1'b0;
			sched_line <= 16'd0;
			buf_wr <= 1'b0; buf_wa <= 6'd0; buf_wd <= 48'd0;
			shadow_wr <= 1'b0; shadow_wa <= 5'd0; shadow_wd <= 32'd0;
			shadow_ra <= 5'd0;
		end else begin
			rom_rd_o   <= 1'b0;
			buf_wr     <= 1'b0;
			shadow_wr  <= 1'b0;
			seq_done_o <= 1'b0;

			// -- the sampler asked and missed: that half is now wanted
			if (smp_sel_i && !smp_hit_o) begin
				if (smp_half) want_1[smp_voice_i] <= 1'b1;
				else          want_0[smp_voice_i] <= 1'b1;
			end

			// -- the slot's result
			if (upd_i) begin
				shadow_wr <= 1'b1;
				shadow_wa <= upd_voice_i;
				shadow_wd <= {upd_line_next_i, upd_line_cur_i};

				if (upd_reset_i) begin
					// Both halves are stale and both are wanted: the note's
					// first line, and the line after it.
					valid_0[upd_voice_i] <= 1'b0;
					valid_1[upd_voice_i] <= 1'b0;
					want_0[upd_voice_i]  <= 1'b1;
					want_1[upd_voice_i]  <= 1'b1;
					cur_sel[upd_voice_i] <= 1'b0;
				end else if (upd_cross_i) begin
					// The half just left becomes the one to fetch ahead.
					cur_sel[upd_voice_i] <= ~cur_sel[upd_voice_i];
					if (cur_sel[upd_voice_i]) begin
						valid_1[upd_voice_i] <= 1'b0;
						want_1[upd_voice_i]  <= 1'b1;
					end else begin
						valid_0[upd_voice_i] <= 1'b0;
						want_0[upd_voice_i]  <= 1'b1;
					end
				end
			end

			// -- the sequencer's request
			if (seq_rd_i && !seq_pending) begin
				seq_addr_q  <= seq_addr_i[15:0];
				seq_pending <= 1'b1;
			end else if (seq_pending) begin
				seq_data_o  <= ctl_q_i;
				seq_done_o  <= 1'b1;
				seq_pending <= 1'b0;
			end

			if (kill_now) req_kill <= 1'b1;

			case (sc)
			SC_IDLE: begin
				if (!pick_block[5]) begin
					req_voice  <= pick_block[4:0];
					req_half   <= cur_sel[pick_block[4:0]];
					req_kill   <= 1'b0;
					req_pre    <= 1'b0;
					shadow_ra  <= pick_block[4:0];
					busy       <= 1'b1;
					sc         <= SC_LOOK1;
				end else if (!pick_pre[5]) begin
					req_voice  <= pick_pre[4:0];
					req_half   <= ~cur_sel[pick_pre[4:0]];
					req_kill   <= 1'b0;
					req_pre    <= 1'b1;
					shadow_ra  <= pick_pre[4:0];
					busy       <= 1'b1;
					sc         <= SC_LOOK1;
				end
			end

			// The memory sees shadow_ra this cycle and answers in the next.
			SC_LOOK1: sc <= SC_LOOK2;
			SC_LOOK2: begin
				sched_line <= req_pre ? shadow_q[31:16] : shadow_q[15:0];
				sc         <= SC_ADDR;
			end

			// The bridge takes one cycle of req while busy is low.
			SC_ADDR: if (!rom_busy_i) begin
				rom_rd_o   <= 1'b1;
				rom_line_o <= sched_line;
				sc         <= SC_WAIT;
			end

			SC_WAIT: begin
				if (rom_done_i) begin
					if (!req_kill && !kill_now) begin
						buf_wr <= 1'b1;
						buf_wa <= {req_voice, req_half};
						buf_wd <= {rom_dout_i[63:52], rom_dout_i[47:36],
						           rom_dout_i[31:20], rom_dout_i[15:4]};
						if (req_half) valid_1[req_voice] <= 1'b1;
						else          valid_0[req_voice] <= 1'b1;
					end
					busy <= 1'b0;
					sc   <= SC_IDLE;
				end
			end

			default: sc <= SC_IDLE;
			endcase
		end
	end

endmodule
