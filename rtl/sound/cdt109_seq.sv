// Copyright (c) 2026 Jamie Blanks

// The CDT109's control sequencer: the four channels, the note allocator, the
// two envelope walkers, and the order they run in. For every output sample:
//
//   1. whatever MIDI and panel changes have arrived
//   2. the volume envelope pass, every 96th sample
//   3. the pitch envelope pass, every 695th
//   4. the sampler pass, in cdt109_voices
//
// Each step reads what the one before it wrote, so the sequencer runs to
// completion before starting the sampler. The output goes through a queue
// drained by the sample clock, so a slow sample costs queue depth.
// ROM reads go through cdt109_rom one byte at a time.

module cdt109_seq (
	input  wire        clk_i,
	input  wire        rst_i,

	// The sample clock drains the queue; the engine fills it.
	input  wire        ce_sample_i,
	output reg signed [15:0] out_l_o,
	output reg signed [15:0] out_r_o,

	// Panel commands from the RH-7501.
	input  wire        cmd_valid_i,
	input  wire [1:0]  cmd_kind_i,
	input  wire [1:0]  cmd_arg_i,
	output reg         cmd_ack_o,

	// MIDI events.
	input  wire        ev_valid_i,
	input  wire [1:0]  ev_channel_i,
	input  wire [2:0]  ev_kind_i,
	input  wire [7:0]  ev_value_i,
	output reg         ev_ack_o,
	output wire [3:0]  midi_en_o,

	// The panel's two volume sliders, latched so a change lands between
	// samples.
	input  wire [2:0]  slider_a_i,
	input  wire [2:0]  slider_r_i,
	output reg  [2:0]  slider_a_o,
	output reg  [2:0]  slider_r_o,

	// The sampler.
	output reg         smp_run_o,
	input  wire        smp_busy_i,
	input  wire        smp_done_i,
	input  wire signed [15:0] smp_l_i,
	input  wire signed [15:0] smp_r_i,
	output wire [63:0] vch_o,
	output wire [31:0] bend_o,

	// The voice RAM, through the sampler's spare port.
	output wire [4:0]   ram_raddr_o,
	input  wire [319:0] ram_q_i,
	output reg          ram_wr_o,
	output reg  [4:0]   ram_waddr_o,
	output reg  [319:0] ram_wdata_o,

	// The wave ROM bus.
	output reg         rom_rd_o,
	output reg  [18:0] rom_addr_o,
	input  wire [7:0]  rom_data_i,
	input  wire        rom_done_i,
	input  wire        rom_busy_i,
	output reg         upd_o,
	output reg  [4:0]  upd_voice_o,
	output reg  [15:0] upd_line_cur_o,
	output reg  [15:0] upd_line_next_o,
	input  wire [31:0] cur_ready_i,
	input  wire        rom_idle_i,

	output wire        idle_o,

	// Savestate.
	input  wire [63:0] ss_din_i,
	input  wire [9:0]  ss_addr_i,
	input  wire        ss_wren_i,
	input  wire        ss_rst_i,
	output wire [63:0] ss_dout_o
);
	`include "cdt109_defs.svh"
	`include "ss_map.svh"

	/* verilator lint_off UNUSEDPARAM */
	localparam [1:0] CMD_CFG = 2'd0, CMD_RESET = 2'd1;
	localparam [2:0] EV_NOTE_OFF = 3'd0, EV_NOTE_ON = 3'd1, EV_PROG = 3'd2,
	                 EV_BEND = 3'd3, EV_SUSTAIN = 3'd4;
	/* verilator lint_on UNUSEDPARAM */
	// ---- channel state -----------------------------------------------------

	reg  [3:0] ch_midi_en;
	reg  [3:0] ch_sustain;
	reg  [3:0] ch_layered;
	reg  [4:0] ch_first    [4];
	reg  [5:0] ch_count    [4];
	reg [15:0] ch_partials [4];
	reg  [7:0] ch_keymap   [4];
	reg signed [7:0] ch_bend [4];
	reg  [4:0] ch_alloc    [4];

	assign midi_en_o = ch_midi_en;
	assign bend_o = {ch_bend[3], ch_bend[2], ch_bend[1], ch_bend[0]};

	reg [63:0] vch;
	assign vch_o = vch;

	reg [31:0] v_active, v_sustained;
	reg        booted;

	// Table pointers out of the wave ROM's header.
	reg [18:0] p_partials, p_pitchenv, p_volenv, p_sampdesc;

	reg  [6:0] venv_cnt;      // 0-95; the volume envelope pass falls on zero
	reg [23:0] clk2_acc;
	localparam [23:0] CLK2_INC = 24'd15625;
	localparam [23:0] CLK2_DIV = 24'd10862592;   // 128 * 84864

	// ---- output queue ------------------------------------------------------
	//
	// Eight samples deep; the worst case, all 32 voices taking two envelope
	// steps in one pass, is under eight periods. Running dry repeats a sample.
	// The queue is part of the savestate.

	reg [31:0] oq [8];
	reg  [3:0] oq_wr, oq_rd;
	wire oq_full  = (oq_wr[2:0] == oq_rd[2:0]) && (oq_wr[3] != oq_rd[3]);
	wire oq_empty = (oq_wr == oq_rd);

	always @(posedge clk_i) begin
		if (rst_i) begin
			oq_wr <= ss_global2[35:32];
			oq_rd <= ss_global2[39:36];
			out_l_o <= 16'sd0; out_r_o <= 16'sd0;
			for (int q = 0; q < 8; q++) oq[q] <= ss_oq[q >> 1][{q[0], 5'd0} +: 32];
		end else begin
			if (smp_done_i && !oq_full) begin
				oq[oq_wr[2:0]] <= {smp_r_i, smp_l_i};
				oq_wr <= oq_wr + 4'd1;
			end
			if (ce_sample_i) begin
				if (!oq_empty) begin
					out_l_o <= $signed(oq[oq_rd[2:0]][15:0]);
					out_r_o <= $signed(oq[oq_rd[2:0]][31:16]);
					oq_rd   <= oq_rd + 4'd1;
				end
			end
		end
	end

	// ---- working registers -------------------------------------------------

	voice_t w;                     // the voice being worked on
	voice_t rq;                    // and what the RAM is presenting
	assign rq = voice_t'(ram_q_i);

	reg  [5:0] v_idx;
	reg  [5:0] v_end;
	reg  [1:0] op_ch;
	reg  [7:0] op_val;
	reg  [6:0] op_note;            // the note as MIDI sent it
	reg  [6:0] note_r;             // and folded into the sampled range
	reg  [2:0] vpn;                // voice slots per note, 2 or 4
	reg  [2:0] vn;
	reg  [1:0] cfg_arg;
	reg [16:0] partial_addr;
	reg [15:0] tmp16;
	reg  [7:0] tmp8;
	reg [13:0] cur_penv;
	reg        already;
	reg  [4:0] alloc_v, alloc_i;

	reg [18:0] rd_addr;
	reg  [1:0] rd_n, rd_i;
	reg [23:0] rd_val;
	reg  [6:0] rd_ret, op_ret, cmd_ret;
	reg  [2:0] ev_kind;

	assign ram_raddr_o = v_idx[4:0];

	// ---- states ------------------------------------------------------------

	localparam [6:0]
		S_BOOT   = 7'd0,  S_BOOT1  = 7'd1,  S_BOOT2  = 7'd2,  S_BOOT3  = 7'd3,
		S_BOOT4  = 7'd4,
		S_TOP    = 7'd5,  S_VENVGO = 7'd6,  S_PENVGO = 7'd7,
		S_RUN    = 7'd8,  S_RUNW   = 7'd9,
		S_RDREQ  = 7'd10, S_RDWAIT = 7'd11,
		S_PC0    = 7'd12, S_PC1    = 7'd13, S_PC2    = 7'd14, S_PC3 = 7'd15,
		S_PC4    = 7'd16, S_PC5    = 7'd17,
		S_RC0    = 7'd18, S_RC1    = 7'd19, S_RC2    = 7'd20, S_RC3 = 7'd21,
		S_CFG    = 7'd22,
		S_EV     = 7'd23,
		S_NON0   = 7'd24, S_NON1   = 7'd25, S_NON2   = 7'd26, S_NON3 = 7'd27,
		S_NON4   = 7'd28, S_NON5   = 7'd29, S_NON6   = 7'd30, S_NON7 = 7'd31,
		S_NON8   = 7'd32, S_NON9   = 7'd33, S_NON10  = 7'd34, S_NON11 = 7'd35,
		S_NON12  = 7'd36, S_NON13  = 7'd37, S_NON14  = 7'd38, S_NON15 = 7'd39,
		S_NON16  = 7'd40, S_NON17  = 7'd41,
		S_GF0    = 7'd42, S_GF1    = 7'd43,
		S_NOF0   = 7'd44, S_NOF1   = 7'd45, S_NOF2   = 7'd46,
		S_BEND0  = 7'd47, S_BEND1  = 7'd48,
		S_SUS    = 7'd49,
		S_VE0    = 7'd50, S_VE1    = 7'd51, S_VE2    = 7'd52, S_VE3 = 7'd53,
		S_VE4    = 7'd54, S_VE5    = 7'd55, S_VE6    = 7'd56, S_VE7 = 7'd57,
		S_VENEXT = 7'd58,
		S_PE0    = 7'd59, S_PE1    = 7'd60, S_PE2    = 7'd61, S_PE3 = 7'd62,
		S_PE4    = 7'd63, S_PE5    = 7'd64, S_PE6    = 7'd65,
		S_PENEXT = 7'd66,
		S_VE7A   = 7'd67, S_PE6A   = 7'd68, S_NON18  = 7'd69,
		S_BOOT5  = 7'd70, S_NLD1   = 7'd71, S_NLD2   = 7'd72,
		S_REHY0  = 7'd73, S_REHY1  = 7'd74, S_REHY2  = 7'd75, S_REHY3 = 7'd76,
		S_REHY4  = 7'd77;

	reg [6:0] st;
	assign idle_o = (st == S_TOP) & ~cmd_valid_i & ~ev_valid_i;

	// ---- helpers -----------------------------------------------------------

	// Silence a voice the way a program change does: decay to nothing over 512
	// samples, in the release phase so the envelope walker leaves it alone.
	function automatic [319:0] silence(input [319:0] word);
		voice_t t;
		begin
			t = voice_t'(word);
			t.volume_rate_mul = 16'((({1'b0, t.volume} + 17'd511) >> 9));
			t.volume_rate_div = 9'd1;
			t.volume_target   = 16'd0;
			t.volume_down     = 1'b1;
			t.volume_env_step = 5'd16;
			silence = t;
		end
	endfunction

	// One pass of the volume envelope divider, for the three passes in four
	// that do nothing else.
	function automatic [319:0] bump_phase(input [319:0] word);
		voice_t t;
		begin
			t = voice_t'(word);
			t.volume_env_phase = t.volume_env_phase + 2'd1;
			bump_phase = t;
		end
	endfunction

	// A channel's voice slots as a bit mask.
	function automatic [31:0] ch_mask(input [4:0] first, input [5:0] count);
		ch_mask = (32'hFFFFFFFF >> (6'd32 - count)) << first;
	endfunction

	wire [31:0] sus_release = v_sustained & ch_mask(ch_first[op_ch], ch_count[op_ch]);

	wire [18:0] volenv_base   = p_volenv   + {w.volume_env, 6'd0};
	wire [18:0] pitchenv_base = p_pitchenv + {w.pitch_env, 5'd0};

	// The line a pointer is on, and the line it wants next: the loop point
	// when the sample ends inside this line.
	wire [17:0] w_line_last = {w.sample_ptr[17:2], 2'b11};
	// Only the line number is used.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [18:0] w_follow = (w.sample_end <= {1'b0, w_line_last}) ? w.sample_loop
	                                                            : ({1'b0, w_line_last} + 19'd1);
	/* verilator lint_on UNUSEDSIGNAL */

	// The volume envelope's decision, all of it comparisons on w.
	wire [9:0] ve_delay_next = w.volume_env_delay - 10'd1;
	wire       ve_delaying   = (w.volume_env_delay > 10'd0);
	wire       ve_hold       = ve_delaying & (ve_delay_next > 10'd0);
	wire       ve_delay_done = ve_delaying & ~ve_hold & v_active[v_idx[4:0]];
	wire       ve_release    = (w.volume_env_step < 5'd16) & (w.volume > 16'd0)
	                           & ~v_active[v_idx[4:0]];
	wire       ve_at_target  = w.volume_down ? (w.volume <= w.volume_target)
	                                         : (w.volume >= w.volume_target);
	wire       ve_advance    = ve_at_target & (w.volume_target > 16'd0)
	                           & (w.volume_rate_mul != 16'd0);
	wire [4:0] ve_next_step  = {w.volume_env_step[4], (w.volume_env_step[3:0] + 4'd1)};
	wire       ve_changed    = ve_delay_done | ve_release | (~ve_release & ve_advance);

	// The pitch envelope's, the same way.
	wire [16:0] pe_delay_next = w.pitch_env_delay - 17'd1;
	wire        pe_delaying   = (w.pitch_env_delay > 17'd0);
	wire        pe_hold       = pe_delaying & (pe_delay_next > 17'd0);
	wire signed [19:0] pe_value_next = w.pitch_env_value + $signed({{7{w.pitch_env_rate[12]}},
	                                                               w.pitch_env_rate});
	wire        pe_ramping = (w.pitch_env_rate != 13'sd0);
	wire        pe_reached = pe_ramping &
	              ((w.pitch_env_rate > 13'sd0) ? (pe_value_next >= w.pitch_env_target)
	                                           : (pe_value_next <= w.pitch_env_target));
	wire  [3:0] pe_next_step = (w.pitch_env_step + 4'd1 >= 4'd8) ? 4'd1
	                                                             : w.pitch_env_step + 4'd1;
	wire        pe_changed = (pe_delaying & ~pe_hold) | pe_reached;

	// The step a pitch envelope entry adds to the target. The word is
	// unsigned; no target word in the ROM exceeds 0x190, so twenty bits hold
	// the result.
	wire signed [19:0] pe_tgt_step = $signed({rd_val[15:0], 4'd0});
	wire signed [19:0] pe_tgt_next = tmp16[12] ? (w.pitch_env_target - pe_tgt_step)
	                                           : (w.pitch_env_target + pe_tgt_step);

	// note_on's arithmetic. Only the low bit of the offset picks the keymap
	// nibble, and the partial offset is a word index that fits in sixteen.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [6:0] nr_off  = note_r - 7'd36;
	wire [3:0] keymap_val = nr_off[0] ? rd_val[7:4] : rd_val[3:0];
	wire [16:0] partial_step = (vpn == 3'd4) ? {9'd0, keymap_val, 4'd0} - {11'd0, keymap_val, 2'd0}
	                                         : {10'd0, keymap_val, 3'd0} - {12'd0, keymap_val, 1'd0};
	/* verilator lint_on UNUSEDSIGNAL */
	wire signed [8:0]  note_delta = $signed({2'd0, note_r}) - $signed({1'b0, tmp8});
	wire signed [13:0] note_pitch = (tmp8 != 8'd0) ? {note_delta, 5'd0} : 14'sd512;
	wire signed [12:0] pi_mag = $signed({1'b0, tmp16[11:0]});
	wire signed [19:0] pi_val = tmp16[15:12] != 4'd0 ? -{{3{pi_mag[12]}}, pi_mag, 4'd0}
	                                                 :  {{3{pi_mag[12]}}, pi_mag, 4'd0};

	// ---- the machine -------------------------------------------------------

	integer k;

	always @(posedge clk_i) begin
		if (rst_i) begin
			// A restored state has already read the ROM header and set its
			// channels up; a cold start has not.
			st <= ss_global[63] ? S_REHY0 : S_BOOT;
			ram_wr_o <= 1'b0; ram_waddr_o <= 5'd0; ram_wdata_o <= 320'd0;
			rom_rd_o <= 1'b0; rom_addr_o <= 19'd0;
			upd_o <= 1'b0; upd_voice_o <= 5'd0;
			upd_line_cur_o <= 16'd0; upd_line_next_o <= 16'd0;
			cmd_ack_o <= 1'b0; ev_ack_o <= 1'b0; smp_run_o <= 1'b0;
			v_idx <= 6'd0; v_end <= 6'd0; op_ch <= 2'd0; op_val <= 8'd0;
			op_note <= 7'd0; note_r <= 7'd0; vpn <= 3'd2; vn <= 3'd0;
			cfg_arg <= 2'd0; partial_addr <= 17'd0; tmp16 <= 16'd0; tmp8 <= 8'd0;
			cur_penv <= 14'd0; already <= 1'b0;
			alloc_v <= 5'd0; alloc_i <= 5'd0;
			rd_addr <= 19'd0; rd_n <= 2'd1; rd_i <= 2'd0; rd_val <= 24'd0;
			rd_ret <= S_TOP; op_ret <= S_TOP; cmd_ret <= S_TOP; ev_kind <= 3'd0;
			w <= '0;
			slider_a_o <= ss_global2[42:40]; slider_r_o <= ss_global2[45:43];
			booted <= ss_global[63];

			venv_cnt    <= ss_global[6:0];
			clk2_acc    <= ss_global[30:7];
			v_active    <= ss_global[62:31];
			v_sustained <= ss_global2[31:0];
			vch         <= ss_vch;
			p_partials  <= ss_ptr0[18:0];
			p_pitchenv  <= ss_ptr0[37:19];
			p_volenv    <= ss_ptr0[56:38];
			p_sampdesc  <= ss_ptr1[18:0];
			ch_midi_en  <= {ss_chw[3][36], ss_chw[2][36], ss_chw[1][36], ss_chw[0][36]};
			ch_sustain  <= {ss_chw[3][11], ss_chw[2][11], ss_chw[1][11], ss_chw[0][11]};
			ch_layered  <= {ss_chw[3][45], ss_chw[2][45], ss_chw[1][45], ss_chw[0][45]};

			for (k = 0; k < 4; k = k + 1) begin
				ch_first[k]    <= ss_chw[k][4:0];
				ch_count[k]    <= ss_chw[k][10:5];
				ch_partials[k] <= ss_chw[k][27:12];
				ch_keymap[k]   <= ss_chw[k][35:28];
				ch_bend[k]     <= $signed(ss_chw[k][44:37]);
				ch_alloc[k]    <= ss_chw[k][50:46];
			end
		end else begin
			rom_rd_o     <= 1'b0;
			ram_wr_o     <= 1'b0;
			upd_o        <= 1'b0;
			cmd_ack_o    <= 1'b0;
			ev_ack_o     <= 1'b0;
			smp_run_o    <= 1'b0;

			case (st)

			// ---- power on --------------------------------------------------
			S_BOOT:  begin rd_addr <= 19'd0; rd_n <= 2'd2; rd_ret <= S_BOOT1; st <= S_RDREQ; end
			S_BOOT1: begin p_partials <= {rd_val[13:0], 5'd0};
			               rd_addr <= 19'd2; rd_n <= 2'd2; rd_ret <= S_BOOT2; st <= S_RDREQ; end
			S_BOOT2: begin p_pitchenv <= {rd_val[13:0], 5'd0};
			               rd_addr <= 19'd4; rd_n <= 2'd2; rd_ret <= S_BOOT3; st <= S_RDREQ; end
			S_BOOT3: begin p_volenv <= {rd_val[13:0], 5'd0};
			               rd_addr <= 19'd6; rd_n <= 2'd2; rd_ret <= S_BOOT4; st <= S_RDREQ; end
			S_BOOT4: begin
				p_sampdesc <= {rd_val[13:0], 5'd0};
				// Program 0 on all four channels, then the single-channel
				// layout.
				op_val  <= 8'd0;
				cmd_ret <= S_BOOT5;
				st      <= S_RC0;
			end
			S_BOOT5: begin
				cfg_arg <= 2'b00;
				cmd_ret <= S_TOP;
				booted  <= 1'b1;
				st      <= S_CFG;
			end

			// ---- after a restore -------------------------------------------
			//
			// The savestate holds the sample pointers but not the wave lines,
			// so refetch every voice's lines before the first sample.
			S_REHY0: begin v_idx <= 6'd0; st <= S_REHY1; end
			S_REHY1: st <= S_REHY2;
			S_REHY2: begin w <= rq; st <= S_REHY3; end
			S_REHY3: begin
				upd_o           <= 1'b1;
				upd_voice_o     <= v_idx[4:0];
				upd_line_cur_o  <= w.sample_ptr[17:2];
				upd_line_next_o <= w_follow[17:2];
				v_idx <= v_idx + 6'd1;
				st    <= (v_idx == 6'd31) ? S_REHY4 : S_REHY1;
			end
			S_REHY4: if (!upd_o && rom_idle_i) st <= S_TOP;

			// ---- one output sample -----------------------------------------
			S_TOP: begin
				if (cmd_valid_i) begin
					cmd_ack_o    <= 1'b1;
					if (cmd_kind_i == CMD_CFG) begin
						cfg_arg <= cmd_arg_i;
						cmd_ret <= S_TOP;
						st      <= S_CFG;
					end else begin
						op_val  <= cmd_arg_i[0] ? 8'd0 : 8'd128;
						cmd_ret <= S_TOP;
						st      <= S_RC0;
					end
				end else if (ev_valid_i) begin
					ev_ack_o     <= 1'b1;
					op_ch        <= ev_channel_i;
					op_val       <= ev_value_i;
					op_note      <= ev_value_i[6:0];
					ev_kind      <= ev_kind_i;
					op_ret       <= S_TOP;
					st           <= S_EV;
				end else if ((slider_a_o != slider_a_i) || (slider_r_o != slider_r_i)) begin
					slider_a_o   <= slider_a_i;
					slider_r_o   <= slider_r_i;
				end else if (!oq_full) begin
					if (venv_cnt == 7'd0) begin
						op_ret <= S_VENVGO;
						st     <= S_VE0;
					end else begin
						st <= S_VENVGO;
					end
				end
			end

			S_VENVGO: begin
				if (clk2_acc + CLK2_INC >= CLK2_DIV) begin
					clk2_acc <= clk2_acc + CLK2_INC - CLK2_DIV;
					op_ret   <= S_PENVGO;
					st       <= S_PE0;
				end else begin
					clk2_acc <= clk2_acc + CLK2_INC;
					st       <= S_PENVGO;
				end
			end

			S_PENVGO: begin smp_run_o <= 1'b1; st <= S_RUN; end
			S_RUN:  if (smp_busy_i) st <= S_RUNW;
			S_RUNW: if (!smp_busy_i) begin
				venv_cnt     <= (venv_cnt == 7'd95) ? 7'd0 : venv_cnt + 7'd1;
				st           <= S_TOP;
			end

			// ---- byte reader -------------------------------------------------
			S_RDREQ: if (!rom_busy_i && !rom_rd_o) begin
				rom_rd_o   <= 1'b1;
				rom_addr_o <= rd_addr + {17'd0, rd_i};
				st         <= S_RDWAIT;
			end
			S_RDWAIT: if (rom_done_i) begin
				rd_val[{rd_i, 3'd0} +: 8] <= rom_data_i;
				if (rd_i + 2'd1 >= rd_n) begin
					rd_i <= 2'd0;
					st   <= rd_ret;
				end else begin
					rd_i <= rd_i + 2'd1;
					st   <= S_RDREQ;
				end
			end

			// ---- program change ----------------------------------------------
			S_PC0: begin
				v_idx <= {1'b0, ch_first[op_ch]};
				v_end <= {1'b0, ch_first[op_ch]} + ch_count[op_ch];
				st    <= (ch_count[op_ch] == 6'd0) ? S_PC3 : S_PC1;
			end
			S_PC1: st <= S_PC2;                    // the RAM sees v_idx now
			S_PC2: begin
				ram_wr_o    <= 1'b1;
				ram_waddr_o <= v_idx[4:0];
				ram_wdata_o <= silence(ram_q_i);
				v_active[v_idx[4:0]]    <= 1'b0;
				v_sustained[v_idx[4:0]] <= 1'b0;
				v_idx <= v_idx + 6'd1;
				st    <= (v_idx + 6'd1 >= v_end) ? S_PC3 : S_PC1;
			end
			S_PC3: begin
				ch_alloc[op_ch] <= 5'd0;
				if (op_val > 8'd109) begin
					st <= op_ret;
				end else begin
					rd_addr <= ROM_INSTDESC + {9'd0, op_val, 2'd0};
					rd_n    <= 2'd2;
					rd_ret  <= S_PC4;
					st      <= S_RDREQ;
				end
			end
			S_PC4: begin
				ch_partials[op_ch] <= rd_val[15:0];
				rd_addr <= ROM_INSTDESC + {9'd0, op_val, 2'd0} + 19'd2;
				rd_n    <= 2'd2;
				rd_ret  <= S_PC5;
				st      <= S_RDREQ;
			end
			S_PC5: begin
				ch_keymap[op_ch]  <= rd_val[7:0];
				ch_layered[op_ch] <= rd_val[12];      // flags bit 4
				st <= op_ret;
			end

			// ---- reset the four channels --------------------------------------
			S_RC0: begin op_ch <= 2'd0; op_ret <= S_RC1;    st <= S_PC0; end
			S_RC1: begin op_ch <= 2'd1; op_ret <= S_RC2;    st <= S_PC0; end
			S_RC2: begin op_ch <= 2'd2; op_ret <= S_RC3;    st <= S_PC0; end
			S_RC3: begin op_ch <= 2'd3; op_ret <= cmd_ret;  st <= S_PC0; end

			// ---- channel configuration ------------------------------------------
			S_CFG: begin
				if (cfg_arg[1]) begin
					ch_first[0] <= 5'd0;  ch_count[0] <= 6'd12;
					ch_first[1] <= 5'd12; ch_count[1] <= 6'd8;
					ch_first[2] <= 5'd20; ch_count[2] <= 6'd4;
					ch_first[3] <= 5'd24; ch_count[3] <= 6'd8;
					ch_midi_en  <= {cfg_arg[0], 3'b111};
					vch <= {{8{2'd3}}, {4{2'd2}}, {8{2'd1}}, {12{2'd0}}};
				end else begin
					ch_first[0] <= 5'd0; ch_count[0] <= 6'd24;
					ch_count[1] <= 6'd0; ch_count[2] <= 6'd0; ch_count[3] <= 6'd0;
					ch_midi_en  <= 4'b0001;
					vch <= 64'd0;
				end
				st <= cmd_ret;
			end

			// ---- MIDI event ------------------------------------------------------
			S_EV: case (ev_kind)
				EV_NOTE_ON:  st <= S_NON0;
				EV_NOTE_OFF: st <= S_NOF0;
				EV_PROG:     st <= S_PC0;
				EV_BEND:     st <= S_BEND0;
				default:     st <= S_SUS;
			endcase

			// ---- note on ---------------------------------------------------------
			S_NON0: begin
				note_r <= op_note;
				vpn    <= ch_layered[op_ch] ? 3'd4 : 3'd2;
				vn     <= 3'd0;
				st     <= S_NON1;
			end
			S_NON1: begin
				// Fold the note into the sampled range, an octave at a time.
				if (note_r < 7'd36)      note_r <= note_r + 7'd12;
				else if (note_r > 7'd96) note_r <= note_r - 7'd12;
				else begin
					rd_addr <= 19'(ROM_KEYMAPS + {6'd0, ch_keymap[op_ch], 5'd0}
					                           + {13'd0, (note_r - 7'd36) >> 1});
					rd_n    <= 2'd1;
					rd_ret  <= S_NON2;
					st      <= S_RDREQ;
				end
			end
			S_NON2: begin
				// A partial group is three words per voice slot, and the word
				// offset doubles into a byte offset.
				partial_addr <= {ch_partials[op_ch] + partial_step[15:0], 1'b0};
				st <= S_GF0;
			end

			// ---- allocator ---------------------------------------------------------
			// Scan from where this channel stopped last time for a slot that is
			// not sounding, and take it whether or not one was found.
			S_GF0: begin
				alloc_v <= ch_first[op_ch] + ch_alloc[op_ch];
				alloc_i <= 5'd0;
				st      <= S_GF1;
			end
			S_GF1: begin
				ch_alloc[op_ch] <= (ch_alloc[op_ch] + 5'd1 >= ch_count[op_ch][4:0])
				                   ? 5'd0 : ch_alloc[op_ch] + 5'd1;
				if ((alloc_i >= ch_count[op_ch][4:0]) || !v_active[alloc_v]) begin
					v_idx <= {1'b0, alloc_v};
					st    <= S_NLD1;
				end else begin
					alloc_v <= ch_first[op_ch]
					         + ((ch_alloc[op_ch] + 5'd1 >= ch_count[op_ch][4:0])
					            ? 5'd0 : ch_alloc[op_ch] + 5'd1);
					alloc_i <= alloc_i + 5'd1;
				end
			end

			// The slot being taken over keeps sample_cur and the volume ramp
			// counter, so the word is read before it is rebuilt.
			S_NLD1: st <= S_NLD2;
			S_NLD2: begin
				w  <= rq;
				st <= S_NON3;
			end

			S_NON3: begin
				rd_addr <= p_partials + {2'd0, partial_addr};
				rd_n    <= 2'd2; rd_ret <= S_NON4; st <= S_RDREQ;
			end
			S_NON4: begin
				w.pitch_env <= rd_val[13:0];
				cur_penv    <= rd_val[13:0];
				rd_addr <= p_partials + {2'd0, partial_addr} + 19'd2;
				rd_n    <= 2'd2; rd_ret <= S_NON5; st <= S_RDREQ;
			end
			S_NON5: begin
				w.volume_env <= rd_val[12:0];
				rd_addr <= p_partials + {2'd0, partial_addr} + 19'd4;
				rd_n    <= 2'd2; rd_ret <= S_NON6; st <= S_RDREQ;
			end
			S_NON6: begin
				// A sample descriptor is ten bytes: note, start, end, loop.
				tmp16   <= rd_val[15:0];
				rd_addr <= p_sampdesc + {2'd0, rd_val[15:0], 1'b0}
				                      + {rd_val[15:0], 3'd0};
				rd_n    <= 2'd1; rd_ret <= S_NON7; st <= S_RDREQ;
			end
			S_NON7: begin
				tmp8    <= rd_val[7:0];
				rd_addr <= rd_addr + 19'd1;
				rd_n    <= 2'd3; rd_ret <= S_NON8; st <= S_RDREQ;
			end
			S_NON8: begin
				w.sample_ptr <= rd_val[18:0];
				rd_addr <= rd_addr + 19'd3;
				rd_n    <= 2'd3; rd_ret <= S_NON9; st <= S_RDREQ;
			end
			S_NON9: begin
				w.sample_end <= rd_val[18:0];
				rd_addr <= rd_addr + 19'd3;
				rd_n    <= 2'd3; rd_ret <= S_NON10; st <= S_RDREQ;
			end
			S_NON10: begin
				w.sample_loop  <= rd_val[18:0];
				w.sample_fract <= 15'd0;
				w.sample_last  <= 12'sd0;
				// sample_cur keeps whatever the slot last played.
				w.note   <= op_note;
				w.pitch  <= note_pitch;
				w.volume <= 16'd0;
				w.volume_target       <= 16'd0;
				w.volume_rate_mul     <= 16'd0;
				w.volume_rate_div     <= 9'd1;
				w.volume_down         <= 1'b0;
				w.volume_env_delay    <= 10'd0;
				w.volume_env_step     <= 5'd0;
				w.volume_env_phase    <= 2'd0;
				rd_addr <= p_volenv + {w.volume_env, 6'd0};
				rd_n    <= 2'd2; rd_ret <= S_NON11; st <= S_RDREQ;
			end
			S_NON11: begin
				tmp8 <= rd_val[7:0];                       // the rate byte
				if (rd_val[15:8] == 8'd0) begin
					// A target of zero means this first step is a delay.
					w.volume_env_delay <= ({2'd0, rd_val[7:0]} + 10'd1) << 1;
					w.volume_env_step  <= 5'd1;
					st <= S_NON14;
				end else begin
					w.volume_down <= rd_val[7];
					rd_addr <= ROM_VOLTABLE + {10'd0, rd_val[15:8], 1'b0};
					rd_n    <= 2'd2; rd_ret <= S_NON12; st <= S_RDREQ;
				end
			end
			S_NON12: begin
				w.volume_target <= rd_val[15:0];
				if (tmp8[6:0] == 7'd127) begin
					w.volume_rate_mul <= 16'hFFFF;
					w.volume_rate_div <= 9'd1;
					st <= S_NON14;
				end else begin
					rd_addr <= ROM_RATETABLE + {8'd0, tmp8[6:0], 3'd0} + 19'd8;
					rd_n    <= 2'd3; rd_ret <= S_NON13; st <= S_RDREQ;
				end
			end
			S_NON13: begin
				w.volume_rate_mul <= rd_val[15:0];
				w.volume_rate_div <= {1'b0, rd_val[23:16]} + 9'd1;
				st <= S_NON14;
			end
			S_NON14: begin
				rd_addr <= p_pitchenv + {cur_penv, 5'd0};
				rd_n    <= 2'd2; rd_ret <= S_NON15; st <= S_RDREQ;
			end
			S_NON15: begin
				tmp16 <= rd_val[15:0];
				rd_addr <= p_pitchenv + {cur_penv, 5'd0} + 19'd2;
				rd_n    <= 2'd2; rd_ret <= S_NON16; st <= S_RDREQ;
			end
			S_NON16: begin
				w.pitch_env_value  <= pi_val;
				w.pitch_env_target <= pi_val;
				w.pitch_env_rate   <= 13'sd0;
				w.pitch_env_step   <= 4'd1;
				w.pitch_env_delay  <= {1'b0, rd_val[15:0]} + 17'd1;
				st <= S_NON17;
			end
			S_NON17: begin
				// Write the slot, wake it, and ask for its wave line.
				ram_wr_o    <= 1'b1;
				ram_waddr_o <= v_idx[4:0];
				ram_wdata_o <= w;
				v_active[v_idx[4:0]]    <= 1'b1;
				v_sustained[v_idx[4:0]] <= 1'b0;
				upd_o           <= 1'b1;
				upd_voice_o     <= v_idx[4:0];
				upd_line_cur_o  <= w.sample_ptr[17:2];
				upd_line_next_o <= w_follow[17:2];
				st <= S_NON18;
			end
			S_NON18: begin
				// Wait for the line so the note starts on time. On this state's
				// first cycle cur_ready_i still describes the slot's previous
				// note, so let the update land first.
				if (!upd_o && cur_ready_i[v_idx[4:0]]) begin
					partial_addr <= partial_addr + 17'd6;
					vn <= vn + 3'd1;
					st <= (vn + 3'd1 >= vpn) ? op_ret : S_GF0;
				end
			end

			// ---- note off ---------------------------------------------------------
			S_NOF0: begin
				vpn   <= ch_layered[op_ch] ? 3'd4 : 3'd2;
				v_idx <= {1'b0, ch_first[op_ch]};
				v_end <= {1'b0, ch_first[op_ch]} + ch_count[op_ch];
				st    <= S_NOF1;
			end
			S_NOF1: st <= (v_idx >= v_end) ? op_ret : S_NOF2;
			S_NOF2: begin
				if ((rq.note == op_note) && v_active[v_idx[4:0]]
				    && !v_sustained[v_idx[4:0]]) begin
					if (ch_sustain[op_ch])
						v_sustained <= v_sustained | ch_mask(v_idx[4:0], {3'd0, vpn});
					else
						v_active <= v_active & ~ch_mask(v_idx[4:0], {3'd0, vpn});
					st <= op_ret;
				end else begin
					v_idx <= v_idx + {3'd0, vpn};
					st    <= S_NOF1;
				end
			end

			// ---- pitch bend --------------------------------------------------------
			S_BEND0: begin
				rd_addr <= ROM_RATETABLE + {9'd0, op_val, 2'd0} + 19'd3;
				rd_n    <= 2'd1; rd_ret <= S_BEND1; st <= S_RDREQ;
			end
			S_BEND1: begin
				// The table's bend column is biased by 128: flip the sign bit.
				ch_bend[op_ch] <= $signed({~rd_val[7], rd_val[6:0]});
				st <= op_ret;
			end

			// ---- sustain pedal ------------------------------------------------------
			S_SUS: begin
				ch_sustain[op_ch] <= op_val[0];
				// Lifting the pedal ends every note it was holding.
				if (!op_val[0]) begin
					v_sustained <= v_sustained & ~sus_release;
					v_active    <= v_active    & ~sus_release;
				end
				st <= op_ret;
			end

			// ---- volume envelope pass -----------------------------------------------
			S_VE0: begin v_idx <= 6'd0; st <= S_VE1; end
			S_VE1: st <= S_VE2;                        // the RAM sees v_idx now
			S_VE2: begin
				w <= rq;
				// Three passes out of four only move the divider on.
				if (rq.volume_env_phase != 2'd3) begin
					ram_wr_o    <= 1'b1;
					ram_waddr_o <= v_idx[4:0];
					ram_wdata_o <= bump_phase(ram_q_i);
					st <= S_VENEXT;
				end else begin
					w.volume_env_phase <= 2'd0;
					st <= S_VE3;
				end
			end
			S_VE3: begin
				already <= 1'b0;
				if (ve_delaying) w.volume_env_delay <= ve_delay_next;
				if (ve_hold) begin
					st <= S_VE7;
				end else begin
					if (ve_release)      w.volume_env_step <= w.volume_env_step | 5'd16;
					else if (ve_advance) w.volume_env_step <= ve_next_step;
					st <= ve_changed ? S_VE4 : S_VE7;
				end
			end
			S_VE4: begin
				rd_addr <= volenv_base + {13'd0, w.volume_env_step, 1'b0};
				rd_n    <= 2'd2; rd_ret <= S_VE5; st <= S_RDREQ;
			end
			S_VE5: begin
				tmp8    <= rd_val[7:0];
				rd_addr <= ROM_VOLTABLE + {10'd0, rd_val[15:8], 1'b0};
				rd_n    <= 2'd2; rd_ret <= S_VE6; st <= S_RDREQ;
			end
			S_VE6: begin
				w.volume_down   <= tmp8[7];
				w.volume_target <= rd_val[15:0];
				if (tmp8[6:0] == 7'd127) begin
					w.volume_rate_mul <= 16'hFFFF;
					w.volume_rate_div <= 9'd1;
					st <= S_VE7;
				end else if ((tmp8[6:0] == 7'd0) && tmp8[7]) begin
					// Rate zero going down is the hold every sustain uses.
					w.volume_rate_mul <= 16'd0;
					w.volume_rate_div <= 9'd1;
					st <= S_VE7;
				end else if ((rd_val[15:0] == 16'd0) && !tmp8[7] && !already) begin
					// A rising step to zero restarts the envelope; some use
					// "00 00" to loop.
					w.volume_env_step <= {w.volume_env_step[4], 4'd0};
					already <= 1'b1;
					st <= S_VE4;
				end else begin
					rd_addr <= ROM_RATETABLE + {8'd0, tmp8[6:0], 3'd0} + 19'd8;
					rd_n    <= 2'd3; rd_ret <= S_VE7A; st <= S_RDREQ;
				end
			end
			S_VE7A: begin
				w.volume_rate_mul <= rd_val[15:0];
				w.volume_rate_div <= {1'b0, rd_val[23:16]} + 9'd1;
				st <= S_VE7;
			end
			S_VE7: begin
				ram_wr_o    <= 1'b1;
				ram_waddr_o <= v_idx[4:0];
				ram_wdata_o <= w;
				st <= S_VENEXT;
			end
			S_VENEXT: begin
				v_idx <= v_idx + 6'd1;
				st    <= (v_idx == 6'd31) ? op_ret : S_VE1;
			end

			// ---- pitch envelope pass -------------------------------------------------
			S_PE0: begin v_idx <= 6'd0; st <= S_PE1; end
			S_PE1: st <= S_PE2;
			S_PE2: begin
				w <= rq;
				st <= (rq.volume == 16'd0) ? S_PENEXT : S_PE3;
			end
			S_PE3: begin
				already <= 1'b0;
				if (pe_delaying) w.pitch_env_delay <= pe_delay_next;
				if (pe_hold) begin
					st <= S_PE6;
				end else begin
					if (pe_ramping) begin
						w.pitch_env_value <= pe_reached ? w.pitch_env_target : pe_value_next;
						if (pe_reached) w.pitch_env_step <= pe_next_step;
					end
					st <= pe_changed ? S_PE4 : S_PE6;
				end
			end
			S_PE4: begin
				rd_addr <= pitchenv_base + {13'd0, w.pitch_env_step, 2'd0};
				rd_n    <= 2'd2; rd_ret <= S_PE5; st <= S_RDREQ;
			end
			S_PE5: begin
				tmp16   <= rd_val[15:0];
				rd_addr <= pitchenv_base + {13'd0, w.pitch_env_step, 2'd0} + 19'd2;
				rd_n    <= 2'd2; rd_ret <= S_PE6A; st <= S_RDREQ;
			end
			S_PE6A: begin
				if (tmp16[13]) begin
					// A loop entry sends the walk back to one of the first
					// eight steps, once.
					w.pitch_env_step <= {1'b0, tmp16[2:0]};
					already <= 1'b1;
					st <= already ? S_PE6 : S_PE4;
				end else begin
					w.pitch_env_rate <= tmp16[12] ? -$signed({1'b0, tmp16[11:0]})
					                              :  $signed({1'b0, tmp16[11:0]});
					w.pitch_env_target <= pe_tgt_next;
					st <= S_PE6;
				end
			end
			S_PE6: begin
				ram_wr_o    <= 1'b1;
				ram_waddr_o <= v_idx[4:0];
				ram_wdata_o <= w;
				st <= S_PENEXT;
			end
			S_PENEXT: begin
				v_idx <= v_idx + 6'd1;
				st    <= (v_idx == 6'd31) ? op_ret : S_PE1;
			end

			default: st <= S_TOP;
			endcase
		end
	end

	// ---- savestate ---------------------------------------------------------

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_global, ss_global2, ss_vch, ss_ptr0, ss_ptr1;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_chw [4];
	wire [63:0] ss_d_global, ss_d_global2, ss_d_vch, ss_d_ptr0, ss_d_ptr1;
	wire [63:0] ss_d_ch [4];

	ss_reg #(.ADDR (SSW_SND_GLOBAL)) u_ss_g (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_d_global),
		.din_i      ({booted, v_active, clk2_acc, venv_cnt}),
		.dout_o     (ss_global)
	);
	// Both sliders come up at position 4, which is bits 45:40 of this word.
	ss_reg #(.ADDR (SSW_SND_GLOBAL2), .DEFAULT (64'h0000_2400_0000_0000)) u_ss_g2 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_d_global2),
		.din_i      ({18'd0, slider_r_o, slider_a_o, oq_rd, oq_wr, v_sustained}),
		.dout_o     (ss_global2)
	);
	ss_reg #(.ADDR (SSW_SND_VCH)) u_ss_vch (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_d_vch),
		.din_i      (vch),
		.dout_o     (ss_vch)
	);
	ss_reg #(.ADDR (SSW_SND_PTR0)) u_ss_p0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_d_ptr0),
		.din_i      ({7'd0, p_volenv, p_pitchenv, p_partials}),
		.dout_o     (ss_ptr0)
	);
	ss_reg #(.ADDR (SSW_SND_PTR1)) u_ss_p1 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_d_ptr1),
		.din_i      ({45'd0, p_sampdesc}),
		.dout_o     (ss_ptr1)
	);

	wire [63:0] ss_oq [4];
	wire [63:0] ss_d_oq [4];

	genvar gq;
	generate
		for (gq = 0; gq < 4; gq = gq + 1) begin : g_oq
			ss_reg #(.ADDR (SSW_SND_OQ + gq)) u_ss_oq (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din_i),
				.bus_addr_i (ss_addr_i),
				.bus_wren_i (ss_wren_i),
				.bus_rst_i  (ss_rst_i),
				.bus_dout_o (ss_d_oq[gq]),
				.din_i      ({oq[gq * 2 + 1], oq[gq * 2]}),
				.dout_o     (ss_oq[gq])
			);
		end
	endgenerate

	genvar gi;
	generate
		for (gi = 0; gi < 4; gi = gi + 1) begin : g_ch
			ss_reg #(.ADDR (SSW_SND_CH + gi)) u_ss_ch (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din_i),
				.bus_addr_i (ss_addr_i),
				.bus_wren_i (ss_wren_i),
				.bus_rst_i  (ss_rst_i),
				.bus_dout_o (ss_d_ch[gi]),
				.din_i      ({13'd0, ch_alloc[gi], ch_layered[gi], ch_bend[gi], ch_midi_en[gi], ch_keymap[gi], ch_partials[gi], ch_sustain[gi], ch_count[gi], ch_first[gi]}),
				.dout_o     (ss_chw[gi])
			);
		end
	endgenerate

	assign ss_dout_o = ss_d_global | ss_d_global2 | ss_d_vch | ss_d_ptr0 | ss_d_ptr1
	                 | ss_d_ch[0] | ss_d_ch[1] | ss_d_ch[2] | ss_d_ch[3]
	                 | ss_d_oq[0] | ss_d_oq[1] | ss_d_oq[2] | ss_d_oq[3];

endmodule
