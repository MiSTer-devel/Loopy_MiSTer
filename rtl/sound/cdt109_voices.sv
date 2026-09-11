// Copyright (c) 2026 Jamie Blanks

// The CDT109's sampler: the 32 voice slots, the RAM they live in, and one
// pipeline that walks all of them once per output sample, one slot per cycle.
//
//   A  address the voice RAM
//   B  volume ramp; work out the pitch table index; ask for the sample word
//   C  advance the sample pointer, write the slot back, tell the ROM bus
//      where the pointer went
//   D  (cur - last)
//   E  x fraction
//   F  + last, giving the interpolated sample
//   G  x volume
//   H  x volume slider
//   I  accumulate into the left or right side
//
// Even slots are the left channel and odd slots the right, so a note takes
// two slots. Divisions truncate towards zero, so each arithmetic shift adds
// a bias first. A voice whose wave line has not arrived is frozen for that
// sample.

module cdt109_voices (
	input  wire        clk_i,
	input  wire        rst_i,

	// One pass per output sample.
	input  wire        run_i,
	output wire        busy_o,
	output reg         done_o,
	output reg signed [15:0] out_l_o,
	output reg signed [15:0] out_r_o,

	// Channel context, held by the sequencer.
	input  wire [63:0] vch_i,          // two bits of channel per voice
	input  wire [31:0] bend_i,         // four signed bytes
	input  wire [12:0] slider_a_i,     // channels 1-3
	input  wire [12:0] slider_r_i,     // channel 4

	// The sequencer's window into the voice RAM, open while no pass is running.
	input  wire [4:0]   seq_raddr_i,
	output wire [319:0] seq_q_o,
	input  wire         seq_wr_i,
	input  wire [4:0]   seq_waddr_i,
	input  wire [319:0] seq_wdata_i,

	// The wave ROM bus.
	output wire        smp_sel_o,
	output wire [4:0]  smp_voice_o,
	output wire [1:0]  smp_word_o,
	input  wire        smp_hit_i,
	// Only the top twelve bits of a ROM word are sample data.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [15:0] smp_data_i,
	/* verilator lint_on UNUSEDSIGNAL */
	output reg         upd_o,
	output reg  [4:0]  upd_voice_o,
	output reg  [15:0] upd_line_cur_o,
	output reg  [15:0] upd_line_next_o,
	output reg         upd_cross_o,

	// The pitch increment table, mirrored in the ROM model.
	output wire [10:0] pitch_idx_o,
	input  wire [15:0] pitch_q_i,

	// The savestate walk's byte-wide window onto the voice RAM, at a 64-byte
	// stride: bytes 40-63 of each slot read as zero and ignore writes. A
	// restore holds the synth in reset during the transfer, so the walk has
	// its own reset.
	input  wire        ssb_rst_i,
	input  wire        ssb_rd_i,
	input  wire        ssb_wr_i,
	input  wire [10:0] ssb_addr_i,      // {voice[4:0], byte[5:0]}
	input  wire [7:0]  ssb_wdata_i,
	output reg  [7:0]  ssb_q_o,
	output reg         ssb_ready_o
);
	`include "cdt109_defs.svh"

	// ---- the voice RAM -----------------------------------------------------

	reg          ram_wr;
	reg  [4:0]   ram_waddr;
	reg  [319:0] ram_wdata;
	wire [4:0]   ram_raddr;
	wire [319:0] ram_q;

	cache_ram_dp #(.ADDR_WIDTH (5), .DATA_WIDTH (VOICE_BITS)) u_voices (
		.clk_i     (clk_i),
		.addr_a_i  (ram_waddr),
		.wren_a_i  (ram_wr),
		.wdata_a_i (ram_wdata),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o     (),
		/* verilator lint_on PINCONNECTEMPTY */
		.addr_b_i  (ram_raddr),
		.wren_b_i  (1'b0),
		.wdata_b_i (320'd0),
		.q_b_o     (ram_q)
	);

	// ---- pass control ------------------------------------------------------

	reg  [5:0] slot;          // 0-31 issue a voice, then the pipeline drains
	reg        running;
	assign busy_o = running;

	wire issuing = running & ~slot[5];

	wire ssb_busy;
	assign ram_raddr = issuing  ? slot[4:0]
	                 : ssb_busy ? ssb_addr_i[10:6]
	                            : seq_raddr_i;
	assign seq_q_o   = ram_q;

	// ---- stage B: the slot's state is on ram_q -----------------------------

	reg        b_v;
	reg  [4:0] b_idx;

	// Only a few fields are read here; the rest travel on inside c_word.
	/* verilator lint_off UNUSEDSIGNAL */
	voice_t s;
	/* verilator lint_on UNUSEDSIGNAL */
	assign s = voice_t'(ram_q);

	wire [1:0]        b_ch   = vch_i[{b_idx, 1'b0} +: 2];
	wire signed [7:0] b_bend = $signed(bend_i[{b_ch, 3'd0} +: 8]);

	// Volume ramp: the volume moves, by at most the rate limit, each time the
	// counter reaches the divider.
	wire [9:0]  b_rc   = {1'b0, s.volume_rate_counter} + 10'd1;
	wire        b_due  = (b_rc >= {1'b0, s.volume_rate_div});
	wire [15:0] b_cap  = (s.volume_rate_mul > VOLUME_RATE_LIMIT) ? VOLUME_RATE_LIMIT
	                                                             : s.volume_rate_mul;
	wire signed [17:0] b_down = $signed({2'b00, s.volume}) - $signed({2'b00, b_cap});
	wire        [16:0] b_up   = {1'b0, s.volume} + {1'b0, b_cap};
	wire [15:0] b_ramped = s.volume_down
	        ? (($signed({2'b00, s.volume_target}) > b_down) ? s.volume_target : b_down[15:0])
	        : ((        {1'b0, s.volume_target}   < b_up)   ? s.volume_target : b_up[15:0]);
	wire [15:0] b_volume  = b_due ? b_ramped : s.volume;
	wire [8:0]  b_counter = b_due ? 9'd0 : b_rc[8:0];
	wire        b_playing = (b_volume != 16'd0);

	// Pitch index. The envelope contributes its value divided by sixteen,
	// truncating towards zero.
	wire signed [19:0] b_pev   = s.pitch_env_value;
	/* verilator lint_off UNUSEDSIGNAL */
	wire signed [19:0] b_pev16 = (b_pev + (b_pev[19] ? 20'sd15 : 20'sd0)) >>> 4;
	/* verilator lint_on UNUSEDSIGNAL */
	wire signed [17:0] b_rel   = $signed({{4{s.pitch[13]}}, s.pitch})
	                           + $signed(b_pev16[17:0])
	                           + $signed({{10{b_bend[7]}}, b_bend});
	wire b_clamp_lo = b_rel[17];
	wire b_clamp_hi = ~b_rel[17] & (b_rel > 18'sd1535);
	assign pitch_idx_o = b_clamp_lo ? 11'd0 : (b_clamp_hi ? 11'd1535 : b_rel[10:0]);

	// The ROM bus answers the hit in this cycle and the word in the next.
	assign smp_sel_o   = b_v;
	assign smp_voice_o = b_idx;
	assign smp_word_o  = s.sample_ptr[1:0];

	// ---- stage C: the sample pointer ---------------------------------------

	reg         c_v;
	reg  [4:0]  c_idx;
	reg [319:0] c_word;
	reg [15:0]  c_volume;
	reg  [8:0]  c_counter;
	reg         c_playing;
	reg  [1:0]  c_ch;
	reg         c_hit;

	voice_t cs;
	assign cs = voice_t'(c_word);

	wire [15:0] c_sum   = {1'b0, cs.sample_fract} + pitch_q_i;
	wire        c_over  = c_sum[15];
	wire        c_adv   = c_playing & c_over;
	wire        c_stall = c_adv & ~c_hit;
	wire        c_move  = c_playing & ~c_stall;

	// A ROM word's top twelve bits, less 0x800, which is the same as flipping
	// the sign bit.
	wire signed [11:0] c_raw  = $signed({~smp_data_i[15], smp_data_i[14:4]});
	wire signed [11:0] c_last = c_adv ? cs.sample_cur : cs.sample_last;
	wire c_wrap_dn = (c_last < WRAP_MIN) & (c_raw > WRAP_MAX);
	wire c_wrap_up = (c_last > WRAP_MAX) & (c_raw < WRAP_MIN);
	// -2048 does not fit a 12-bit signed literal, so it is written as its bit
	// pattern.
	wire signed [11:0] c_new = c_wrap_dn ? $signed(12'h800)
	                                     : (c_wrap_up ? 12'sd2047 : c_raw);
	wire signed [11:0] c_cur = c_adv ? c_new : cs.sample_cur;

	wire [18:0] c_ptr1    = cs.sample_ptr + (c_adv ? 19'd1 : 19'd0);
	wire        c_wrapped = c_move & (c_ptr1 > cs.sample_end);
	wire [18:0] c_ptr     = c_move ? (c_wrapped ? cs.sample_loop : c_ptr1) : cs.sample_ptr;
	// The overflow bit is bit 15, so dropping it is the subtraction of 0x8000.
	wire [14:0] c_fract   = c_move ? c_sum[14:0] : cs.sample_fract;

	// The line the pointer is on, and the line it will want next: the loop
	// point when the sample ends inside this line.
	wire [17:0] c_line_last = {c_ptr[17:2], 2'b11};
	// Only the line number is used.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [18:0] c_follow = (cs.sample_end <= {1'b0, c_line_last}) ? cs.sample_loop
	                                                             : ({1'b0, c_line_last} + 19'd1);
	/* verilator lint_on UNUSEDSIGNAL */

	voice_t c_out;
	always_comb begin
		c_out                     = cs;
		c_out.volume              = c_volume;
		c_out.volume_rate_counter = c_counter;
		c_out.sample_fract        = c_fract;
		c_out.sample_ptr          = c_ptr;
		c_out.sample_last         = c_move ? c_last : cs.sample_last;
		c_out.sample_cur          = c_move ? c_cur  : cs.sample_cur;
	end

	// ---- the mix, one multiply per stage -----------------------------------

	reg               d_v;
	reg  [4:0]        d_idx;
	reg signed [11:0] d_last, d_cur;
	reg [14:0]        d_fract;
	reg [15:0]        d_volume;
	reg  [1:0]        d_ch;
	reg               d_contrib;

	reg               e_v;
	reg  [4:0]        e_idx;
	reg signed [12:0] e_diff;
	reg [14:0]        e_fract;
	reg signed [11:0] e_last;
	reg [15:0]        e_volume;
	reg  [1:0]        e_ch;
	reg               e_contrib;

	reg               f_v;
	reg  [4:0]        f_idx;
	reg signed [28:0] f_m1;
	reg signed [11:0] f_last;
	reg [15:0]        f_volume;
	reg  [1:0]        f_ch;
	reg               f_contrib;

	// Each biased product is read from its shift point up.
	/* verilator lint_off UNUSEDSIGNAL */
	wire signed [28:0] f_bias = f_m1 + (f_m1[28] ? 29'sd32767 : 29'sd0);
	wire signed [13:0] f_sd   = $signed(f_bias[28:15]);
	wire signed [14:0] f_s    = $signed({{3{f_last[11]}}, f_last}) + $signed({f_sd[13], f_sd});

	reg               g_v;
	reg  [4:0]        g_idx;
	reg signed [14:0] g_s;
	reg [15:0]        g_volume;
	reg  [1:0]        g_ch;
	reg               g_contrib;

	reg               h_v;
	reg  [4:0]        h_idx;
	reg signed [31:0] h_m2;
	reg  [1:0]        h_ch;
	reg               h_contrib;

	wire signed [31:0] h_bias = h_m2 + (h_m2[31] ? 32'sd65535 : 32'sd0);
	wire signed [15:0] h_s2   = $signed(h_bias[31:16]);
	wire [12:0] h_slider = (h_ch == 2'd3) ? slider_r_i : slider_a_i;

	reg               i_v;
	reg  [4:0]        i_idx;
	reg signed [29:0] i_m3;
	reg signed [15:0] i_s2;
	reg  [1:0]        i_ch;
	reg               i_contrib;

	wire signed [29:0] i_bias = i_m3 + (i_m3[29] ? 30'sd4095 : 30'sd0);
	wire signed [17:0] i_s3   = $signed(i_bias[29:12]);
	/* verilator lint_on UNUSEDSIGNAL */
	wire signed [17:0] i_val  = (i_ch == 2'd0) ? $signed({{2{i_s2[15]}}, i_s2}) : i_s3;

	reg signed [23:0] acc_l, acc_r;
	wire signed [23:0] i_ext = $signed({{6{i_val[17]}}, i_val});

	function automatic signed [15:0] clamp16(input signed [23:0] v);
		if (v >  24'sd32767)      clamp16 =  16'sd32767;
		else if (v < -24'sd32767) clamp16 = -16'sd32767;
		else                      clamp16 = v[15:0];
	endfunction

	// ---- the savestate window ----------------------------------------------
	//
	// A read is two cycles for the RAM and one to pick the byte out; a write
	// is the same plus a write back of the whole 320-bit slot.

	localparam [2:0] SB_IDLE = 3'd0, SB_R1 = 3'd1, SB_R2 = 3'd2,
	                 SB_W1 = 3'd3, SB_W2 = 3'd4, SB_W3 = 3'd5;
	reg [2:0] sb;
	assign ssb_busy = (sb != SB_IDLE);

	wire [8:0] ssb_bit = {ssb_addr_i[5:0], 3'd0};
	wire       ssb_in_word = (ssb_addr_i[5:0] < 6'd40);

	// Replace one byte of the slot. Forty lane decodes instead of a 320-bit
	// barrel shifter.
	function automatic [319:0] replace_byte(input [319:0] word, input [5:0] idx,
	                                        input [7:0] b);
		begin
			replace_byte = word;
			for (int j = 0; j < 40; j++)
				if (idx == 6'(j)) replace_byte[j * 8 +: 8] = b;
		end
	endfunction

	// ---- the pass ----------------------------------------------------------

	always @(posedge clk_i) begin
		if (rst_i) begin
			slot <= 6'd0; running <= 1'b0; done_o <= 1'b0;
			b_v <= 1'b0; c_v <= 1'b0; d_v <= 1'b0; e_v <= 1'b0;
			f_v <= 1'b0; g_v <= 1'b0; h_v <= 1'b0; i_v <= 1'b0;
			b_idx <= 5'd0; c_idx <= 5'd0; d_idx <= 5'd0; e_idx <= 5'd0;
			f_idx <= 5'd0; g_idx <= 5'd0; h_idx <= 5'd0; i_idx <= 5'd0;
			acc_l <= 24'sd0; acc_r <= 24'sd0;
			out_l_o <= 16'sd0; out_r_o <= 16'sd0;
			ram_wr <= 1'b0; ram_waddr <= 5'd0; ram_wdata <= 320'd0;
			upd_o <= 1'b0; upd_voice_o <= 5'd0;
			upd_line_cur_o <= 16'd0; upd_line_next_o <= 16'd0; upd_cross_o <= 1'b0;
			c_word <= 320'd0; c_volume <= 16'd0; c_counter <= 9'd0;
			c_playing <= 1'b0; c_ch <= 2'd0; c_hit <= 1'b0;
			d_last <= 12'sd0; d_cur <= 12'sd0; d_fract <= 15'd0;
			d_volume <= 16'd0; d_ch <= 2'd0; d_contrib <= 1'b0;
			e_diff <= 13'sd0; e_fract <= 15'd0; e_last <= 12'sd0;
			e_volume <= 16'd0; e_ch <= 2'd0; e_contrib <= 1'b0;
			f_m1 <= 29'sd0; f_last <= 12'sd0; f_volume <= 16'd0;
			f_ch <= 2'd0; f_contrib <= 1'b0;
			g_s <= 15'sd0; g_volume <= 16'd0; g_ch <= 2'd0; g_contrib <= 1'b0;
			h_m2 <= 32'sd0; h_ch <= 2'd0; h_contrib <= 1'b0;
			i_m3 <= 30'sd0; i_s2 <= 16'sd0; i_ch <= 2'd0; i_contrib <= 1'b0;
		end else begin
			done_o      <= 1'b0;
			upd_o       <= 1'b0;

			// -- pass sequencing. The sequencer owns the write port between
			//    passes and the sampler owns it during one.
			if (!running) begin
				ram_wr    <= seq_wr_i;
				ram_waddr <= seq_waddr_i;
				ram_wdata <= seq_wdata_i;
				if (run_i) begin
					running <= 1'b1;
					slot    <= 6'd0;
					acc_l   <= 24'sd0;
					acc_r   <= 24'sd0;
				end
			end else begin
				ram_wr <= 1'b0;
				if (slot != 6'd63) slot <= slot + 6'd1;
			end

			// -- A -> B
			b_v   <= issuing;
			b_idx <= slot[4:0];

			// -- B -> C
			c_v       <= b_v;
			c_idx     <= b_idx;
			c_word    <= ram_q;
			c_volume  <= b_volume;
			c_counter <= b_counter;
			c_playing <= b_playing;
			c_ch      <= b_ch;
			c_hit     <= smp_hit_i;

			// -- C: write the slot back and say where the pointer went
			if (c_v) begin
				ram_wr    <= 1'b1;
				ram_waddr <= c_idx;
				ram_wdata <= c_out;

				upd_o           <= 1'b1;
				upd_voice_o     <= c_idx;
				upd_line_cur_o  <= c_ptr[17:2];
				upd_line_next_o <= c_follow[17:2];
				upd_cross_o     <= (c_ptr[17:2] != cs.sample_ptr[17:2]);
			end

			// -- C -> D
			d_v       <= c_v;
			d_idx     <= c_idx;
			d_last    <= c_out.sample_last;
			d_cur     <= c_out.sample_cur;
			d_fract   <= c_out.sample_fract;
			d_volume  <= c_volume;
			d_ch      <= c_ch;
			d_contrib <= (c_volume != 16'd0);

			// -- D -> E
			e_v       <= d_v;
			e_idx     <= d_idx;
			e_diff    <= $signed({d_cur[11], d_cur}) - $signed({d_last[11], d_last});
			e_fract   <= d_fract;
			e_last    <= d_last;
			e_volume  <= d_volume;
			e_ch      <= d_ch;
			e_contrib <= d_contrib;

			// -- E -> F
			f_v       <= e_v;
			f_idx     <= e_idx;
			f_m1      <= e_diff * $signed({1'b0, e_fract});
			f_last    <= e_last;
			f_volume  <= e_volume;
			f_ch      <= e_ch;
			f_contrib <= e_contrib;

			// -- F -> G
			g_v       <= f_v;
			g_idx     <= f_idx;
			g_s       <= f_s;
			g_volume  <= f_volume;
			g_ch      <= f_ch;
			g_contrib <= f_contrib;

			// -- G -> H
			h_v       <= g_v;
			h_idx     <= g_idx;
			h_m2      <= g_s * $signed({1'b0, g_volume});
			h_ch      <= g_ch;
			h_contrib <= g_contrib;

			// -- H -> I
			i_v       <= h_v;
			i_idx     <= h_idx;
			i_m3      <= h_s2 * $signed({1'b0, h_slider});
			i_s2      <= h_s2;
			i_ch      <= h_ch;
			i_contrib <= h_contrib;

			// -- I: accumulate, and finish the pass on the last slot
			if (i_v && i_contrib) begin
				if (i_idx[0]) acc_r <= acc_r + i_ext;
				else          acc_l <= acc_l + i_ext;
			end
			if (i_v && (i_idx == 5'd31)) begin
				// Slot 31 is odd, so only the right side still owes an add.
				out_l_o <= clamp16(acc_l);
				out_r_o <= clamp16(i_contrib ? acc_r + i_ext : acc_r);
				done_o  <= 1'b1;
				running <= 1'b0;
				slot    <= 6'd0;
			end
		end

		// -- the savestate walk. Placed after the reset branch because a restore
		//    holds the synth in reset while these bytes go in. It only runs
		//    with no pass in flight.
		if (ssb_rst_i) begin
			sb          <= SB_IDLE;
			ssb_q_o     <= 8'd0;
			ssb_ready_o <= 1'b0;
		end else begin
			ssb_ready_o <= 1'b0;

			case (sb)
			SB_IDLE: if (!running) begin
				if      (ssb_rd_i) sb <= SB_R1;
				else if (ssb_wr_i) sb <= SB_W1;
			end
			SB_R1: sb <= SB_R2;
			SB_R2: begin
				ssb_q_o     <= ssb_in_word ? ram_q[ssb_bit +: 8] : 8'd0;
				ssb_ready_o <= 1'b1;
				sb          <= SB_IDLE;
			end
			SB_W1: sb <= SB_W2;
			SB_W2: sb <= SB_W3;
			SB_W3: begin
				ssb_ready_o <= 1'b1;
				sb          <= SB_IDLE;
			end
			default: sb <= SB_IDLE;
			endcase

			if (sb == SB_W2 && !running) begin
				ram_wr    <= 1'b1;
				ram_waddr <= ssb_addr_i[10:6];
				ram_wdata <= replace_byte(ram_q, ssb_addr_i[5:0], ssb_wdata_i);
			end
		end
	end

endmodule
