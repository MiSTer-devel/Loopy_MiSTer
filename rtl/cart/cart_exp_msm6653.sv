// Copyright (c) 2026 Jamie Blanks

// OKI MSM6653A-457, the ADPCM speech chip on the Wanwan Aijou Monogatari
// cartridge. Modelled from the 1994 OKI data book (microcomputer interface
// mode) and from the game's own driver.
//
// The chip resolves a phrase code inside its own 544 Kbit mask ROM, which is
// undumped, so phrase lookup is an external port: the model asks for a phrase
// code and is handed a byte range to read. The pin protocol, command decode,
// both channels, repeat, fade-out, beep, silence, STOP, NAR and BUSY follow
// the data book. Only ADPCM is decoded; the family's PCM and melody methods
// and per-phrase sampling rate live in the ROM, so the bank reports one rate.
//
// Pin contract, as wired on the cart:
//
//   RESET  PB1   L puts the chip in standby and restores the power-on options
//   CMD    PB5   L while ST is low means the latched byte is a command
//   ST     PB10  speech starts on the fall, I6-I0 are captured on the rise
//   CH     PB8   H selects channel 1, L selects channel 2
//   I6-I0        the 74HC273A latch on D6-D0, modelled outside this file
//   NAR    PB7   H when the address/command register will accept a new ST
//   BUSY         L while either channel is vocalizing (not wired on this cart)
//   AOUT         12-bit signed, 0 = 1/2 VDD, DAC or LPF by the oa option
//
// Everything that advances with time (both voices, the NAR windows, the
// output filter) runs on ce_osc, the 4.096 MHz cart resonator. The pin
// synchronisers and the address/command register run on clk_sys: on the real
// part the CPU's edges are asynchronous to the oscillator and the I6-I0
// register is clocked by ST itself.

module cart_exp_msm6653 #(
	// 4.096 MHz from clk_sys = 2835/88 MHz:  4.096 / (2835/88) = 45056 / 354375.
	parameter int OSC_CE_NUM   = 45056,
	parameter int OSC_CE_DEN   = 354375,
	// Simulation only. Divides every sample period, silence tick and beep
	// period by 2^N; sample values are untouched.
	parameter int FS_DIV_SHIFT = 0,
	// Silence and beep duration unit. The data book's 16.384 ms is 2^16
	// periods of a 4.000 MHz oscillator; this cart runs at 4.096 MHz, where the
	// same counter gives 16.0 ms and the beep frequencies come out exact.
	parameter int TICK_DIV     = 65536
) (
	input  logic               clk_sys,

	input  logic               reset_pin_n,   // RESET, active low
	input  logic               cmd_pin,       // CMD
	input  logic               st_pin,        // ST
	input  logic               ch_pin,        // CH, 1 = channel 1
	input  logic [6:0]         id_pins,       // I6-I0 from the external latch

	output logic               nar,
	output logic               busy_n,

	output logic signed [11:0] aout,

	// Phrase lookup. The mask ROM's own table is undumped, so this leaves the
	// chip. phrase_len == 0 means "not a user phrase".
	output logic [6:0]         phrase_req,
	output logic               phrase_ch,     // voice index, 0 = channel 1
	output logic               phrase_req_stb,
	input  logic [23:0]        phrase_start,
	input  logic [23:0]        phrase_len,
	input  logic [2:0]         phrase_rate,   // sampling-rate code
	input  logic               phrase_valid,

	// Speech data read port.  rom_req marks a byte the model is waiting for,
	// rom_ch says which voice is asking, and rom_ready means rom_data belongs
	// to rom_addr.
	output logic               rom_req,
	output logic               rom_ch,
	output logic [23:0]        rom_addr,
	input  logic [7:0]         rom_data,
	input  logic               rom_ready
);

	// ------------------------------------------------------------ declarations
	logic [1:0] rst_sync, st_sync, cmd_sync, ch_sync;

	logic       cap_is_cmd;    // CMD level sampled at the fall of ST
	logic       cap_ch;        // CH   level sampled at the fall of ST
	logic [6:0] cap_data;      // I6-I0 sampled at the rise of ST
	logic       cap_pending;
	logic       cap_taken;

	logic       opt_oa;        // 1 = AOUT is the raw DAC, 0 = through the LPF
	logic       opt_ov;        // 1 = 1/4..3/4 VDD, i.e. half amplitude

	logic [1:0] pend_class;    // what the next address byte will start
	logic       ctrl_sm;
	logic [1:0] ctrl_rp;
	logic [1:0] ctrl_vl;
	logic [1:0] beep_bl;
	logic [1:0] beep_bf;

	logic        cstate;
	logic [15:0] nar_timer [0:1];
	logic        start_ch;
	logic [6:0]  start_units;
	logic [23:0] found_start, found_len;
	logic [2:0]  found_rate;

	logic        v_start_speech [0:1];
	logic        v_start_sil    [0:1];
	logic        v_start_beep   [0:1];
	logic        v_stop         [0:1];

	logic               v_active   [0:1];
	logic [2:0]         v_rate     [0:1];
	logic               v_drives   [0:1];
	logic signed [11:0] v_out      [0:1];
	logic               v_rom_req  [0:1];
	logic [23:0]        v_rom_addr [0:1];
	logic               v_rom_rdy  [0:1];

	// -------------------------------------------------------------- pin_sync
	always_ff @(posedge clk_sys) begin
		rst_sync <= {rst_sync[0], reset_pin_n};
		st_sync  <= {st_sync[0],  st_pin};
		cmd_sync <= {cmd_sync[0], cmd_pin};
		ch_sync  <= {ch_sync[0],  ch_pin};
	end

	// The part has an internal power-on reset.  On the FPGA the same job falls
	// to the power-up state of the registers, so hold reset for the first
	// 128 fabric clocks and let every register below take its idle value.
	/* verilator lint_off PROCASSINIT */
	logic [7:0] por_cnt = 8'd0;
	/* verilator lint_on PROCASSINIT */

	always_ff @(posedge clk_sys)
		if (!por_cnt[7]) por_cnt <= por_cnt + 8'd1;

	wire chip_rst = ~rst_sync[1] | ~por_cnt[7];
	wire st_fall  =  st_sync[1] & ~st_sync[0];
	wire st_rise  = ~st_sync[1] &  st_sync[0];

	// ---------------------------------------------------------------- clocking
	// 4.096 MHz enable, a rational phase accumulator.
	localparam int ACC_W = $clog2(OSC_CE_DEN) + 1;

	logic [ACC_W-1:0] osc_acc;
	logic             ce_osc;

	wire [ACC_W-1:0] osc_sum = osc_acc + ACC_W'(OSC_CE_NUM);

	// RESET stops oscillation on the real part, so the accumulator stops here.
	always_ff @(posedge clk_sys) begin
		if (chip_rst) begin
			osc_acc <= '0;
			ce_osc  <= 1'b0;
		end else if (osc_sum >= ACC_W'(OSC_CE_DEN)) begin
			osc_acc <= osc_sum - ACC_W'(OSC_CE_DEN);
			ce_osc  <= 1'b1;
		end else begin
			osc_acc <= osc_sum;
			ce_osc  <= 1'b0;
		end
	end

	// Every sampling rate is an exact division of 4.096 MHz, which is why the
	// cart carries that resonator.
	function automatic [11:0] rate_div(input [2:0] rate);
		case (rate)
		3'd0:    rate_div = 12'd1024;   //  4.0 kHz
		3'd1:    rate_div = 12'd768;    //  5.3 kHz
		3'd2:    rate_div = 12'd640;    //  6.4 kHz
		3'd3:    rate_div = 12'd512;    //  8.0 kHz
		3'd4:    rate_div = 12'd384;    // 10.6 kHz
		3'd5:    rate_div = 12'd320;    // 12.8 kHz
		3'd6:    rate_div = 12'd256;    // 16.0 kHz
		default: rate_div = 12'd128;    // 32.0 kHz
		endcase
	endfunction

	// ------------------------------------------------------- sample clock
	// One sample clock for the whole part. The data book gives channel 1's
	// sampling frequency priority when both play (9), and a beep on channel 1
	// forces 8 kHz so channel 2 speech "may be either too slow or too fast"
	// (6.3). That wrong speed is the real behaviour.
	wire [2:0] fs_rate = v_drives[0] ? v_rate[0]
	                   : v_drives[1] ? v_rate[1]
	                   :               3'd3;      // idle: 8 kHz

	wire [11:0] fs_div = rate_div(fs_rate) >> FS_DIV_SHIFT;

	logic [11:0] fs_cnt;

	// Combinational, so the sample tick lands on the same clk_sys cycle as the
	// oscillator tick that produced it; the voices advance on `ce_osc && fs_ce`.
	wire fs_ce = ce_osc && (fs_cnt <= 12'd1);

	always_ff @(posedge clk_sys) begin
		if (chip_rst) begin
			fs_cnt <= 12'd1;
		end else if (ce_osc) begin
			fs_cnt <= (fs_cnt <= 12'd1) ? fs_div : fs_cnt - 12'd1;
		end
	end

	// ------------------------------------------------------------ cmd_capture
	// The pins decide what the byte is, never the byte value.  0000000 with
	// CMD low is an option-setting command; the same byte with CMD high is the
	// stop code.
	always_ff @(posedge clk_sys) begin
		if (chip_rst) begin
			cap_is_cmd  <= 1'b0;
			cap_ch      <= 1'b1;
			cap_data    <= 7'h00;
			cap_pending <= 1'b0;
		end else begin
			if (st_fall) begin
				cap_is_cmd <= ~cmd_sync[1];
				cap_ch     <=  ch_sync[1];
			end
			if (st_rise) begin
				cap_data    <= id_pins;
				cap_pending <= 1'b1;
			end else if (cap_taken) begin
				cap_pending <= 1'b0;
			end
		end
	end

	// -------------------------------------------------- cmd_decode, controller
	// Table 6.1.  I6:I5 picks the class, and the class also decides what the
	// next address byte means.
	localparam logic [1:0] CLS_OPTION  = 2'b00;
	localparam logic [1:0] CLS_SILENCE = 2'b01;
	localparam logic [1:0] CLS_BEEP    = 2'b10;
	localparam logic [1:0] CLS_SPEECH  = 2'b11;

	localparam logic C_IDLE   = 1'b0;
	localparam logic C_LOOKUP = 1'b1;

	// NAR is per channel: whether that channel's I6-I0 register is open, with
	// the CH pin picking which one the pin reports. The data book's NAR times
	// (350/375/400 us at 8 kHz) are three sample periods, so the window
	// follows that channel's fs and falls back to 8 kHz when idle.
	wire [11:0] nar_fs_div = fs_div;

	wire nar_sel = ch_sync[1] ? 1'b0 : 1'b1;

	always_ff @(posedge clk_sys) begin
		cap_taken         <= 1'b0;
		phrase_req_stb    <= 1'b0;
		v_start_speech[0] <= 1'b0;
		v_start_speech[1] <= 1'b0;
		v_start_sil[0]    <= 1'b0;
		v_start_sil[1]    <= 1'b0;
		v_start_beep[0]   <= 1'b0;
		v_start_beep[1]   <= 1'b0;
		v_stop[0]         <= 1'b0;
		v_stop[1]         <= 1'b0;

		if (chip_rst) begin
			cstate      <= C_IDLE;
			opt_oa      <= 1'b0;     // Table 4.2: LPF output
			opt_ov      <= 1'b0;     // full amplitude
			pend_class  <= CLS_SPEECH;  // the power-on command is 1100000
			ctrl_sm     <= 1'b0;
			ctrl_rp     <= 2'b00;
			ctrl_vl     <= 2'b00;
			beep_bl     <= 2'b00;
			beep_bf     <= 2'b00;
			nar_timer[0] <= 16'd0;
			nar_timer[1] <= 16'd0;
			start_ch    <= 1'b1;
			start_units <= 7'd0;
			phrase_req  <= 7'd0;
			phrase_ch   <= 1'b0;
			found_start <= 24'd0;
			found_len   <= 24'd0;
			found_rate  <= 3'd3;
		end else begin
			if (ce_osc) begin
				if (nar_timer[0] != 16'd0) nar_timer[0] <= nar_timer[0] - 16'd1;
				if (nar_timer[1] != 16'd0) nar_timer[1] <= nar_timer[1] - 16'd1;
			end

			case (cstate)
			C_IDLE:
				if (cap_pending) begin
					cap_taken <= 1'b1;
					nar_timer[cap_ch ? 0 : 1] <= {4'd0, nar_fs_div}
					                           + {4'd0, nar_fs_div}
					                           + {4'd0, nar_fs_div};
					if (cap_is_cmd) begin
						case (cap_data[6:5])
						CLS_OPTION: begin
							// An option leaves pend_class alone. The data book
							// tells the driver to re-issue a speech, silence or
							// beep command afterwards, which reads as advice
							// rather than the chip forgetting.
							opt_oa <= cap_data[3];
							opt_ov <= cap_data[0];
						end
						CLS_SILENCE: begin
							pend_class <= CLS_SILENCE;
						end
						CLS_BEEP: begin
							pend_class <= CLS_BEEP;
							beep_bl    <= cap_data[3:2];
							beep_bf    <= cap_data[1:0];
						end
						default: begin
							pend_class <= CLS_SPEECH;
							ctrl_sm    <= cap_data[4];
							ctrl_rp    <= cap_data[3:2];
							ctrl_vl    <= cap_data[1:0];
						end
						endcase
					end else begin
						start_ch    <= cap_ch;
						start_units <= cap_data;
						if (cap_data == 7'h00) begin
							// Stop code, addressed by CH like everything else.
							v_stop[cap_ch ? 0 : 1] <= 1'b1;
						end else if (pend_class == CLS_SPEECH) begin
							phrase_req     <= cap_data;
							phrase_ch      <= ~cap_ch;
							phrase_req_stb <= 1'b1;
							cstate         <= C_LOOKUP;
						end else if (pend_class == CLS_BEEP) begin
							// The beep generator exists in channel 1 only.
							v_start_beep[0] <= 1'b1;
						end else begin
							v_start_sil[cap_ch ? 0 : 1] <= 1'b1;
						end
					end
				end

			default:
				if (phrase_valid) begin
					found_start <= phrase_start;
					found_len   <= phrase_len;
					found_rate  <= phrase_rate;
					// A code with no phrase behind it vocalizes nothing; the
					// data book's Figure 7.2 shows NAR pulsing and AOUT
					// settling to 1/2 VDD.
					if (phrase_len != 24'd0)
						v_start_speech[start_ch ? 0 : 1] <= 1'b1;
					cstate <= C_IDLE;
				end
			endcase
		end
	end

	assign nar = (nar_timer[nar_sel] == 16'd0);

	// ------------------------------------------------------------------ voices
	// One speech-data read port, time-shared.  Channel 1 wins; each channel
	// needs one byte per two samples, so channel 2 cannot starve.
	wire gnt0 = v_rom_req[0];
	wire gnt1 = v_rom_req[1] & ~v_rom_req[0];

	assign rom_req      = gnt0 | gnt1;
	assign rom_ch       = ~gnt0;
	assign rom_addr     = gnt0 ? v_rom_addr[0] : v_rom_addr[1];
	assign v_rom_rdy[0] = gnt0 & rom_ready;
	assign v_rom_rdy[1] = gnt1 & rom_ready;

	generate
		genvar c;
		for (c = 0; c < 2; c = c + 1) begin : chan
			msm6653_voice #(
				.FS_DIV_SHIFT (FS_DIV_SHIFT),
				.TICK_DIV     (TICK_DIV),
				.IS_CH1       (c == 0)
			) u_voice (
				.clk_sys       (clk_sys),
				.ce_osc        (ce_osc),
				.rst           (chip_rst),

				.start_speech  (v_start_speech[c]),
				.start_silence (v_start_sil[c]),
				.start_beep    (v_start_beep[c]),
				.stop          (v_stop[c]),

				.phrase_start  (found_start),
				.phrase_len    (found_len),
				.phrase_rate   (found_rate),
				.units         (start_units),

				.ctrl_sm       (ctrl_sm),
				.ctrl_rp       (ctrl_rp),
				.ctrl_vl       (ctrl_vl),
				.beep_bl       (beep_bl),
				.beep_bf       (beep_bf),

				.rom_req       (v_rom_req[c]),
				.rom_addr      (v_rom_addr[c]),
				.rom_data      (rom_data),
				.rom_ready     (v_rom_rdy[c]),

				.fs_ce         (fs_ce),
				.active        (v_active[c]),
				.rate_code     (v_rate[c]),
				.rate_drives   (v_drives[c]),
				.sample        (v_out[c])
			);
		end
	endgenerate

	assign busy_n = ~(v_active[0] | v_active[1]);

	// ------------------------------------------------------------------- mixer
	// Both channels already carry their own attenuation; only the ov amplitude
	// option and the 12-bit rail remain. Two channels at full scale sum past
	// the rail and clip, as the note under Table 12.1 warns.
	wire signed [12:0] mix_sum = {v_out[0][11], v_out[0]} + {v_out[1][11], v_out[1]};
	wire signed [12:0] mix_amp = opt_ov ? (mix_sum >>> 1) : mix_sum;

	wire signed [11:0] dac = (mix_amp >  13'sd2047) ? 12'sh7FF   //  2047
	                       : (mix_amp < -13'sd2048) ? 12'sh800   // -2048
	                       :  mix_amp[11:0];

	// --------------------------------------------------------------------- lpf
	// The real filter is a switched-capacitor -40 dB/oct lowpass with cutoff
	// about 0.4 x fs (Table 12.2). It smooths the analog staircase, so this
	// runs seven one-pole sections at the 4.096 MHz oscillator rather than at
	// fs. Each section is y += (x - y) >> k: shift and add only.
	function automatic [2:0] lpf_shift(input [2:0] rate);
		case (rate)
		3'd0, 3'd1, 3'd2: lpf_shift = 3'd7;   //  4.0 /  5.3 /  6.4 kHz
		3'd3, 3'd4, 3'd5: lpf_shift = 3'd6;   //  8.0 / 10.6 / 12.8 kHz
		3'd6:             lpf_shift = 3'd5;   // 16.0 kHz
		default:          lpf_shift = 3'd4;   // 32.0 kHz
		endcase
	endfunction

	// Twelve fraction bits: each section's shift leaves a residue of up to
	// 2^k-1 below its input, and seven of those in series have to stay well
	// under half an output LSB or silence would sit at -1 forever.
	localparam int LPF_FRAC  = 12;
	localparam int LPF_W     = 14 + LPF_FRAC;
	localparam int LPF_ORDER = 7;

	wire [2:0] lpf_k_raw = lpf_shift(fs_rate);
	wire [2:0] lpf_k     = (lpf_k_raw > 3'(FS_DIV_SHIFT)) ? lpf_k_raw - 3'(FS_DIV_SHIFT)
	                                                      : 3'd1;

	logic signed [LPF_W-1:0] lpf_y [0:LPF_ORDER-1];
	wire  signed [LPF_W-1:0] lpf_in [0:LPF_ORDER-1];

	wire signed [LPF_W-1:0] dac_ext = $signed({{(LPF_W-12){dac[11]}}, dac}) <<< LPF_FRAC;

	generate
		genvar p;
		for (p = 0; p < LPF_ORDER; p = p + 1) begin : lpf_stage
			if (p == 0) begin : gen_first
				assign lpf_in[p] = dac_ext;
			end else begin : gen_next
				assign lpf_in[p] = lpf_y[p - 1];
			end

			always_ff @(posedge clk_sys) begin
				if (chip_rst)    lpf_y[p] <= '0;
				else if (ce_osc) lpf_y[p] <= lpf_y[p] + ((lpf_in[p] - lpf_y[p]) >>> lpf_k);
			end
		end
	endgenerate

	// Round rather than truncate on the way out.  Each section's shift leaves
	// a residue of well under half an output LSB, and flooring would turn that
	// into a permanent -1 offset once the signal has run down to silence.
	wire signed [11:0] lpf_out =
		12'((lpf_y[LPF_ORDER-1] + LPF_W'(1 <<< (LPF_FRAC - 1))) >>> LPF_FRAC);

	// AOUT source, the oa option. Under RESET the real pin is pulled to GND;
	// the model idles at 1/2 VDD, since a full-scale DC step into the MiSTer
	// mixer is a click with no cart op-amp to absorb it.
	always_comb begin
		if (chip_rst) aout = 12'sd0;
		else          aout = opt_oa ? dac : lpf_out;
	end

endmodule


// One of the chip's two phrase engines: address pointer, nibble phase, ADPCM
// state, repeat counter, fade step, plus the silence and beep generators.
module msm6653_voice #(
	parameter int FS_DIV_SHIFT = 0,
	parameter int TICK_DIV     = 65536,
	parameter bit IS_CH1       = 1
) (
	input  logic               clk_sys,
	input  logic               ce_osc,
	input  logic               rst,

	input  logic               start_speech,
	input  logic               start_silence,
	input  logic               start_beep,
	input  logic               stop,

	input  logic [23:0]        phrase_start,
	input  logic [23:0]        phrase_len,
	input  logic [2:0]         phrase_rate,
	input  logic [6:0]         units,

	input  logic               ctrl_sm,
	input  logic [1:0]         ctrl_rp,
	input  logic [1:0]         ctrl_vl,
	input  logic [1:0]         beep_bl,
	input  logic [1:0]         beep_bf,

	output logic               rom_req,
	output logic [23:0]        rom_addr,
	input  logic [7:0]         rom_data,
	input  logic               rom_ready,

	input  logic               fs_ce,        // shared sample clock, see below
	output logic               active,
	output logic [2:0]         rate_code,    // this voice's own sampling rate
	output logic               rate_drives,  // ... and whether it sets the pace
	output logic signed [11:0] sample
);

	// step[n] = floor(16 * 1.1^n), the OKI/Dialogic 49-entry table. Assumed
	// for this part.
	function automatic [10:0] step_size(input [5:0] n);
		case (n)
		6'd0 : step_size = 11'd16  ;
		6'd1 : step_size = 11'd17  ;
		6'd2 : step_size = 11'd19  ;
		6'd3 : step_size = 11'd21  ;
		6'd4 : step_size = 11'd23  ;
		6'd5 : step_size = 11'd25  ;
		6'd6 : step_size = 11'd28  ;
		6'd7 : step_size = 11'd31  ;
		6'd8 : step_size = 11'd34  ;
		6'd9 : step_size = 11'd37  ;
		6'd10: step_size = 11'd41  ;
		6'd11: step_size = 11'd45  ;
		6'd12: step_size = 11'd50  ;
		6'd13: step_size = 11'd55  ;
		6'd14: step_size = 11'd60  ;
		6'd15: step_size = 11'd66  ;
		6'd16: step_size = 11'd73  ;
		6'd17: step_size = 11'd80  ;
		6'd18: step_size = 11'd88  ;
		6'd19: step_size = 11'd97  ;
		6'd20: step_size = 11'd107 ;
		6'd21: step_size = 11'd118 ;
		6'd22: step_size = 11'd130 ;
		6'd23: step_size = 11'd143 ;
		6'd24: step_size = 11'd157 ;
		6'd25: step_size = 11'd173 ;
		6'd26: step_size = 11'd190 ;
		6'd27: step_size = 11'd209 ;
		6'd28: step_size = 11'd230 ;
		6'd29: step_size = 11'd253 ;
		6'd30: step_size = 11'd279 ;
		6'd31: step_size = 11'd307 ;
		6'd32: step_size = 11'd337 ;
		6'd33: step_size = 11'd371 ;
		6'd34: step_size = 11'd408 ;
		6'd35: step_size = 11'd449 ;
		6'd36: step_size = 11'd494 ;
		6'd37: step_size = 11'd544 ;
		6'd38: step_size = 11'd598 ;
		6'd39: step_size = 11'd658 ;
		6'd40: step_size = 11'd724 ;
		6'd41: step_size = 11'd796 ;
		6'd42: step_size = 11'd876 ;
		6'd43: step_size = 11'd963 ;
		6'd44: step_size = 11'd1060;
		6'd45: step_size = 11'd1166;
		6'd46: step_size = 11'd1282;
		6'd47: step_size = 11'd1411;
		6'd48: step_size = 11'd1552;
		default: step_size = 11'd1552;
		endcase
	endfunction


	// Half-period of each beep tone in oscillator ticks: 4.096 MHz / 2f.
	function automatic [12:0] beep_half(input [1:0] bf);
		case (bf)
		2'd0:    beep_half = 13'd4096;  // 0.5 kHz
		2'd1:    beep_half = 13'd2048;  // 1.0 kHz
		2'd2:    beep_half = 13'd1280;  // 1.6 kHz
		default: beep_half = 13'd1024;  // 2.0 kHz
		endcase
	endfunction

	// Beep amplitude as a fraction of channel 1 full scale (Table 6.3). The 1/3
	// setting is why these are constants rather than shifts. The vl attenuation
	// does not stack on it (Table 12.1 lists the beep only as 1/2, 1/3, 1/4,
	// 1/8); the ov option still halves it.
	function automatic signed [11:0] beep_amp(input [1:0] bl);
		case (bl)
		2'd0:    beep_amp = 12'sd256;   // 1/8
		2'd1:    beep_amp = 12'sd512;   // 1/4
		2'd2:    beep_amp = 12'sd682;   // 1/3
		default: beep_amp = 12'sd1024;  // 1/2
		endcase
	endfunction

	localparam logic [1:0] V_IDLE = 2'd0;
	localparam logic [1:0] V_PLAY = 2'd1;
	localparam logic [1:0] V_SIL  = 2'd2;
	localparam logic [1:0] V_BEEP = 2'd3;

	localparam logic signed [11:0] SIG_MAX = 12'sh7FF;   //  2047
	localparam logic signed [11:0] SIG_MIN = 12'sh800;   // -2048

	logic [1:0]  state;
	logic [23:0] base, len_hold, ptr, bytes_left;
	logic        nib_hi;
	logic [7:0]  cur_byte;
	logic        byte_ok;

	logic signed [11:0] sig;
	logic [5:0]         step;
	logic [2:0]         rate_l;

	logic [15:0] tick_cnt;
	logic [6:0]  units_left;

	logic [12:0] beep_cnt;
	logic        beep_lvl;
	logic        zero_next;    // let the last sample stand for its own period

	logic        rep_inf;
	logic [2:0]  plays_left;
	logic [1:0]  fade;
	logic        sm_l;
	logic [1:0]  vl_l;
	logic [1:0]  bl_l, bf_l;

	localparam logic [15:0] TICK_RELOAD = 16'(TICK_DIV >> FS_DIV_SHIFT) - 16'd1;

	// The fade-out walks the same 1, 1/2, 1/4, 1/8 ladder as the attenuation
	// and stops at 1/8, so both together are one shift amount.
	wire [2:0] att_sum = {1'b0, vl_l} + {1'b0, fade};
	wire [1:0] att     = !sm_l              ? vl_l
	                   : (att_sum > 3'd3)   ? 2'd3
	                   :                      att_sum[1:0];

	// ADPCM: diff = sign * (v*b1 + v/2*b2 + v/4*b3 + v/8), reconstruction
	// clamped to 12 bits; the step pointer moves by {-1,-1,-1,-1,2,4,6,8}.
	wire [3:0]  nib = nib_hi ? cur_byte[7:4] : cur_byte[3:0];
	wire [10:0] v   = step_size(step);

	wire [12:0] mag = (nib[2] ? {2'b0, v}        : 13'd0)
	                + (nib[1] ? {3'b0, v[10:1]}  : 13'd0)
	                + (nib[0] ? {4'b0, v[10:2]}  : 13'd0)
	                +           {5'b0, v[10:3]};

	wire signed [13:0] sig_ext  = {{2{sig[11]}}, sig};
	wire signed [13:0] mag_ext  = {1'b0, mag};
	wire signed [13:0] next_sig = nib[3] ? sig_ext - mag_ext : sig_ext + mag_ext;

	wire signed [11:0] new_sig = (next_sig > 14'sd2047)  ? SIG_MAX
	                           : (next_sig < -14'sd2048) ? SIG_MIN
	                           :  next_sig[11:0];

	// {-1,-1,-1,-1,2,4,6,8} is "-1 below 4, else 2 + 2 x the low two bits".
	wire signed [7:0] step_next = nib[2] ? $signed({2'b0, step}) + $signed({4'b0, nib[1:0], 1'b0}) + 8'sd2
	                                     : $signed({2'b0, step}) - 8'sd1;

	// A beep is always sampled at 8 kHz (data book 6.3), whatever the last
	// phrase used.
	assign rate_code   = (state == V_BEEP) ? 3'd3 : rate_l;
	assign rate_drives = (state == V_PLAY) || (state == V_BEEP);
	assign active      = (state != V_IDLE);
	assign rom_req     = (state == V_PLAY) && !byte_ok;
	assign rom_addr    = ptr;

	always_ff @(posedge clk_sys) begin
		if (rst) begin
			state       <= V_IDLE;
			base        <= 24'd0;
			len_hold    <= 24'd0;
			ptr         <= 24'd0;
			bytes_left  <= 24'd0;
			nib_hi      <= 1'b1;
			cur_byte    <= 8'd0;
			byte_ok     <= 1'b0;
			sig         <= 12'sd0;
			step        <= 6'd0;
			rate_l      <= 3'd3;
			tick_cnt    <= 16'd0;
			units_left  <= 7'd0;
			beep_cnt    <= 13'd0;
			beep_lvl    <= 1'b0;
			zero_next   <= 1'b0;
			rep_inf     <= 1'b0;
			plays_left  <= 3'd0;
			fade        <= 2'd0;
			sm_l        <= 1'b0;
			vl_l        <= 2'd0;
			bl_l        <= 2'd0;
			bf_l        <= 2'd0;
			sample      <= 12'sd0;
		end else if (stop) begin
			// Stop code: vocalization ends and AOUT settles to 1/2 VDD.  The
			// oscillator and the analog block keep running.
			state   <= V_IDLE;
			sample  <= 12'sd0;
			byte_ok <= 1'b0;
			zero_next  <= 1'b0;
		end else if (start_speech) begin
			state      <= V_PLAY;
			base       <= phrase_start;
			len_hold   <= phrase_len;
			ptr        <= phrase_start;
			bytes_left <= phrase_len;
			nib_hi     <= 1'b1;
			byte_ok    <= 1'b0;
			sig        <= 12'sd0;
			step       <= 6'd0;
			rate_l     <= phrase_rate;
			sm_l       <= ctrl_sm;
			vl_l       <= ctrl_vl;
			fade       <= 2'd0;
			rep_inf    <= (ctrl_rp == 2'b11);
			zero_next  <= 1'b0;
			case (ctrl_rp)
			2'b01:   plays_left <= 3'd2;
			2'b10:   plays_left <= 3'd4;
			default: plays_left <= 3'd1;
			endcase
		end else if (start_silence) begin
			state      <= V_SIL;
			units_left <= units;
			tick_cnt   <= TICK_RELOAD;
			sample     <= 12'sd0;
			zero_next  <= 1'b0;
		end else if (start_beep && IS_CH1) begin
			state      <= V_BEEP;
			units_left <= units;
			tick_cnt   <= TICK_RELOAD;
			bl_l       <= beep_bl;
			bf_l       <= beep_bf;
			beep_cnt   <= beep_half(beep_bf) >> FS_DIV_SHIFT;
			beep_lvl   <= 1'b1;
			sample     <= beep_amp(beep_bl);
			zero_next  <= 1'b0;
		end else begin
			// One byte in flight at a time.  The fetch runs independently of
			// the sample tick, so a slow cart ROM read cannot skew the rate.
			if (rom_req && rom_ready) begin
				cur_byte <= rom_data;
				byte_ok  <= 1'b1;
			end

			if (ce_osc) begin
				// The last sample of a phrase stays on the DAC for its own
				// period; only then does AOUT settle back to 1/2 VDD.
				if (zero_next && fs_ce) begin
					zero_next <= 1'b0;
					sample    <= 12'sd0;
				end

				case (state)
				V_PLAY:
					if (fs_ce) begin
						if (byte_ok) begin
							sig         <= new_sig;
							sample      <= new_sig >>> att;
							step        <= (step_next < 8'sd0)  ? 6'd0
							             : (step_next > 8'sd48) ? 6'd48
							             :  step_next[5:0];

							if (nib_hi) begin
								nib_hi <= 1'b0;
							end else begin
								nib_hi     <= 1'b1;
								byte_ok    <= 1'b0;
								ptr        <= ptr + 24'd1;
								bytes_left <= bytes_left - 24'd1;
								if (bytes_left <= 24'd1) begin
									if (rep_inf || plays_left > 3'd1) begin
										// Repeat: the predictor restarts with
										// the phrase, and the fade steps down.
										ptr        <= base;
										bytes_left <= len_hold;
										sig        <= 12'sd0;
										step       <= 6'd0;
										if (fade != 2'd3) fade <= fade + 2'd1;
										if (!rep_inf) plays_left <= plays_left - 3'd1;
									end else begin
										state     <= V_IDLE;
										zero_next <= 1'b1;
									end
								end
							end
						end
					end

				V_SIL:
					if (tick_cnt == 16'd0) begin
						tick_cnt <= TICK_RELOAD;
						if (units_left <= 7'd1) state <= V_IDLE;
						else units_left <= units_left - 7'd1;
					end else begin
						tick_cnt <= tick_cnt - 16'd1;
					end

				V_BEEP: begin
					if (beep_cnt <= 13'd1) begin
						beep_cnt <= beep_half(bf_l) >> FS_DIV_SHIFT;
						beep_lvl <= ~beep_lvl;
						sample   <= beep_lvl ? -beep_amp(bl_l) : beep_amp(bl_l);
					end else begin
						beep_cnt <= beep_cnt - 13'd1;
					end
					if (tick_cnt == 16'd0) begin
						tick_cnt <= TICK_RELOAD;
						if (units_left <= 7'd1) begin
							state  <= V_IDLE;
							sample <= 12'sd0;
						end else begin
							units_left <= units_left - 7'd1;
						end
					end else begin
						tick_cnt <= tick_cnt - 16'd1;
					end
				end

				default: ;
				endcase
			end
		end
	end

endmodule
