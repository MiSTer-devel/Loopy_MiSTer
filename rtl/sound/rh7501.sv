// Copyright (c) 2026 Jamie Blanks

// LSI201, the Casio RH-7501: the keyboard front panel the CDT109 expects,
// six momentary buttons and two three-position volume sliders, driven here
// from the SOUND_CTRL register. The clocks come from `loopy_clocks` and the
// working RAM sits inside the CDT109 model.
//
// Software sets a button bit, waits a frame and clears it, so the rising edge
// of each bit is a press. One write can press several buttons; they are
// walked in a fixed order and handed to the CDT109 one command at a time.
//
//   ON    clear to one channel, clear the programs
//   DEMO  toggle the internal tune (not played)
//   MIDI  leave keyboard mode; games run here
//   EXT   select a rhythm preset (not played)
//   CH4   split into four channels with MIDI on all of them
//   CH3   split into four channels with MIDI on the first three
//
// MIDI, CH3 and CH4 silence the voices on every press, even when the mode
// stays the same: a BGM track's header write is what stops the notes the
// previous track left holding.

module rh7501 (
	input  wire        clk_i,
	input  wire        rst_i,

	// SOUND_CTRL, already across from the VDP's clock domain.
	input  wire [11:0] snd_ctl_i,

	// To the CDT109.
	output reg         cmd_valid_o,
	output reg  [1:0]  cmd_kind_o,
	output reg  [1:0]  cmd_arg_o,
	input  wire        cmd_ack_i,
	output wire [2:0]  slider_a_o,     // channels 1-3, index into the level table
	output wire [2:0]  slider_r_o,     // channel 4
	output wire        midi_gate_o,    // MIDI reaches the synth only when high

	output wire        idle_o,

	// Savestate.
	input  wire [63:0] ss_din_i,
	input  wire [9:0]  ss_addr_i,
	input  wire        ss_wren_i,
	input  wire        ss_rst_i,
	output wire [63:0] ss_dout_o
);
	`include "ss_map.svh"

	localparam [1:0] CMD_CFG   = 2'd0;   // arg = {multi, all}
	localparam [1:0] CMD_RESET = 2'd1;   // arg[0] = clear the programs

	// Button bit positions inside SOUND_CTRL.
	/* verilator lint_off UNUSEDPARAM */
	localparam int B_DEMO = 0;
	localparam int B_CH3  = 1;
	localparam int B_CH4  = 2;
	localparam int B_EXT  = 3;
	localparam int B_ON   = 4;
	localparam int B_MIDI = 5;
	/* verilator lint_on UNUSEDPARAM */

	reg  [5:0] buttons_last;
	reg  [5:0] pushed;
	reg  [2:0] config_state;    // 0 keyboard, 1 MIDI, 3 three channels, 4 four
	reg        in_demo;
	reg  [2:0] slider_a, slider_r;

	assign slider_a_o  = slider_a;
	assign slider_r_o  = slider_r;
	assign midi_gate_o = ~in_demo & (config_state != 3'd0);

	// The sliders decode straight from the register, outside the walk.
	wire [2:0] sw_a = snd_ctl_i[8:6];
	wire [2:0] sw_r = snd_ctl_i[11:9];

	function automatic [2:0] slider_level(input [2:0] sw, input [2:0] held);
		if      (sw[0]) slider_level = 3'd2;
		else if (sw[1]) slider_level = 3'd3;
		else if (sw[2]) slider_level = 3'd4;
		else            slider_level = held;
	endfunction

	// ---- the walk ---------------------------------------------------------
	//
	// One state per button, in order. Each button that fired issues its
	// commands before the next is looked at; a command waits until acked.

	localparam [3:0] S_IDLE = 4'd0, S_ON = 4'd1, S_DEMO = 4'd2, S_MIDI = 4'd3,
	                 S_EXT  = 4'd4, S_CH4 = 4'd5, S_CH3 = 4'd6, S_WAIT = 4'd7;

	reg  [3:0] state;
	reg  [3:0] state_after;
	reg  [1:0] second_kind;
	reg  [1:0] second_arg;
	reg        second_valid;

	wire [5:0] buttons = snd_ctl_i[5:0];
	wire [5:0] rising  = buttons & ~buttons_last;

	assign idle_o = (state == S_IDLE) & (rising == 6'd0);

	// Issue `first` now and `second` after it, then go to `next`.
	task automatic issue(input [1:0] k0, input [1:0] a0,
	                     input       have1, input [1:0] k1, input [1:0] a1,
	                     input [3:0] next);
		begin
			cmd_valid_o  <= 1'b1;
			cmd_kind_o   <= k0;
			cmd_arg_o    <= a0;
			second_valid <= have1;
			second_kind  <= k1;
			second_arg   <= a1;
			state_after  <= next;
			state        <= S_WAIT;
		end
	endtask

	always @(posedge clk_i) begin
		if (rst_i) begin
			buttons_last <= ss_rh[5:0];
			config_state <= ss_rh[8:6];
			in_demo      <= ss_rh[9];
			slider_a     <= ss_rh[12:10];
			slider_r     <= ss_rh[15:13];
			pushed       <= 6'd0;
			state        <= S_IDLE;
			state_after  <= S_IDLE;
			second_valid <= 1'b0;
			second_kind  <= CMD_CFG;
			second_arg   <= 2'd0;
			cmd_valid_o  <= 1'b0;
			cmd_kind_o   <= CMD_CFG;
			cmd_arg_o    <= 2'd0;
		end else begin
			slider_a <= slider_level(sw_a, slider_a);
			slider_r <= slider_level(sw_r, slider_r);

			case (state)
			S_IDLE: begin
				if (rising != 6'd0) begin
					pushed       <= rising;
					buttons_last <= buttons;
					state        <= S_ON;
				end else begin
					buttons_last <= buttons;
				end
			end

			S_ON: begin
				if (pushed[B_ON]) begin
					config_state <= 3'd0;
					issue(CMD_CFG, 2'b00, 1'b1, CMD_RESET, 2'b01, S_DEMO);
				end else begin
					state <= S_DEMO;
				end
			end

			S_DEMO: begin
				if (pushed[B_DEMO]) begin
					in_demo <= ~in_demo;
					// Entering the tune silences what MIDI was playing.
					if (!in_demo) issue(CMD_RESET, 2'b00, 1'b0, CMD_CFG, 2'd0, S_MIDI);
					else          state <= S_MIDI;
				end else begin
					state <= S_MIDI;
				end
			end

			S_MIDI: begin
				if (pushed[B_MIDI] && (config_state == 3'd0)) begin
					config_state <= 3'd1;
					issue(CMD_CFG, 2'b00, 1'b1, CMD_RESET, 2'b01, S_EXT);
				end else if (pushed[B_MIDI]) begin
					// Already out of keyboard mode: the press still silences but
					// keeps the programs. Many BGM track headers press MIDI alone
					// and rely on it to stop the previous track's notes.
					issue(CMD_RESET, 2'b00, 1'b0, CMD_CFG, 2'd0, S_EXT);
				end else begin
					state <= S_EXT;
				end
			end

			// EXT selects a rhythm preset. The format is unknown and no title
			// presses it, so the button is consumed.
			S_EXT: state <= S_CH4;

			S_CH4: begin
				if (pushed[B_CH4] && ((config_state == 3'd1) || (config_state == 3'd3))) begin
					config_state <= 3'd4;
					issue(CMD_CFG, 2'b11, 1'b1, CMD_RESET, 2'b00, S_CH3);
				end else if (pushed[B_CH4] && (config_state == 3'd4)) begin
					// Already in this layout: the press still silences.
					issue(CMD_RESET, 2'b00, 1'b0, CMD_CFG, 2'd0, S_CH3);
				end else begin
					state <= S_CH3;
				end
			end

			S_CH3: begin
				if (pushed[B_CH3] && (config_state == 3'd1)) begin
					config_state <= 3'd3;
					issue(CMD_CFG, 2'b10, 1'b1, CMD_RESET, 2'b00, S_IDLE);
				end else if (pushed[B_CH3] && (config_state == 3'd3)) begin
					issue(CMD_RESET, 2'b00, 1'b0, CMD_CFG, 2'd0, S_IDLE);
				end else begin
					state <= S_IDLE;
				end
			end

			S_WAIT: begin
				if (cmd_ack_i) begin
					if (second_valid) begin
						cmd_kind_o   <= second_kind;
						cmd_arg_o    <= second_arg;
						second_valid <= 1'b0;
					end else begin
						cmd_valid_o <= 1'b0;
						state       <= state_after;
					end
				end
			end

			default: state <= S_IDLE;
			endcase
		end
	end

	// ---- savestate --------------------------------------------------------

	// A savestate waits for idle_o, so only the state between walks is saved.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_rh;              // the spare bits are growth room
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] rh_back = {48'd0, slider_r, slider_a, in_demo,
	                       config_state, buttons_last};

	// Both sliders come up at position 4, which is 0x9000 in this layout.
	ss_reg #(.ADDR (SSW_SND_RH7501), .DEFAULT (64'h0000_0000_0000_9000)) u_ss (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_dout_o),
		.din_i      (rh_back),
		.dout_o     (ss_rh)
	);

endmodule
