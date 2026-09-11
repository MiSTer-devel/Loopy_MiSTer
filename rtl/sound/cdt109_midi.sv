// Copyright (c) 2026 Jamie Blanks

// The CDT109's MIDI input: a 31250 baud receiver and the message parser
// behind it, fed from the SH7021's SCI1 TxD. Baud comes from the RH-7501's
// 4 MHz X201: 4 MHz / 128 is 31250 exactly, and 4 MHz / 8 is the 16x sampling
// clock.
//
// Running status is restored after every complete message, a full parameter
// buffer swallows further data bytes until the next status byte, and sysex is
// skipped by counting. Only channels 1 to 4 exist; messages on other channels
// are parsed to completion so running status stays right, then dropped.

module cdt109_midi (
	input  wire        clk_i,
	input  wire        rst_i,
	input  wire        ce_4m_i,        // RH-7501 X201, the receiver's clock

	input  wire        rxd_i,          // SCI1 TxD, idles high

	// Which of the four channels the configuration has enabled for MIDI.
	input  wire [3:0]  midi_en_i,

	// One event at a time to the sequencer.
	output wire        ev_valid_o,
	output wire [1:0]  ev_channel_o,
	output wire [2:0]  ev_kind_o,
	output wire [7:0]  ev_value_o,
	input  wire        ev_ack_i,

	output wire        idle_o,         // no byte in the shifter, queue empty

	// Savestate.
	input  wire [63:0] ss_din_i,
	input  wire [9:0]  ss_addr_i,
	input  wire        ss_wren_i,
	input  wire        ss_rst_i,
	output wire [63:0] ss_dout_o
);
	`include "ss_map.svh"

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_midi, ss_midiq;
	/* verilator lint_on UNUSEDSIGNAL */

	localparam [2:0] EV_NOTE_OFF = 3'd0;
	localparam [2:0] EV_NOTE_ON  = 3'd1;
	localparam [2:0] EV_PROG     = 3'd2;
	localparam [2:0] EV_BEND     = 3'd3;
	localparam [2:0] EV_SUSTAIN  = 3'd4;

	// ---- receiver ---------------------------------------------------------
	//
	// 4 MHz / 8 gives the 500 kHz sampling clock, so a bit is 16 samples and
	// the middle of a bit is 8 after its edge.

	reg  [2:0] baud_div;
	wire       ce_16x = ce_4m_i & (baud_div == 3'd7);

	reg  [1:0] rxd_sync;
	wire       rxd = rxd_sync[1];

	reg        rx_busy;
	reg  [3:0] rx_phase;      // 0-15 within the bit
	reg  [3:0] rx_bit;        // 0-7 data, 8 stop
	reg  [7:0] rx_shift;
	reg        rx_stb;
	reg  [7:0] rx_data;

	always @(posedge clk_i) begin
		if (rst_i) begin
			baud_div <= 3'd0;
			rxd_sync <= 2'b11;
			rx_busy  <= ss_midi[37];
			rx_phase <= ss_midi[41:38];
			rx_bit   <= ss_midi[45:42];
			rx_shift <= ss_midi[53:46];
			rx_stb   <= 1'b0;
			rx_data  <= ss_midi[61:54];
		end else begin
			rxd_sync <= {rxd_sync[0], rxd_i};
			rx_stb   <= 1'b0;

			if (ce_4m_i) baud_div <= baud_div + 3'd1;

			if (ce_16x) begin
				if (!rx_busy) begin
					// A low line is a start bit until the middle of it says
					// otherwise.
					if (!rxd) begin
						rx_busy  <= 1'b1;
						rx_phase <= 4'd0;
						rx_bit   <= 4'd0;
					end
				end else begin
					rx_phase <= rx_phase + 4'd1;
					if (rx_phase == 4'd7) begin
						if (rx_bit == 4'd0) begin
							// Middle of the start bit.
							if (rxd) rx_busy <= 1'b0;   // noise, not a start
						end else if (rx_bit <= 4'd8) begin
							rx_shift <= {rxd, rx_shift[7:1]};
						end else begin
							// Middle of the stop bit. A framing error drops the byte.
							if (rxd) begin
								rx_stb  <= 1'b1;
								rx_data <= rx_shift;
							end
							rx_busy <= 1'b0;
						end
					end
					if (rx_phase == 4'd15) rx_bit <= rx_bit + 4'd1;
				end
			end
		end
	end

	// Queue state, declared ahead of the interlock that reads q_full.
	localparam int QD = 4;

	reg [12:0] queue [QD];
	reg  [2:0] q_wr, q_rd;
	wire       q_empty = (q_wr == q_rd);
	wire       q_full  = (q_wr[1:0] == q_rd[1:0]) && (q_wr[2] != q_rd[2]);

	// ---- the interlock ------------------------------------------------------
	//
	// A framed byte is held until the parser can take it, and the parser takes
	// one only when the event queue has room, so `push` never meets a full
	// queue. The remaining loss is a second byte framed while the first is
	// still held, which takes a whole byte time of stall and is counted.
	reg  [7:0] rx_hold;
	reg        rx_hold_valid;
	wire       parse_go = rx_hold_valid & ~q_full;

	always @(posedge clk_i) begin
		if (rst_i) begin
			rx_hold       <= 8'd0;
			rx_hold_valid <= 1'b0;
		end else begin
			if (rx_stb) begin
				// A byte arriving on top of one still waiting is lost, the
				// overrun a real receiver reports.
				rx_hold       <= rx_data;
				rx_hold_valid <= 1'b1;
			end else if (parse_go) begin
				rx_hold_valid <= 1'b0;
			end
		end
	end


	// ---- parser -----------------------------------------------------------

	reg  [7:0] status;
	reg  [7:0] running_status;
	reg  [6:0] param0, param1;
	reg  [3:0] param_count;
	reg        in_sysex;

	wire [3:0] status_hi = status[7:4];
	wire [1:0] ev_channel = status[1:0];
	wire       channel_ok = (status[3:2] == 2'b00);      // MIDI channels 1-4
	wire       enabled = channel_ok & midi_en_i[ev_channel];
	wire [3:0] msg_size = ((status_hi == 4'hC) || (status_hi == 4'hD)) ? 4'd1 : 4'd2;

	// The parameter byte that is landing this cycle, so a one-byte message can
	// be dispatched without waiting a cycle for the register.
	wire [6:0] this_param = rx_hold[6:0];
	wire [6:0] use0 = (param_count == 4'd0) ? this_param : param0;
	wire [6:0] use1 = (param_count == 4'd1) ? this_param : param1;

	// Bend byte: the coarse value stretched to eight bits by repeating its top
	// bit, so 0x7F reaches 0xFF and the neutral 0x40 lands on 0x81.
	wire [7:0] bend_byte = {use1, 1'b0} | {7'd0, use1[6]};

	reg        push;
	reg  [1:0] push_channel;
	reg  [2:0] push_kind;
	reg  [7:0] push_value;

	always @(posedge clk_i) begin
		if (rst_i) begin
			status         <= ss_midi[7:0];
			running_status <= ss_midi[15:8];
			param0         <= ss_midi[22:16];
			param1         <= ss_midi[30:24];
			param_count    <= ss_midi[35:32];
			in_sysex       <= ss_midi[36];
			push           <= 1'b0;
		end else begin
			push <= 1'b0;

			if (parse_go) begin
				if (rx_hold[7]) begin
					// Status byte.
					if (rx_hold == 8'hF0) in_sysex <= 1'b1;
					if (rx_hold == 8'hF7) in_sysex <= 1'b0;
					if (rx_hold < 8'hF8) begin
						status         <= rx_hold;
						running_status <= (rx_hold < 8'hF0) ? rx_hold : 8'd0;
						param_count    <= 4'd0;
					end
				end else if ((param_count < 4'd8) && (status != 8'd0)) begin
					if (param_count == 4'd0) param0 <= this_param;
					if (param_count == 4'd1) param1 <= this_param;
					param_count <= param_count + 4'd1;

					if (!in_sysex && (status_hi != 4'hF) &&
					    (param_count + 4'd1 >= msg_size)) begin
						param_count <= 4'd0;
						status      <= running_status;

						if (enabled) begin
							push_channel <= ev_channel;
							push_value   <= {1'b0, use0};
							case (status_hi)
							4'h8: begin push <= 1'b1; push_kind <= EV_NOTE_OFF; end
							4'h9: begin
								push <= 1'b1;
								push_kind <= (use1 != 7'd0) ? EV_NOTE_ON : EV_NOTE_OFF;
							end
							4'hB: if (use0 == 7'h40) begin
								push       <= 1'b1;
								push_kind  <= EV_SUSTAIN;
								push_value <= {7'd0, use1[6]};
							end
							4'hC: begin push <= 1'b1; push_kind <= EV_PROG; end
							4'hE: begin
								push       <= 1'b1;
								push_kind  <= EV_BEND;
								push_value <= bend_byte;
							end
							default: ;
							endcase
						end
					end
				end
			end
		end
	end

	// ---- event queue ------------------------------------------------------

	always @(posedge clk_i) begin
		if (rst_i) begin
			q_wr <= ss_midiq[54:52];
			q_rd <= ss_midiq[57:55];
			for (int i = 0; i < QD; i++) queue[i] <= ss_midiq[i*13 +: 13];
		end else begin
			if (push) begin
				queue[q_wr[1:0]] <= {push_channel, push_kind, push_value};
				q_wr <= q_wr + 3'd1;
			end
			if (ev_valid_o && ev_ack_i) q_rd <= q_rd + 3'd1;
		end
	end

	assign ev_valid_o   = ~q_empty;
	assign ev_channel_o = queue[q_rd[1:0]][12:11];
	assign ev_kind_o    = queue[q_rd[1:0]][10:8];
	assign ev_value_o   = queue[q_rd[1:0]][7:0];

	// A save waits for idle, which includes the held byte, so it needs no
	// savestate word.
	assign idle_o = q_empty & ~rx_busy & ~push & ~rx_hold_valid;

	// ---- savestate --------------------------------------------------------

	wire [63:0] ss_dout_midi, ss_dout_midiq;
	assign ss_dout_o = ss_dout_midi | ss_dout_midiq;

	wire [63:0] midi_back = {2'd0,
	                         rx_data, rx_shift, rx_bit, rx_phase, rx_busy,
	                         in_sysex, param_count,
	                         1'b0, param1, 1'b0, param0,
	                         running_status, status};

	ss_reg #(.ADDR (SSW_SND_MIDI)) u_ss_midi (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_dout_midi),
		.din_i      (midi_back),
		.dout_o     (ss_midi)
	);

	wire [63:0] midiq_back = {6'd0, q_rd, q_wr,
	                          queue[3], queue[2], queue[1], queue[0]};

	ss_reg #(.ADDR (SSW_SND_MIDIQ)) u_ss_midiq (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din_i),
		.bus_addr_i (ss_addr_i),
		.bus_wren_i (ss_wren_i),
		.bus_rst_i  (ss_rst_i),
		.bus_dout_o (ss_dout_midiq),
		.din_i      (midiq_back),
		.dout_o     (ss_midiq)
	);

endmodule
