// Copyright (c) 2026 Jamie Blanks

// The seal printer mechanism, seen only through the sensors the VDP reads.
//
// Writes to PRINT_MOTOR step a four-phase stepper through the nibbles C, 6,
// 3, 9 and park it at 0. A step is a move to the next or previous ring entry:
//
//     forward   C -> 6 -> 3 -> 9 -> C        reverse   C -> 9 -> 3 -> 6 -> C
//     index     0    1    2    3             a +1 index difference is forward,
//                                            +3 is reverse, anything else is
//                                            a jump and moves nothing
//
// One counter stands for the paper travel. The two paper sensors are windows
// on it, offset because the upper sensor sits at the front of the mechanism
// and the lower one behind the head:
//
//     step in period   0                63
//     upper mark       ####----------------
//     lower mark       --------####--------
//     home rests here                    ^
//
// The BIOS homing loops only wait for a transition on a debounced sensor byte
// within a step budget, so the spacing is a guess chosen to sit well inside
// the tightest budget.
//
// With no cassette fitted the outputs are SCT 0, OPTO 011.

module loopy_printer
(
	input  wire        clk,        // clk_video, the VDP's clock
	input  wire        reset,
	input  wire        ce,         // ce_vdp; the mechanism is slower than this
										// by five orders of magnitude

	// Savestate scalar bus (clk_sys).
	input  wire        ss_clk,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	input  wire        cassette,   // a seal cassette is fitted
	input  wire [3:0]  motor_phase,

	output wire [2:0]  sct,
	output wire [2:0]  opto,

	// One pulse per motor step, either way round. While a row is printing the
	// motor takes exactly two steps between head words; a retract is hundreds
	// with the head silent. Gears and clutches decide what a step moves.
	output reg         step_o
);
	// Paper travel. MARK_PERIOD is the distance between alignment marks and
	// MARK_LEN how much of it is black; SENSOR_GAP separates the two paper
	// sensors. HOME_STEP parks the model clear of both windows so a fitted
	// cassette reads OPTO 100 before anything moves. Guessed geometry.
	localparam [6:0] MARK_PERIOD = 7'd64;
	localparam [6:0] MARK_LEN    = 7'd16;
	localparam [6:0] SENSOR_GAP  = 7'd24;
	localparam [6:0] HOME_STEP   = 7'd48;

	// Ribbon travel between colour changes, in steps of the same motor.
	localparam [9:0] INK_PERIOD  = 10'd512;


	// Restored savestate values; the packing is at the foot of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_pr;
	/* verilator lint_on UNUSEDSIGNAL */

	reg  [1:0] idx_q;
	reg        idx_valid_q;
	reg  [6:0] mark_cnt;
	reg  [9:0] ink_cnt;
	reg        ink;


	// Phase nibble to ring index. Only the four driven patterns count; the
	// parked value 0 and anything else leave the mechanism where it is.
	reg  [1:0] idx;
	reg        idx_valid;
	always @* begin
		idx_valid = 1'b1;
		case (motor_phase)
		4'hC:    idx = 2'd0;
		4'h6:    idx = 2'd1;
		4'h3:    idx = 2'd2;
		4'h9:    idx = 2'd3;
		default: begin idx = 2'd0; idx_valid = 1'b0; end
		endcase
	end

	wire [1:0] delta   = idx - idx_q;
	wire       moving  = idx_valid & idx_valid_q;
	wire       step_fw = moving & (delta == 2'd1);
	wire       step_rv = moving & (delta == 2'd3);

	always @(posedge clk) begin
		step_o <= 1'b0;

		if (reset) begin
			idx_q       <= ss_pr[1:0];
			idx_valid_q <= ss_pr[2];
			mark_cnt    <= ss_pr[9:3];
			ink_cnt     <= ss_pr[19:10];
			ink         <= ss_pr[20];
		end else if (ce) begin
			idx_q       <= idx;
			idx_valid_q <= idx_valid;

			if (step_fw | step_rv) step_o <= 1'b1;

			if (step_fw) begin
				mark_cnt <= (mark_cnt == MARK_PERIOD - 7'd1) ? 7'd0 : mark_cnt + 7'd1;
				if (ink_cnt == INK_PERIOD - 10'd1) begin
					ink_cnt <= 10'd0;
					ink     <= ~ink;
				end else begin
					ink_cnt <= ink_cnt + 10'd1;
				end
			end else if (step_rv) begin
				mark_cnt <= (mark_cnt == 7'd0) ? MARK_PERIOD - 7'd1 : mark_cnt - 7'd1;
				if (ink_cnt == 10'd0) begin
					ink_cnt <= INK_PERIOD - 10'd1;
					ink     <= ~ink;
				end else begin
					ink_cnt <= ink_cnt - 10'd1;
				end
			end
		end
	end

	wire mark_up = (mark_cnt < MARK_LEN);
	wire mark_lo = (mark_cnt >= SENSOR_GAP) & (mark_cnt < SENSOR_GAP + MARK_LEN);

	// XS-11 is the ordinary sticker cassette. With nothing fitted the ink
	// sensor passes red light and both paper sensors see no paper.
	assign sct  = cassette ? 3'b001 : 3'b000;
	assign opto = cassette ? {ink, mark_up, mark_lo} : 3'b011;

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"

	// The reset value parks a fitted cassette at OPTO 100, which is where the
	// print routine expects to find it.
	localparam [63:0] SS_HOME = {43'd0, 1'b1, 10'd0, HOME_STEP, 1'b0, 2'd0};

	ss_reg #(.ADDR (SSW_PRINTER), .DEFAULT (SS_HOME)) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({43'd0, ink, ink_cnt, mark_cnt, idx_valid_q, idx_q}),
		.dout_o     (ss_pr)
	);

endmodule
