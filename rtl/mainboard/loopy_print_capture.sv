// Copyright (c) 2026 Jamie Blanks

// Rebuilds the sticker from what the print routine sends the thermal head.
// Each DMA is 34 sub-rows of 64 words bracketed by PRINT_HEAD_CTRL; bit b of
// word w is head position (w >> 1) + 32*b - 8 (0..111 is picture), and a bit
// stays high for its first v sub-rows, so counting pulses recovers intensity.
// Averaging a DMA pair and two source columns gives three bits per ink at
// 128 x 112. Printed columns advance right to left; the overlay reverses x.

module loopy_print_capture #(
	// Motor steps with no head word that mean a colour has finished. Printing
	// never puts more than two between words, a retract is hundreds.
	parameter integer RETRACT_STEPS = 16,
	// Frames after the last head write; 900 is fifteen seconds of NTSC.
	parameter integer HOLD_FRAMES = 900
) (
	input  wire        clk,          // clk_video
	input  wire        reset,
	input  wire        job_enable,   // PRINT_SENSORS.ENP brackets a complete job.

	// Savestate scalar bus (clk_sys).
	input  wire        ss_clk,
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// Bits 3 and 2 are thermal control and hold still across a burst.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [3:0]  head_ctrl,
	input  wire [15:0] data,         // only the low nibble carries dots
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        data_wr,
	input  wire        frame_tick,
	// One pulse per motor step. While a row prints the motor takes exactly two
	// steps between head words, so a long run of steps with the head silent is
	// the retract between colours.
	input  wire        step_i,

	// Complete columns stream without waiting for external memory.
	output reg         clear,
	output reg         cap_wr,
	output reg  [6:0]  cap_y,
	output reg  [6:0]  cap_x,
	output reg  [1:0]  cap_pass,
	output reg  [2:0]  cap_ink,

	output wire        show
);
	localparam integer HOLD_W = $clog2(HOLD_FRAMES + 1);
	localparam [HOLD_W-1:0] HOLD_MAX = HOLD_FRAMES[HOLD_W-1:0];
	localparam [HOLD_W-1:0] HOLD_ONE = {{(HOLD_W-1){1'b0}}, 1'b1};

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_pr;
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- where we are ---------------------------------------------------------

	reg              hc_q;
	reg              job_enable_q;
	reg  [5:0]       wpos;        // word inside the 64-word sub-row
	reg              half;        // which burst of the pair
	reg  [7:0]       row;         // printed rows this pass
	reg  [7:0]       step_run;    // motor steps since the last head word
	reg  [1:0]       pass_q;
	reg [HOLD_W-1:0] hold;

	assign show    = (hold != HOLD_MAX);

	wire new_print  = data_wr & ~show;
	// Fires once as the run crosses the limit, so one boundary per retract.
	wire pass_edge  = step_i & (step_run == RETRACT_STEPS[7:0] - 8'd1);
	wire hc_now     = head_ctrl[0];
	wire burst_beg  = hc_now & ~hc_q;
	wire burst_end  = ~hc_now & hc_q;

	// ---- the pulse accumulator ------------------------------------------------

	// Sum four source samples before quantizing; each contributes at most 33.
	wire [7:0] acc_q;
	reg  [6:0] acc_addr;
	reg        acc_we;
	reg  [7:0] acc_din;

	cache_ram_tdp_dc #(.ADDR_WIDTH (7), .DATA_WIDTH (8)) u_acc (
		.clk_a_i (clk), .addr_a_i (acc_addr), .wren_a_i (acc_we),
		.wdata_a_i (acc_din), .q_a_o (acc_q),
		.clk_b_i (clk), .addr_b_i (7'd0), .wren_b_i (1'b0),
		/* verilator lint_off PINCONNECTEMPTY */
		.wdata_b_i (8'd0), .q_b_o ()
		/* verilator lint_on PINCONNECTEMPTY */
	);

	reg have_line;

	// ---- the walk -------------------------------------------------------------

	localparam [3:0] S_IDLE  = 4'd0,
	                 S_ACC_R = 4'd1,
	                 S_ACC_1 = 4'd2,
	                 S_ACC_W = 4'd3,
	                 S_EMIT  = 4'd4,
	                 S_EMIT1 = 4'd5,
	                 S_EMIT2 = 4'd10,   // the RAM answers a cycle after its address
	                 S_CLR   = 4'd9;

	reg [3:0]  st;
	reg [1:0]  blk;          // which of the four bits is being banked
	reg [3:0]  nyb_q;
	/* verilator lint_off UNUSEDSIGNAL */
	reg [5:0]  wq;      // bit 0 is the half selector, which the row pairing uses
	/* verilator lint_on UNUSEDSIGNAL */
	reg [6:0]  walk;
	reg        acc_busy;

	// Wrap the leading margin into the unused positions above 111.
	wire [6:0] pos_of = {blk, 5'd0} + {2'd0, wq[5:1]} + 7'd120;

	function automatic [2:0] ink(input [7:0] c);
		/* verilator lint_off UNUSEDSIGNAL */
		reg [7:0] unbiased;
		/* verilator lint_on UNUSEDSIGNAL */
		begin
			// Four samples each include the BIOS +2 pulse bias. Divide by
			// four to average and four again to retain three density bits.
			unbiased = c - 8'd8;
			ink = c <= 8'd8 ? 3'd0 : c >= 8'd136 ? 3'd7 : unbiased[6:4];
		end
	endfunction

	always @(posedge clk) begin
		if (reset) begin
			job_enable_q <= job_enable;
			hc_q <= 1'b0; wpos <= 6'd0; half <= 1'b0; step_run <= 8'd0;
			row  <= ss_pr[7:0]; pass_q <= ss_pr[9:8]; hold <= ss_pr[10+HOLD_W:11];
			st   <= S_CLR; walk <= 7'd0; blk <= 2'd0;
			have_line <= 1'b0;
			acc_we <= 1'b0; acc_addr <= 7'd0; acc_din <= 8'd0;
			nyb_q <= 4'd0; wq <= 6'd0; acc_busy <= 1'b1;
			clear <= 1'b1; cap_wr <= 1'b0;
			cap_y <= 7'd0; cap_x <= 7'd0; cap_pass <= 2'd0; cap_ink <= 3'd0;
		end else begin
			job_enable_q <= job_enable;
			acc_we <= 1'b0;
			cap_wr <= 1'b0;
			clear <= 1'b0;
			hc_q   <= hc_now;

			if (burst_beg) wpos <= 6'd0;

			if (data_wr)                       step_run <= 8'd0;
			else if (step_i && (step_run != 8'hFF)) step_run <= step_run + 8'd1;

			if (data_wr) begin
				hold <= {HOLD_W{1'b0}};
				if (new_print) begin
					clear <= 1'b1;
					pass_q <= 2'd0; row <= 8'd0; half <= 1'b0; have_line <= 1'b0;
				end
			end else if (frame_tick && show) begin
				hold <= hold + HOLD_ONE;
				if (hold == HOLD_MAX - HOLD_ONE) clear <= 1'b1;
			end

			case (st)
			S_IDLE: begin
				if (data_wr) begin
					nyb_q <= data[3:0];
					wq    <= wpos;
					wpos  <= wpos + 6'd1;
					blk   <= 2'd0;
					st <= S_ACC_R;
				end else if (burst_end) begin
					half <= ~half;
					if (half) begin
						have_line <= ~have_line;
						if (have_line) begin
							walk <= 7'd0;
							st <= S_EMIT;
						end else row <= row + 8'd1;
					end
				end
			end

			// Four blocks, one read-modify-write each. Words are about 181
			// cycles apart, so twelve cycles of work is free.
			S_ACC_R: begin acc_addr <= pos_of; st <= S_ACC_1; end
			S_ACC_1: st <= S_ACC_W;
			S_ACC_W: begin
				if (nyb_q[blk] && (acc_q != 8'hFF)) begin
					acc_din <= acc_q + 8'd1;
					acc_we  <= 1'b1;
				end
				blk <= blk + 2'd1;
				st  <= (blk == 2'd3) ? S_IDLE : S_ACC_R;
			end

			// Four bursts cover both row parities and adjacent source columns.
			// Quantize their mean only after all samples have contributed.
			S_EMIT: begin acc_addr <= walk; st <= S_EMIT1; end
			S_EMIT1: st <= S_EMIT2;
			S_EMIT2: begin
				if (walk < 7'd112) begin
					cap_wr <= 1'b1;
					cap_y <= walk;
					cap_x <= row[7:1];
					cap_pass <= pass_q;
					cap_ink <= ink(acc_q);
				end
				if (walk == 7'd127) begin
					walk     <= 7'd0;
					row <= row + 8'd1;
					acc_busy <= 1'b1;
					st <= S_CLR;
				end else begin
					walk <= walk + 7'd1;
					st   <= S_EMIT;
				end
			end

			// Wipe the accumulator for the next four bursts.
			S_CLR: begin
				if (acc_busy) begin
					acc_addr <= walk;
					acc_din  <= 8'd0;
					acc_we   <= 1'b1;
					walk     <= walk + 7'd1;
					if (walk == 7'd127) acc_busy <= 1'b0;
				end
				if (!acc_busy) st <= S_IDLE;
			end

			default: st <= S_IDLE;
			endcase

			if (pass_edge && show && !new_print && (pass_q != 2'd2)) begin
				pass_q <= pass_q + 2'd1;
				row    <= 8'd0;
				half   <= 1'b0;
				have_line <= 1'b0;
			end

			// ENP rises before motor setup, leaving time to clear the accumulator.
			// Its falling edge leaves the finished sticker on screen.
			if (job_enable && !job_enable_q) begin
				pass_q <= 2'd0; row <= 8'd0; half <= 1'b0;
				step_run <= 8'd0; wpos <= 6'd0;
				hold <= HOLD_MAX;
				have_line <= 1'b0;
				walk <= 7'd0; clear <= 1'b1;
				acc_we <= 1'b0; cap_wr <= 1'b0;
				acc_busy <= 1'b1;
				st <= S_CLR;
			end
		end
	end

	// ---- savestate -------------------------------------------------------------

	`include "ss_map.svh"

	localparam [63:0] SS_IDLE = {{(53-HOLD_W){1'b0}}, HOLD_MAX, 11'd0};

	ss_reg #(.ADDR (SSW_PRINTCAP), .DEFAULT (SS_IDLE)) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({{(53-HOLD_W){1'b0}}, hold, 1'b0, pass_q, row}),
		.dout_o     (ss_pr)
	);

endmodule
