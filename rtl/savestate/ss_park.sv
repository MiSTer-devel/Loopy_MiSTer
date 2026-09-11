// Copyright (c) 2026 Jamie Blanks

// Brings the machine to the one place a savestate may stop it, and holds it
// there. Each domain runs on to a point where its pipelines are drained and
// nothing is owed to SDRAM, so none of that has to be saved.
//
// The CPU (instruction boundary, DMA between units) and the synth (sample
// boundary) stop together on clk_sys. The VDP stops second, at the top of a
// frame: a CPU access to the VDP is held on /WAIT, so stopping the VDP first
// with a cycle outstanding would deadlock. Then the work DRAM's line cache is
// flushed.
//
//   req     ___/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   pause_sys _______/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   pause_video __________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//   flush     ________________/‾‾‾‾‾‾‾‾‾‾‾\______________________
//   parked    ____________________________/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\____
//
// Worst case is one video frame, about 17 ms, before a save or load starts.

module ss_park (
	input  wire clk_sys,
	input  wire reset_sys,

	input  wire req,             // sleep_savestate from the transfer engine
	input  wire cpu_ready,       // SH7021 at an instruction boundary
	input  wire snd_ready,       // synth at a sample boundary
	input  wire mem_idle,        // line cache flushed and idle

	output reg  pause_sys,
	output reg  flush,
	output reg  parked,

	input  wire clk_video,
	input  wire reset_video,
	input  wire vdp_ready,       // RH-7500 at the top of a frame
	output reg  pause_video
);

	localparam [2:0] S_RUN    = 3'd0;
	localparam [2:0] S_CPU    = 3'd1;
	localparam [2:0] S_VDP    = 3'd2;
	localparam [2:0] S_FLUSH  = 3'd3;
	localparam [2:0] S_PARKED = 3'd4;

	reg [2:0] st;

	// The two levels that cross: the request into the video domain and the
	// answer back. Both are single bits held for the whole transfer.
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] vreq_q;
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] vack_q;

	wire want_video = (st == S_VDP) || (st == S_FLUSH) || (st == S_PARKED);

	always @(posedge clk_video) begin
		if (reset_video) begin
			vreq_q      <= 2'b00;
			pause_video <= 1'b0;
		end else begin
			vreq_q <= {vreq_q[0], want_video};
			if (!vreq_q[1])          pause_video <= 1'b0;
			else if (vdp_ready)      pause_video <= 1'b1;
		end
	end

	always @(posedge clk_sys) begin
		if (reset_sys) begin
			st         <= S_RUN;
			pause_sys  <= 1'b0;
			flush      <= 1'b0;
			parked     <= 1'b0;
			vack_q     <= 2'b00;
		end else begin
			vack_q <= {vack_q[0], pause_video};

			case (st)
			S_RUN: begin
				pause_sys <= 1'b0;
				flush     <= 1'b0;
				parked    <= 1'b0;
				if (req) st <= S_CPU;
			end

			S_CPU: begin
				if (!req) begin
					st <= S_RUN;
				end else if (cpu_ready && snd_ready) begin
					pause_sys <= 1'b1;
					st <= S_VDP;
				end
			end

			S_VDP: begin
				if (!req)           st <= S_RUN;
				else if (vack_q[1]) st <= S_FLUSH;
			end

			S_FLUSH: begin
				flush <= 1'b1;
				if (!req) begin
					flush <= 1'b0;
					st    <= S_RUN;
				end else if (flush && mem_idle) begin
					flush  <= 1'b0;
					parked <= 1'b1;
					st     <= S_PARKED;
				end
			end

			S_PARKED: begin
				if (!req) begin
					pause_sys <= 1'b0;
					parked    <= 1'b0;
					st        <= S_RUN;
				end
			end

			default: st <= S_RUN;
			endcase
		end
	end

endmodule
