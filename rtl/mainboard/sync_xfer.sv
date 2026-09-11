// Copyright (c) 2026 Jamie Blanks

// Request/response transfer between clk_sys and clk_ram. clk_ram is exactly
// four times clk_sys off the same PLL, so the paths are timed rather than cut
// and a plain four-phase handshake on two levels is enough: about one and a
// quarter clk_sys cycles end to end. Same interface as `cdc_handshake`, which
// is still required for the 4:3 clk_video crossing.
//
//   src_req   ___/‾\_________________________   one cycle, only when !src_busy
//   pend      ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______   level, cleared when acked
//   dst_valid ____/‾‾‾‾‾‾‾‾‾‾‾‾‾\____________   level, until dst_ack
//   acked     ______________/‾‾‾‾‾‾‾‾‾\______   level, cleared when pend does
//   src_done  ______________________/‾\______   one cycle, src_resp valid

module sync_xfer #(
	parameter int unsigned WIDTH = 32,
	parameter int unsigned RESP_WIDTH = 32
) (
	input  wire                  src_clk,
	input  wire                  src_reset,
	input  wire                  src_req,     // one cycle, only while !src_busy
	input  wire [WIDTH-1:0]      src_data,
	output wire                  src_busy,
	output reg                   src_done,    // one cycle
	output wire [RESP_WIDTH-1:0] src_resp,    // valid on src_done

	input  wire                  dst_clk,
	input  wire                  dst_reset,
	output wire                  dst_valid,   // level, until dst_ack
	output wire [WIDTH-1:0]      dst_data,
	input  wire                  dst_ack,     // one cycle
	input  wire [RESP_WIDTH-1:0] dst_resp
);
	reg                  pend;
	reg                  acked;
	reg [WIDTH-1:0]      data_reg;
	reg [RESP_WIDTH-1:0] resp_reg;

	assign src_busy  = pend;
	assign src_resp  = resp_reg;
	assign dst_data  = data_reg;
	assign dst_valid = pend & ~acked;

	always @(posedge src_clk) begin
		if (src_reset) begin
			pend     <= 1'b0;
			src_done <= 1'b0;
			data_reg <= '0;
		end else begin
			src_done <= 1'b0;
			if (!pend) begin
				if (src_req) begin
					data_reg <= src_data;
					pend     <= 1'b1;
				end
			end else if (acked) begin
				// The far side has finished and its answer has been sitting in
				// resp_reg since before this edge.
				pend     <= 1'b0;
				src_done <= 1'b1;
			end
		end
	end

	always @(posedge dst_clk) begin
		if (dst_reset) begin
			acked    <= 1'b0;
			resp_reg <= '0;
		end else begin
			if (dst_valid && dst_ack) begin
				resp_reg <= dst_resp;
				acked    <= 1'b1;
			end else if (!pend) begin
				acked <= 1'b0;
			end
		end
	end
endmodule
