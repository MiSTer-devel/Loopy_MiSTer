// Four-phase toggle handshake used for every clock domain crossing in the core.
//
// The source latches its payload and flips a toggle; the destination sees it
// through two flops, does the work, and flips its own toggle back. The payload
// never moves while the far side is looking at it, so only the toggles need
// synchronising.
//
//   src_req  ___/‾\_______________________     one cycle, only when !src_busy
//   src_busy ____/‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾‾\______
//   dst_valid __________/‾‾‾‾‾‾\____________   level, until dst_ack
//   dst_ack  _______________/‾\_____________   one cycle
//   src_done ________________________/‾\____   one cycle, src_resp valid
//
// Round trip is roughly four to six cycles of the slower clock.
//
// SYNCHRONIZER_IDENTIFICATION lets Quartus report metastability on the toggle
// synchronisers; the paths into them still have to be cut in the SDC.

module cdc_handshake #(
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

	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] req_sync;
	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] ack_sync;

	reg req_toggle;
	reg ack_toggle;
	reg ack_seen;

			// Each payload register is written by one domain and read by the
			// other, never while the writer could still change it.
	reg [WIDTH-1:0]      data_reg;
	reg [RESP_WIDTH-1:0] resp_reg;

	assign src_busy = (req_toggle != ack_sync[1]);
	assign src_resp = resp_reg;
	assign dst_data = data_reg;
	assign dst_valid = (req_sync[1] != ack_toggle);

	always @(posedge src_clk) begin
		if (src_reset) begin
			req_toggle <= 1'b0;
			ack_sync   <= 2'b00;
			ack_seen   <= 1'b0;
			src_done   <= 1'b0;
			data_reg   <= '0;
		end else begin
			ack_sync <= {ack_sync[0], ack_toggle};
			ack_seen <= ack_sync[1];

			// Acknowledge toggled: transaction finished, src_resp holds the
			// answer. Lands one cycle after src_busy falls.
			src_done <= (ack_sync[1] != ack_seen);

			if (src_req && !src_busy) begin
				data_reg   <= src_data;
				req_toggle <= ~req_toggle;
			end
		end
	end

	always @(posedge dst_clk) begin
		if (dst_reset) begin
			ack_toggle <= 1'b0;
			req_sync   <= 2'b00;
			resp_reg   <= '0;
		end else begin
			req_sync <= {req_sync[0], req_toggle};

			if (dst_valid && dst_ack) begin
				resp_reg   <= dst_resp;
				ack_toggle <= req_sync[1];
			end
		end
	end

endmodule
