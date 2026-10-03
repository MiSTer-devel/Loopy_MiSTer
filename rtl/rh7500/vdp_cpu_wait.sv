// Copyright (c) 2026 Jamie Blanks

// The part of the RH-7500's CPU interface that runs on the CPU clock. WAIT
// for the chip's own memories and registers lasts a fixed number of CPU
// states, whatever the phase of the video clock. On top of the bus
// controller's two states:
//
//   write, any target                   1   (0 to bitmap VRAM with FBM)
//   read registers, OAM, palette, IO    2
//   read bitmap VRAM                    3   (2 with FBM)
//   read tile VRAM                      5
//
// Sound and expansion strobes stay timed on the video side. A flash write
// still holds bitmap accesses from there.

module vdp_cpu_wait
(
	input  wire         clk,            // clk_sys
	input  wire         ce_cpu,         // one per CPU state

	input  wire         cs_n,
	input  wire         rd_n,
	input  wire         wrh_n,
	input  wire         wrl_n,
	input  wire [19:16] a,

	input  wire         bm_fast,        // video domain, changes rarely

	output wire         timed,          // this address is timed here
	output wire         wait_n
);

	wire acc = ~cs_n & (~rd_n | ~wrh_n | ~wrl_n);
	assign timed = ~a[19] & (a[18:16] != 3'd7);

	(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
	reg [1:0] fast_s;
	always @(posedge clk) fast_s <= {fast_s[0], bm_fast};

	wire       bitmap = (a[19:18] == 2'b00);
	wire       tile   = (a[19:16] == 4'h4);
	wire [2:0] waits  = rd_n   ? ((bitmap & fast_s[1]) ? 3'd0 : 3'd1)
	                  : tile   ? 3'd5
	                  : bitmap ? (fast_s[1] ? 3'd2 : 3'd3)
	                  :          3'd2;

	// States since the strobe fell, T2 being 1. Between two back-to-back
	// accesses the strobe is high for only half a state, so that is caught
	// between enables. Both clear themselves once the bus is idle.
	reg [2:0] n;
	reg       gap;
	always @(posedge clk) begin
		if (ce_cpu) begin
			n   <= !acc ? 3'd0
			     : (gap || (n == 3'd0)) ? 3'd1
			     : (n == 3'd7) ? 3'd7 : n + 3'd1;
			gap <= 1'b0;
		end else if (!acc) begin
			gap <= 1'b1;
		end
	end

	assign wait_n = ~(acc & timed & (n <= waits));

endmodule
