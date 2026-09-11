// Scanline capture.
//
// A write to TRIGGER with CAP set arms one capture of the line at
// CAPTURE_CTRL.YCAP. The buffer fills pixel by pixel as that line is drawn
// and `ready` pulses at the start of the next line.
//
//   mode 0    blended output, 15bpp, 512 bytes
//   mode 1    screen A, 15bpp
//   mode 2,3  screen A palette indices, 8bpp, 256 bytes; the rest of the
//             buffer is left as is
//
// The buffer is big-endian like the other VDP memories: pixel 0 is the top
// byte of word 0.

module vdp_capture
(
	input  wire clk,
	input  wire reset,

	// Savestate scalar bus.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	input  wire       trig_cap,
	input  wire [1:0] cap_mode,
	input  wire [7:0] cap_line,

	input  wire [8:0] vcount,
	input  wire [7:0] disp_x,
	input  wire [2:0] phase,
	input  wire       in_render,
	input  wire       line_start,

	input  wire [7:0]  idx_a,
	input  wire [14:0] screen_a,
	input  wire [14:0] blended,

	// Write port into the capture buffer.
	output wire [7:0]  wr_addr,
	output wire        wr_en,
	output wire [1:0]  wr_be,
	output wire [15:0] wr_data,

	output reg         ready       // one-cycle pulse, the IRQ2 capture source
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_cap;
	/* verilator lint_on UNUSEDSIGNAL */

	reg armed;
	reg running;

	wire on_line = (vcount == {1'b0, cap_line});
	wire is_8bpp = cap_mode[1];

	// Sample at phase 7, the same point the video output takes the pixel.
	wire take = running & in_render & (phase == 3'd7);

	assign wr_addr = is_8bpp ? {1'b0, disp_x[7:1]} : disp_x;
	assign wr_en   = take;
	assign wr_be   = is_8bpp ? (disp_x[0] ? 2'b01 : 2'b10) : 2'b11;
	assign wr_data = is_8bpp ? {idx_a, idx_a}
	               : {1'b0, (cap_mode == 2'd0) ? blended : screen_a};

	always @(posedge clk) begin
		ready <= 1'b0;

		if (reset) begin
			armed   <= ss_cap[0];
			running <= ss_cap[1];
		end else begin
			if (trig_cap) armed <= 1'b1;

			if (line_start) begin
				if (running) begin
					running <= 1'b0;
					armed   <= 1'b0;
					ready   <= 1'b1;      // the line is in the buffer
				end else if (armed && on_line) begin
					running <= 1'b1;
				end
			end
		end
	end

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"


	ss_reg #(
		.ADDR    (SSW_VDP_CAPTURE),
		.DEFAULT (64'd0)
	) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({62'd0, running, armed}),
		.dout_o     (ss_cap)
	);

endmodule
