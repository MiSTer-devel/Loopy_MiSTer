// Controller port: six outputs, eight inputs, the matrix scan and the mouse
// counters.
//
// In matrix mode the VDP scans the port once per frame. Six phases, each
// starting 32 lines after the last, each holding its output pin high for 13
// lines:
//
//   phase 0 high at VCOUNT 0, low at 13      phase 3 high at 96, low at 109
//   phase 1 high at 32, low at 45            phase 4 high at 128, low at 141
//   phase 2 high at 64, low at 77            phase 5 high at 160, low at 173
//
// The inputs are latched at the start of the last line the output is high.
// In direct mode the outputs come straight from CONTROL_OUT and the first
// input byte shows the live pins; the other bytes hold their last latch.
//
// The mouse puts two quadrature pairs on input pins 0-3 and its buttons on 4
// and 6. With MCNT set the pairs feed signed 12-bit counters that saturate and
// clear on read. Button pins reach the register unchanged, so pressed reads 0.
//
//   0x5D010-0x5D015  CONTROL_IN[0..2], two latches per register
//   0x5D050          mouse X, with the two buttons
//   0x5D052          mouse Y
//   0x5D054          CONTROL_OUT

module vdp_io_ctrl
(
	input  wire clk,
	input  wire reset,

	// Savestate scalar bus, two words.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// Register bus slice.
	input  wire [11:1] addr,
	input  wire        wr,
	input  wire        rd,
	input  wire [1:0]  be,
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,
	output reg         hit,

	input  wire        mode_cmode,   // 1 = matrix scan
	input  wire        mode_mcnt,    // 1 = mouse counters run

	input  wire        line_start,
	input  wire        ce_pix,
	input  wire [8:0]  vcount,

	// Controller port pins.
	output wire [5:0]  ctrl_out,
	input  wire [7:0]  ctrl_in
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_ctrl;
	wire [63:0] ss_mouse;
	/* verilator lint_on UNUSEDSIGNAL */

	// ---- matrix scan ---------------------------------------------------------
	// Phase is VCOUNT / 32; the pin is high for its first 13 lines.

	wire [2:0] scan_phase = vcount[7:5];
	wire       scan_valid = ~vcount[8] & (vcount[8:5] < 4'd6);
	wire       scan_high  = scan_valid & (vcount[4:0] < 5'd13);
	wire       scan_latch = scan_valid & (vcount[4:0] == 5'd12);

	reg [7:0] lat [0:5];
	reg [5:0] direct_out;

	assign ctrl_out = mode_cmode
		? (scan_high ? (6'd1 << scan_phase) : 6'd0)
		: direct_out;

	// ---- mouse quadrature ----------------------------------------------------
	// Pins 0 and 1 are the X pair, 2 and 3 the Y pair. Only a gray-code step
	// of one moves the counter.

	reg [1:0] qx_prev, qy_prev;
	reg signed [11:0] dx, dy;
	reg rd_mx, rd_my;

	function automatic signed [1:0] qstep (input [1:0] prev, input [1:0] cur);
		begin
			case ({prev, cur})
			4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: qstep =  2'sd1;
			4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: qstep = -2'sd1;
			default:                                qstep =  2'sd0;
			endcase
		end
	endfunction

	wire signed [1:0] sx = qstep(qx_prev, ctrl_in[1:0]);
	wire signed [1:0] sy = qstep(qy_prev, ctrl_in[3:2]);

	function automatic signed [11:0] sat (input signed [11:0] v,
	                                      input signed [1:0] step);
		reg signed [12:0] n;
		begin
			n = {v[11], v} + {{11{step[1]}}, step};
			// Floor written in hex: -12'sd2048 overflows the literal before
			// the sign is applied.
			if (n > 13'sd2047)       sat = 12'sh7FF;
			else if (n < -13'sd2048) sat = 12'sh800;
			else                     sat = n[11:0];
		end
	endfunction

	// ---- register bus --------------------------------------------------------

	// 0x010, 0x012 and 0x014, two latches to a register.
	wire sel_in     = (addr[11:2] == 10'h004) | (addr[11:1] == 11'h00A);
	wire sel_mousex = (addr[11:1] == 11'h028);              // 0x050
	wire sel_mousey = (addr[11:1] == 11'h029);              // 0x052
	wire sel_out    = (addr[11:1] == 11'h02A);              // 0x054

	wire [1:0] in_idx = addr[2:1];

	always @* begin
		rdata = 16'd0;
		hit   = 1'b0;
		if (sel_in) begin
			hit = 1'b1;
			// Direct mode shows the live pins in the first byte only; the
			// rest keep what the last matrix scan latched.
			rdata = {lat[{in_idx, 1'b1}],
			         (mode_cmode | (in_idx != 2'd0)) ? lat[{in_idx, 1'b0}] : ctrl_in};
		end else if (sel_mousex) begin
			hit = 1'b1;
			rdata = {1'b0, ctrl_in[6], 1'b0, ctrl_in[4], dx};
		end else if (sel_mousey) begin
			hit = 1'b1;
			rdata = {4'd0, dy};
		end else if (sel_out) begin
			hit = 1'b1;
			rdata = {10'd0, direct_out};
		end
	end

	integer i;
	always @(posedge clk) begin
		if (reset) begin
			for (i = 0; i < 6; i = i + 1) lat[i] <= ss_ctrl[i*8 +: 8];
			direct_out <= ss_ctrl[53:48];
			rd_mx      <= 1'b0;
			rd_my      <= 1'b0;
			qx_prev    <= ss_mouse[25:24];
			qy_prev    <= ss_mouse[27:26];
			dx         <= ss_mouse[11:0];
			dy         <= ss_mouse[23:12];
		end else begin
			if (mode_cmode & line_start & scan_latch) lat[scan_phase] <= ctrl_in;

			// The previous phase advances whenever a step is consumed: once a
			// pixel, and again on a clear.
			if (ce_pix | rd_mx) qx_prev <= ctrl_in[1:0];
			if (ce_pix | rd_my) qy_prev <= ctrl_in[3:2];

			// A read clears its counter one cycle later, when the bus takes
			// the data; a step in that cycle still counts.
			rd_mx <= rd & sel_mousex;
			rd_my <= rd & sel_mousey;

			if (rd_mx) dx <= mode_mcnt ? sat(12'sd0, sx) : 12'sd0;
			else if (ce_pix & mode_mcnt) dx <= sat(dx, sx);

			if (rd_my) dy <= mode_mcnt ? sat(12'sd0, sy) : 12'sd0;
			else if (ce_pix & mode_mcnt) dy <= sat(dy, sy);

			if (wr & be[0] & sel_out) direct_out <= wdata[5:0];
		end
	end

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"

	wire [63:0] ss_dout_c, ss_dout_m;
	assign ss_dout = ss_dout_c | ss_dout_m;

	ss_reg #(.ADDR (SSW_VDP_IOCTRL), .DEFAULT (64'd0)) u_ss_c (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout_c),
		.din_i      ({10'd0, direct_out, lat[5], lat[4], lat[3], lat[2], lat[1], lat[0]}),
		.dout_o     (ss_ctrl)
	);

	ss_reg #(.ADDR (SSW_VDP_IOMOUSE), .DEFAULT (64'd0)) u_ss_m (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout_m),
		.din_i      ({36'd0, qy_prev, qx_prev, dy, dx}),
		.dout_o     (ss_mouse)
	);

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire _unused = &{1'b0, be[1], wdata[15:6]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

endmodule
