// MiSTer inputs onto the Loopy's controller port pins.
//
// The port is six outputs and eight inputs, scanned by the VDP. What hangs
// off it decides what the inputs mean, so this module is the pad or the mouse.
//
// Gamepad, multitap layout. The row a scan phase drives selects which eight
// bits appear:
//
//   row 0   P1 DET ST TL TR in bits 0-3, P2 in bits 4-7
//   row 1   P1 A D C B
//   row 2   P1 up down left right
//   rows 3-5 the same three rows again for P3 and P4
//
// Several output pins high at once or their rows together, like a real
// switch matrix. DET is high for a pad that is plugged in.
//
// Mouse. Ignores the outputs and presents the same eight bits all the time:
// two quadrature pairs on 0-3, buttons on 4 and 6 active low, detect on 7.
// The VDP decodes the pairs into its own delta counters, so a host mouse
// report becomes a run of gray-code steps paid out one every eight pixel
// clocks (about 670 kHz): fast enough to clear a 255-count report inside a
// frame, slow enough for the VDP to see every edge.

module loopy_input
(
	input  wire clk,
	input  wire reset,
	input  wire ce_pix,

	input  wire        use_mouse,     // OSD device select

	// Four pads, active high: start, TL, TR, A, D, C, B, up, down, left,
	// right, from bit 0 up. Detect is separate because it says a pad is
	// plugged in rather than that a button is down.
	input  wire [10:0] pad0,
	input  wire [10:0] pad1,
	input  wire [10:0] pad2,
	input  wire [10:0] pad3,
	input  wire [3:0]  pad_present,

	// One host mouse report: signed movement and the two buttons.
	input  wire        mouse_stb,
	input  wire signed [8:0] mouse_dx,
	input  wire signed [8:0] mouse_dy,
	input  wire        mouse_l,
	input  wire        mouse_r,

	input  wire [5:0]  ctrl_out,
	output wire [7:0]  ctrl_in
);

	// ---- gamepad matrix ------------------------------------------------------

	// An unplugged pad drives nothing, so its whole nibble reads zero.
	function automatic [3:0] pad_row (input [10:0] pad, input present,
	                                  input [1:0] row);
		begin
			case (row)
			2'd0:    pad_row = {pad[2],  pad[1], pad[0], 1'b1};
			2'd1:    pad_row = {pad[6],  pad[5], pad[4], pad[3]};
			default: pad_row = {pad[10], pad[9], pad[8], pad[7]};
			endcase
			if (!present) pad_row = 4'd0;
		end
	endfunction

	wire [7:0] row [0:5];
	genvar r;
	generate
		for (r = 0; r < 3; r = r + 1) begin : g_rows
			assign row[r]   = {pad_row(pad1, pad_present[1], r[1:0]),
			                   pad_row(pad0, pad_present[0], r[1:0])};
			assign row[r+3] = {pad_row(pad3, pad_present[3], r[1:0]),
			                   pad_row(pad2, pad_present[2], r[1:0])};
		end
	endgenerate

	wire [7:0] matrix = (ctrl_out[0] ? row[0] : 8'd0)
	                  | (ctrl_out[1] ? row[1] : 8'd0)
	                  | (ctrl_out[2] ? row[2] : 8'd0)
	                  | (ctrl_out[3] ? row[3] : 8'd0)
	                  | (ctrl_out[4] ? row[4] : 8'd0)
	                  | (ctrl_out[5] ? row[5] : 8'd0);

	// ---- mouse quadrature ----------------------------------------------------

	reg signed [11:0] accx, accy;
	reg [1:0] qx, qy;
	reg [2:0] tick;

	// 00, 01, 11, 10 forwards; the VDP decodes the same order.
	function automatic [1:0] gray_step (input [1:0] cur, input up);
		begin
			case ({cur, up})
			3'b00_1: gray_step = 2'b01;
			3'b01_1: gray_step = 2'b11;
			3'b11_1: gray_step = 2'b10;
			3'b10_1: gray_step = 2'b00;
			3'b00_0: gray_step = 2'b10;
			3'b10_0: gray_step = 2'b11;
			3'b11_0: gray_step = 2'b01;
			default: gray_step = 2'b00;
			endcase
		end
	endfunction

	// A new report adds to whatever is still being paid out, so fast movement
	// accumulates instead of being dropped. The sum stops at the ends of the
	// range: wrapping there would send the pointer the other way.
	function automatic signed [11:0] acc_add (input signed [11:0] acc,
	                                          input signed [8:0] d);
		reg signed [12:0] n;
		begin
			n = {acc[11], acc} + {{4{d[8]}}, d};
			if (n > 13'sd2047)       acc_add = 12'sh7FF;
			else if (n < -13'sd2048) acc_add = 12'sh800;
			else                     acc_add = n[11:0];
		end
	endfunction

	// A report and a payout can land on the same cycle, so the report is folded
	// in first and the payout works from the sum rather than waiting a tick.
	wire signed [11:0] accx_n = mouse_stb ? acc_add(accx, mouse_dx) : accx;
	wire signed [11:0] accy_n = mouse_stb ? acc_add(accy, mouse_dy) : accy;
	wire pay = ce_pix & (tick == 3'd7);

	always @(posedge clk) begin
		if (reset) begin
			accx <= 12'sd0;
			accy <= 12'sd0;
			qx   <= 2'b00;
			qy   <= 2'b00;
			tick <= 3'd0;
		end else begin
			if (ce_pix) tick <= tick + 3'd1;

			if (pay & (accx_n != 12'sd0)) begin
				qx   <= gray_step(qx, ~accx_n[11]);
				accx <= accx_n[11] ? (accx_n + 12'sd1) : (accx_n - 12'sd1);
			end else begin
				accx <= accx_n;
			end

			if (pay & (accy_n != 12'sd0)) begin
				qy   <= gray_step(qy, ~accy_n[11]);
				accy <= accy_n[11] ? (accy_n + 12'sd1) : (accy_n - 12'sd1);
			end else begin
				accy <= accy_n;
			end
		end
	end

	wire [7:0] mouse_bits = {1'b1, ~mouse_r, 1'b0, ~mouse_l, qy, qx};

	assign ctrl_in = use_mouse ? mouse_bits : matrix;

endmodule
