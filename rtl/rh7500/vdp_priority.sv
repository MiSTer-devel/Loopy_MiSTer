// Layer priority: which of the eight layers is visible on each screen.
//
// The order is twelve slots, front to back, with SCREEN_CTRL.PRIO placing the
// layers:
//
//   OBJ1 is always slot 2                BG1 is always slot 11
//   OBJ0 is slot 1, 6, 9 or 12           by PRIO[3:2]
//   BG0  is slot 10 or 3                 by PRIO[1]
//   BM0 and BM1 are slots 4,5 or 7,8     by PRIO[0]
//   BM2 and BM3 take the other pair
//
// Of the layers enabled, routed to the screen and not transparent, the lowest
// slot wins; slot 15 marks a layer out of the running. Bitmap pairs and object
// layers route by LAYER_CTRL (upper bit screen A, lower bit screen B);
// backgrounds route per tile by the SCRN bit of the tilemap entry.
//
// The argmin is a three-level tournament with a register in the middle.

module vdp_priority
(
	input  wire clk,
	input  wire reset,
	input  wire [2:0] phase,

	input  wire [3:0]  prio,
	input  wire [15:0] layer_ctrl,

	input  wire [7:0] bm0, bm1, bm2, bm3,
	input  wire [7:0] bg0, bg1,
	input  wire       bg0_scrn, bg1_scrn,
	input  wire [7:0] obj0, obj1,

	output reg  [7:0] idx_a,
	output reg  [7:0] idx_b
);

	// ---- slot numbers --------------------------------------------------------

	wire [3:0] s_obj1 = 4'd2;
	wire [3:0] s_bg1  = 4'd11;
	reg  [3:0] s_obj0;
	always @* begin
		case (prio[3:2])
		2'd0:    s_obj0 = 4'd1;
		2'd1:    s_obj0 = 4'd6;
		2'd2:    s_obj0 = 4'd9;
		default: s_obj0 = 4'd12;
		endcase
	end
	wire [3:0] s_bg0 = prio[1] ? 4'd3 : 4'd10;
	wire [3:0] s_bm0 = prio[0] ? 4'd7 : 4'd4;
	wire [3:0] s_bm1 = prio[0] ? 4'd8 : 4'd5;
	wire [3:0] s_bm2 = prio[0] ? 4'd4 : 4'd7;
	wire [3:0] s_bm3 = prio[0] ? 4'd5 : 4'd8;

	// ---- enables and routing -------------------------------------------------

	wire en_bg0  = layer_ctrl[0];
	wire en_bg1  = layer_ctrl[1];
	wire en_bm0  = layer_ctrl[2];
	wire en_bm1  = layer_ctrl[3];
	wire en_bm2  = layer_ctrl[4];
	wire en_bm3  = layer_ctrl[5];
	wire en_obj0 = layer_ctrl[6];
	wire en_obj1 = layer_ctrl[7];

	wire [1:0] sc_bm01 = layer_ctrl[9:8];
	wire [1:0] sc_bm23 = layer_ctrl[11:10];
	wire [1:0] sc_obj0 = layer_ctrl[13:12];
	wire [1:0] sc_obj1 = layer_ctrl[15:14];

	// Eight candidates per screen, each a slot and a palette index. A layer
	// that is off, routed elsewhere or transparent gets slot 15 and loses.
	wire [11:0] cand_a [0:7];
	wire [11:0] cand_b [0:7];

	`define CAND(EN, SC, SLOT, PIX) \
		{((EN) && (SC) && ((PIX) != 8'd0)) ? (SLOT) : 4'd15, (PIX)}

	assign cand_a[0] = `CAND(en_bm0,  sc_bm01[1], s_bm0, bm0);
	assign cand_a[1] = `CAND(en_bm1,  sc_bm01[1], s_bm1, bm1);
	assign cand_a[2] = `CAND(en_bm2,  sc_bm23[1], s_bm2, bm2);
	assign cand_a[3] = `CAND(en_bm3,  sc_bm23[1], s_bm3, bm3);
	assign cand_a[4] = `CAND(en_bg0,  ~bg0_scrn,  s_bg0, bg0);
	assign cand_a[5] = `CAND(en_bg1,  ~bg1_scrn,  s_bg1, bg1);
	assign cand_a[6] = `CAND(en_obj0, sc_obj0[1], s_obj0, obj0);
	assign cand_a[7] = `CAND(en_obj1, sc_obj1[1], s_obj1, obj1);

	assign cand_b[0] = `CAND(en_bm0,  sc_bm01[0], s_bm0, bm0);
	assign cand_b[1] = `CAND(en_bm1,  sc_bm01[0], s_bm1, bm1);
	assign cand_b[2] = `CAND(en_bm2,  sc_bm23[0], s_bm2, bm2);
	assign cand_b[3] = `CAND(en_bm3,  sc_bm23[0], s_bm3, bm3);
	assign cand_b[4] = `CAND(en_bg0,  bg0_scrn,   s_bg0, bg0);
	assign cand_b[5] = `CAND(en_bg1,  bg1_scrn,   s_bg1, bg1);
	assign cand_b[6] = `CAND(en_obj0, sc_obj0[0], s_obj0, obj0);
	assign cand_b[7] = `CAND(en_obj1, sc_obj1[0], s_obj1, obj1);

	`undef CAND

	// ---- tournament ----------------------------------------------------------

	function automatic [11:0] front (input [11:0] x, input [11:0] y);
		front = (x[11:8] <= y[11:8]) ? x : y;
	endfunction

	wire [11:0] a01 = front(cand_a[0], cand_a[1]);
	wire [11:0] a23 = front(cand_a[2], cand_a[3]);
	wire [11:0] a45 = front(cand_a[4], cand_a[5]);
	wire [11:0] a67 = front(cand_a[6], cand_a[7]);
	wire [11:0] b01 = front(cand_b[0], cand_b[1]);
	wire [11:0] b23 = front(cand_b[2], cand_b[3]);
	wire [11:0] b45 = front(cand_b[4], cand_b[5]);
	wire [11:0] b67 = front(cand_b[6], cand_b[7]);

	reg [11:0] a_lo, a_hi, b_lo, b_hi;
	always @(posedge clk) begin
		if (reset) begin
			a_lo <= 12'hF00; a_hi <= 12'hF00;
			b_lo <= 12'hF00; b_hi <= 12'hF00;
		end else if (phase == 3'd2) begin
			a_lo <= front(a01, a23);
			a_hi <= front(a45, a67);
			b_lo <= front(b01, b23);
			b_hi <= front(b45, b67);
		end
	end

	wire [11:0] win_a = front(a_lo, a_hi);
	wire [11:0] win_b = front(b_lo, b_hi);

	always @(posedge clk) begin
		if (reset) begin
			idx_a <= 8'd0;
			idx_b <= 8'd0;
		end else if (phase == 3'd3) begin
			idx_a <= (win_a[11:8] == 4'd15) ? 8'd0 : win_a[7:0];
			idx_b <= (win_b[11:8] == 4'd15) ? 8'd0 : win_b[7:0];
		end
	end

endmodule
