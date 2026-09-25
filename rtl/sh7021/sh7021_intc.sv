// Copyright (c) 2026 Jamie Blanks
//
// SH7021 interrupt controller. Forty request sources in the manual's default
// order: NMI, user break, IRQ0-7, then the on-chip modules. Each carries a
// level from IPRA-IPRE, except NMI at a fixed 16 and user break at a fixed
// 15. The winner is accepted when its level is above SR.I3-I0; NMI ignores
// the mask. The decision costs one register stage (two for an IRQ pin, whose
// sense logic adds the first), matching the manual's 2 and 3 states.

module sh7021_intc (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	// 16-bit on-chip register bus
	input  wire        psel_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,   // registers are 16 bit, so bit 0 is unused
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,

	input  wire        nmi_i,        // pin level, active low request edge
	input  wire [7:0]  irq_i,        // pin levels, active low
	// On-chip requests in the manual's order: UBC, DEI0-3, ITU0-4 as
	// IMIA/IMIB/OVI, SCI0 and SCI1 as ERI/RxI/TxI/TEI, PEI, ITI, CMI.
	input  wire [30:0] onchip_i,

	input  wire [3:0]  sr_mask_i,
	output wire        req_o,
	output wire [7:0]  vec_o,
	output wire [3:0]  level_o,
	input  wire        ack_i,
	output wire        irqout_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	localparam integer NSRC = 40;

	reg [15:0] ipra, iprb, iprc, iprd, ipre;
	reg        nmie;
	reg [7:0]  irqs;                     // 1 = falling edge, 0 = low level
	reg        nmi_d;
	reg [7:0]  irq_d;
	reg        nmi_pend;
	reg [7:0]  irq_pend;

	// NMI is taken on the selected edge; the live edge feeds the decision
	// as well as the latch so NMI stays at two states.
	wire nmi_edge = nmie ? (nmi_i && !nmi_d) : (!nmi_i && nmi_d);

	// ICR holds IRQ0S in bit 7 down to IRQ7S in bit 0.
	wire [7:0]  irqs_icr = {irqs[0], irqs[1], irqs[2], irqs[3],
	                        irqs[4], irqs[5], irqs[6], irqs[7]};
	wire [15:0] icr = {nmi_i, 6'd0, nmie, irqs_icr};

	// ------------------------------------------------------------- registers
	wire [15:0] wd = pwdata_i;
	wire        wr = psel_i && pwr_i;

	always @* begin
		case (paddr_i[8:1])
		8'hC2: prdata_o = ipra;          // H'5FFFF84
		8'hC3: prdata_o = iprb;
		8'hC4: prdata_o = iprc;
		8'hC5: prdata_o = iprd;
		8'hC6: prdata_o = ipre;
		8'hC7: prdata_o = icr;
		default: prdata_o = 16'd0;
		endcase
	end

	function automatic [15:0] wmerge(input [15:0] old, input [15:0] nw,
	                                 input [1:0] be);
		wmerge = {be[1] ? nw[15:8] : old[15:8], be[0] ? nw[7:0] : old[7:0]};
	endfunction

	// ----------------------------------------------------- source properties
	// level and vector per source index, in the manual's default order
	function automatic [4:0] src_level(input integer i);
		begin
			case (i)
			0:  src_level = 5'd16;                       // NMI
			1:  src_level = 5'd15;                       // user break
			2:  src_level = {1'b0, ipra[15:12]};
			3:  src_level = {1'b0, ipra[11:8]};
			4:  src_level = {1'b0, ipra[7:4]};
			5:  src_level = {1'b0, ipra[3:0]};
			6:  src_level = {1'b0, iprb[15:12]};
			7:  src_level = {1'b0, iprb[11:8]};
			8:  src_level = {1'b0, iprb[7:4]};
			9:  src_level = {1'b0, iprb[3:0]};
			10, 11: src_level = {1'b0, iprc[15:12]};     // DEI0, DEI1
			12, 13: src_level = {1'b0, iprc[11:8]};      // DEI2, DEI3
			14, 15, 16: src_level = {1'b0, iprc[7:4]};   // ITU0
			17, 18, 19: src_level = {1'b0, iprc[3:0]};   // ITU1
			20, 21, 22: src_level = {1'b0, iprd[15:12]}; // ITU2
			23, 24, 25: src_level = {1'b0, iprd[11:8]};  // ITU3
			26, 27, 28: src_level = {1'b0, iprd[7:4]};   // ITU4
			29, 30, 31, 32: src_level = {1'b0, iprd[3:0]};   // SCI0
			33, 34, 35, 36: src_level = {1'b0, ipre[15:12]}; // SCI1
			37: src_level = {1'b0, ipre[11:8]};          // PEI
			default: src_level = {1'b0, ipre[7:4]};      // ITI, CMI
			endcase
		end
	endfunction

	function automatic [7:0] src_vector(input integer i);
		begin
			case (i)
			0:  src_vector = 8'd11;
			1:  src_vector = 8'd12;
			2,3,4,5,6,7,8,9: src_vector = 8'd64 + i[7:0] - 8'd2;
			10: src_vector = 8'd72;
			11: src_vector = 8'd74;
			12: src_vector = 8'd76;
			13: src_vector = 8'd78;
			14,15,16: src_vector = 8'd80 + i[7:0] - 8'd14;
			17,18,19: src_vector = 8'd84 + i[7:0] - 8'd17;
			20,21,22: src_vector = 8'd88 + i[7:0] - 8'd20;
			23,24,25: src_vector = 8'd92 + i[7:0] - 8'd23;
			26,27,28: src_vector = 8'd96 + i[7:0] - 8'd26;
			29,30,31,32: src_vector = 8'd100 + i[7:0] - 8'd29;
			33,34,35,36: src_vector = 8'd104 + i[7:0] - 8'd33;
			37: src_vector = 8'd108;
			38: src_vector = 8'd112;
			default: src_vector = 8'd113;
			endcase
		end
	endfunction

	// Assemble the request bits in the documented order.
	reg [NSRC-1:0] reqs;
	integer k;
	always @* begin
		reqs      = {NSRC{1'b0}};
		reqs[0]   = nmi_pend | nmi_edge;
		reqs[1]   = onchip_i[0];
		reqs[9:2] = irq_pend;
		for (k = 10; k < NSRC; k = k + 1)
			reqs[k] = onchip_i[k - 9];
	end

	// ---------------------------------------------------------- reduction
	// Each candidate is {request, level, inverted index}; the largest key
	// wins. A balanced tree, six compares deep for forty sources.
	localparam integer KW = 1 + 5 + 6;

	function automatic [KW-1:0] bigger(input [KW-1:0] x, input [KW-1:0] y);
		bigger = (x > y) ? x : y;
	endfunction

	reg [KW-1:0] k0 [0:63];
	reg [KW-1:0] k1 [0:31];
	reg [KW-1:0] k2 [0:15];
	reg [KW-1:0] k3 [0:7];
	reg [KW-1:0] k4 [0:3];
	reg [KW-1:0] k5 [0:1];
	reg [KW-1:0] win;
	integer t;

	always @(reqs or ipra or iprb or iprc or iprd or ipre) begin
		for (t = 0; t < 64; t = t + 1)
			k0[t] = (t < NSRC) ? {reqs[t], src_level(t), ~t[5:0]} : {KW{1'b0}};
		for (t = 0; t < 32; t = t + 1) k1[t] = bigger(k0[2*t], k0[2*t+1]);
		for (t = 0; t < 16; t = t + 1) k2[t] = bigger(k1[2*t], k1[2*t+1]);
		for (t = 0; t <  8; t = t + 1) k3[t] = bigger(k2[2*t], k2[2*t+1]);
		for (t = 0; t <  4; t = t + 1) k4[t] = bigger(k3[2*t], k3[2*t+1]);
		for (t = 0; t <  2; t = t + 1) k5[t] = bigger(k4[2*t], k4[2*t+1]);
		win = bigger(k5[0], k5[1]);
	end

	wire       win_req   = win[KW-1];
	wire [4:0] win_level = win[KW-2:6];
	wire [5:0] win_idx   = ~win[5:0];
	wire       win_ok    = win_req && ((win_level == 5'd16)
	                                   || (win_level > {1'b0, sr_mask_i}));

	reg [7:0] src_vector_r;
	always @(win_idx) src_vector_r = src_vector({26'd0, win_idx});

	reg        acc_v;
	reg [5:0]  acc_idx;
	reg [3:0]  acc_level;
	reg [7:0]  acc_vec;

	assign req_o    = acc_v;
	assign vec_o    = acc_vec;
	assign level_o  = acc_level;
	assign irqout_o = ~acc_v;

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_INTC0, SS_INTC1;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1;

	wire [63:0] SS_INTC0_BACK = {iprd, iprc, iprb, ipra};
	wire [63:0] SS_INTC1_BACK = {2'd0, acc_vec, acc_level, acc_idx, acc_v,
	                             irq_pend, nmi_pend, irq_d, nmi_d,
	                             irqs, nmie, ipre};

	ss_reg #(.ADDR (SSW_INTC_BASE + 0), .DEFAULT (64'd0)) u_ss0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout0),
		.din_i      (SS_INTC0_BACK),
		.dout_o     (SS_INTC0)
	);
	// The pin history resets high: both request lines are active low.
	ss_reg #(.ADDR (SSW_INTC_BASE + 1),
	         .DEFAULT ({2'd0, 8'd0, 4'd0, 6'd0, 1'b0, 8'd0, 1'b0,
	                    8'hFF, 1'b1, 8'd0, 1'b0, 16'd0}))
	       u_ss1 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout1),
		.din_i (SS_INTC1_BACK), .dout_o (SS_INTC1)
	);
	assign ss_dout = ss_dout0 | ss_dout1;

	// ----------------------------------------------------------------- state
	always @(posedge clk_i) begin
		if (rst_i) begin
			ipra <= SS_INTC0[15:0];  iprb <= SS_INTC0[31:16];
			iprc <= SS_INTC0[47:32]; iprd <= SS_INTC0[63:48];
			ipre <= SS_INTC1[15:0];  nmie <= SS_INTC1[16];
			irqs <= SS_INTC1[24:17];
			nmi_d <= SS_INTC1[25];   irq_d <= SS_INTC1[33:26];
			nmi_pend <= SS_INTC1[34]; irq_pend <= SS_INTC1[42:35];
			acc_v <= SS_INTC1[43];   acc_idx <= SS_INTC1[49:44];
			acc_level <= SS_INTC1[53:50]; acc_vec <= SS_INTC1[61:54];
		end else if (ce_i) begin
			if (wr) case (paddr_i[8:1])
				8'hC2: ipra <= wmerge(ipra, wd, pbe_i);
				8'hC3: iprb <= wmerge(iprb, wd, pbe_i);
				8'hC4: iprc <= wmerge(iprc, wd, pbe_i);
				8'hC5: iprd <= wmerge(iprd, wd, pbe_i);
				8'hC6: ipre <= wmerge(ipre, wd, pbe_i);
				8'hC7: begin
					if (pbe_i[1]) nmie <= wd[8];
					if (pbe_i[0]) irqs <= {wd[0], wd[1], wd[2], wd[3],
					                       wd[4], wd[5], wd[6], wd[7]};
				end
				default: ;
			endcase

			nmi_d <= nmi_i;
			irq_d <= irq_i;

			// NMI is edge only, either edge by NMIE, and is held until taken.
			if (nmi_edge) nmi_pend <= 1'b1;

			// Each IRQ pin is either a level while low, or a latched falling
			// edge held pending for exactly one acceptance.
			for (k = 0; k < 8; k = k + 1) begin
				if (irqs[k]) begin
					if (irq_d[k] && !irq_i[k]) irq_pend[k] <= 1'b1;
				end else begin
					irq_pend[k] <= ~irq_i[k];
				end
			end
			acc_v     <= win_ok;
			acc_idx   <= win_idx;
			acc_level <= (win_level == 5'd16) ? 4'd15 : win_level[3:0];
			acc_vec   <= src_vector_r;

			// The CPU consumes an edge-latched request when it accepts it.
			if (ack_i) begin
				if (acc_idx == 6'd0) nmi_pend <= 1'b0;
				else if (acc_idx >= 6'd2 && acc_idx <= 6'd9)
					if (irqs[acc_idx[2:0] - 3'd2]) irq_pend[acc_idx[2:0] - 3'd2] <= 1'b0;
			end
		end
	end
endmodule
