// Copyright (c) 2026 Jamie Blanks
//
// SH7021 programmable timing pattern controller. Port B is split into four
// groups of four pins; each group picks an ITU channel, and on that channel's
// compare match the group's nibble of the next-data register is copied to
// port B. Only pins enabled in NDERA/NDERB are driven. TPMR is a plain
// register; non-overlap mode has no effect.

module sh7021_tpc (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	// Bit 0 picks which byte of the 16-bit register pair the BSC returns.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	input  wire [3:0]  itu_cma_i,      // compare match A, ITU channels 0-3
	output reg  [15:0] tpc_dr_o,       // pattern presented to port B
	output wire [15:0] tpc_en_o,       // which port B bits the TPC drives

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [3:0]  tpmr;
	reg [7:0]  tpcr;
	reg [7:0]  ndera, nderb;
	reg [7:0]  ndra, ndrb;

	wire [7:0] a = paddr_i[8:1];
	assign phit_o   = psel_i && (a >= 8'hF8) && (a <= 8'hFB);
	assign tpc_en_o = {nderb, ndera};

	// Groups sharing a next-data register share one address only while they
	// share a trigger; with different triggers each group has its own address
	// and the other nibble reads all ones.
	wire same_a = (tpcr[3:2] == tpcr[1:0]);
	wire same_b = (tpcr[7:6] == tpcr[5:4]);

	always @* begin
		case (a)
		8'hF8:   prdata_o = {4'hF, tpmr, tpcr};        // H'5FFFFF0 / F1
		8'hF9:   prdata_o = {nderb, ndera};            // H'5FFFFF2 / F3
		8'hFA:   prdata_o = {same_b ? ndrb : {ndrb[7:4], 4'hF},
		                     same_a ? ndra : {ndra[7:4], 4'hF}};
		default: prdata_o = {same_b ? 8'hFF : {4'hF, ndrb[3:0]},
		                     same_a ? 8'hFF : {4'hF, ndra[3:0]}};
		endcase
	end

	// Group triggers: two bits each in TPCR pick an ITU channel.
	wire [3:0] trig = {itu_cma_i[tpcr[7:6]], itu_cma_i[tpcr[5:4]],
	                   itu_cma_i[tpcr[3:2]], itu_cma_i[tpcr[1:0]]};

	function automatic [7:0] merge8(input [7:0] old, input [7:0] nw,
	                                input lane);
		merge8 = lane ? nw : old;
	endfunction

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_TPC;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] SS_TPC_BACK = {4'd0, tpc_dr_o, ndrb, ndra, nderb, ndera,
	                           tpcr, tpmr};

	ss_reg #(.ADDR (SSW_TPC_BASE), .DEFAULT ({52'd0, 8'hFF, 4'h0})) u_ss (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      (SS_TPC_BACK),
		.dout_o     (SS_TPC)
	);

	always @(posedge clk_i) begin
		if (rst_i) begin
			tpmr  <= SS_TPC[3:0];   tpcr  <= SS_TPC[11:4];
			ndera <= SS_TPC[19:12]; nderb <= SS_TPC[27:20];
			ndra  <= SS_TPC[35:28]; ndrb  <= SS_TPC[43:36];
			tpc_dr_o <= SS_TPC[59:44];
		end else if (ce_i) begin
			if (psel_i && pwr_i) case (a)
				8'hF8: begin
					if (pbe_i[1]) tpmr <= pwdata_i[11:8];
					if (pbe_i[0]) tpcr <= pwdata_i[7:0];
				end
				8'hF9: begin
					nderb <= merge8(nderb, pwdata_i[15:8], pbe_i[1]);
					ndera <= merge8(ndera, pwdata_i[7:0],  pbe_i[0]);
				end
				8'hFA: begin
					if (pbe_i[1])
						ndrb <= same_b ? pwdata_i[15:8]
						               : {pwdata_i[15:12], ndrb[3:0]};
					if (pbe_i[0])
						ndra <= same_a ? pwdata_i[7:0]
						               : {pwdata_i[7:4], ndra[3:0]};
				end
				8'hFB: begin
					if (pbe_i[1] && !same_b) ndrb[3:0] <= pwdata_i[11:8];
					if (pbe_i[0] && !same_a) ndra[3:0] <= pwdata_i[3:0];
				end
				default: ;
			endcase

			// One nibble moves per triggered group.
			if (trig[0]) tpc_dr_o[3:0]   <= ndra[3:0];
			if (trig[1]) tpc_dr_o[7:4]   <= ndra[7:4];
			if (trig[2]) tpc_dr_o[11:8]  <= ndrb[3:0];
			if (trig[3]) tpc_dr_o[15:12] <= ndrb[7:4];
		end
	end
endmodule
