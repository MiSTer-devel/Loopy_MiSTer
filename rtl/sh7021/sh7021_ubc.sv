// Copyright (c) 2026 Jamie Blanks
//
// SH7021 user break controller. BARH H'5FFFF90, BARL H'5FFFF92, BAMRH
// H'5FFFF94, BAMRL H'5FFFF96 and BBR H'5FFFF98, all 16 bits, all reset to
// zero. A zero BBR field never matches, so reset leaves the break disabled.

module sh7021_ubc (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	// Bit 0 picks which byte of the register the BSC returns.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	// the bus cycle being watched
	input  wire        cyc_i,
	input  wire [31:0] cyc_addr_i,
	input  wire        cyc_we_i,
	input  wire        cyc_ifetch_i,
	input  wire        cyc_dma_i,
	input  wire [1:0]  cyc_sz_i,

	output wire        brk_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [15:0] barh, barl, bamrh, bamrl, bbr;

	wire [7:0] a = paddr_i[8:1];
	assign phit_o = psel_i && (a >= 8'hC8) && (a <= 8'hCC);

	always @* begin
		case (a)
		8'hC8:   prdata_o = barh;
		8'hC9:   prdata_o = barl;
		8'hCA:   prdata_o = bamrh;
		8'hCB:   prdata_o = bamrl;
		default: prdata_o = bbr;
		endcase
	end

	// The break request is registered to keep the bus decode out of the
	// INTC's priority tree.
	reg         brk;
	assign brk_o = brk;

	wire [31:0] bar  = {barh, barl};
	wire [31:0] bamr = {bamrh, bamrl};
	wire addr_match = ((cyc_addr_i ^ bar) & ~bamr) == 32'd0;
	// CD0 breaks on CPU cycles and CD1 on DMA cycles; RW0 on reads and RW1 on
	// writes; ID0 on instruction fetches and ID1 on data accesses.
	wire cd_match   = cyc_dma_i    ? bbr[7] : bbr[6];
	wire id_match   = cyc_ifetch_i ? bbr[4] : bbr[5];
	wire rw_match   = cyc_we_i     ? bbr[3] : bbr[2];
	wire sz_match   = (bbr[1:0] == 2'b00) ? 1'b1
	                : (bbr[1:0] == 2'b01) ? (cyc_sz_i == 2'd0)
	                : (bbr[1:0] == 2'b10) ? (cyc_sz_i == 2'd1)
	                :                       (cyc_sz_i == 2'd2);

	wire brk_now = cyc_i && addr_match && cd_match && id_match && rw_match
	               && sz_match;

	function automatic [15:0] merge(input [15:0] old, input [15:0] nw,
	                                input [1:0] lanes);
		merge = {lanes[1] ? nw[15:8] : old[15:8],
		         lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_UBC0, SS_UBC1;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1;
	wire [63:0] SS_UBC0_BACK = {bamrl, bamrh, barl, barh};
	wire [63:0] SS_UBC1_BACK = {47'd0, brk, bbr};

	ss_reg #(.ADDR (SSW_UBC_BASE + 0), .DEFAULT (64'd0)) u_ss0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout0),
		.din_i      (SS_UBC0_BACK),
		.dout_o     (SS_UBC0)
	);
	ss_reg #(.ADDR (SSW_UBC_BASE + 1), .DEFAULT (64'd0)) u_ss1 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout1),
		.din_i      (SS_UBC1_BACK),
		.dout_o     (SS_UBC1)
	);
	assign ss_dout = ss_dout0 | ss_dout1;

	always @(posedge clk_i) begin
		if (rst_i) begin
			barh  <= SS_UBC0[15:0];  barl  <= SS_UBC0[31:16];
			bamrh <= SS_UBC0[47:32]; bamrl <= SS_UBC0[63:48];
			bbr   <= SS_UBC1[15:0];
			brk   <= SS_UBC1[16];
		end else if (ce_i) begin
			brk <= brk_now;
			if (psel_i && pwr_i) case (a)
			8'hC8: barh  <= merge(barh,  pwdata_i, pbe_i);
			8'hC9: barl  <= merge(barl,  pwdata_i, pbe_i);
			8'hCA: bamrh <= merge(bamrh, pwdata_i, pbe_i);
			8'hCB: bamrl <= merge(bamrl, pwdata_i, pbe_i);
			8'hCC: bbr   <= merge(bbr, pwdata_i, pbe_i) & 16'h00FF;
			default: ;
			endcase
		end
	end
endmodule
