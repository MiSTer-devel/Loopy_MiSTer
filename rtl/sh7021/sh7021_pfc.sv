// Copyright (c) 2026 Jamie Blanks
//
// SH7021 pin function controller and I/O ports A and B. Each port bit gets a
// two-bit function code; the one-bit fields (PA8, PA4-PA7) are widened to
// the same form, 0 for the port and 1 for the alternate function.

module sh7021_pfc (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,   // registers are 16 bit, so bit 0 is unused
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	input  wire [15:0] pa_i,          // pin levels
	input  wire [15:0] pb_i,
	output wire [15:0] pa_dr_o,       // port data register value
	output wire [15:0] pb_dr_o,
	output wire [15:0] pa_dir_o,      // 1 = output when the pin is a port bit
	output wire [15:0] pb_dir_o,
	output wire [31:0] pa_fn_o,       // two bits per pin, PA0 in [1:0]
	output wire [31:0] pb_fn_o,
	output wire [1:0]  cs1_fn_o,      // CASCR: 01 CS1, 10 CASH
	output wire [1:0]  cs3_fn_o,      // CASCR: 01 CS3, 10 CASL
	// What port B would drive, before the read-back mix with the pin levels.
	output wire [15:0] pb_out_o,

	// The TPC writes port B through the next-data registers.
	input  wire [15:0] tpc_dr_i,
	input  wire [15:0] tpc_en_i,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [15:0] padr, pbdr, paior, pbior, pacr1, pacr2, pbcr1, pbcr2, cascr;

	wire [7:0] a = paddr_i[8:1];
	assign phit_o = psel_i && ((a >= 8'hE0 && a <= 8'hE7) || (a == 8'hF7));

	always @* begin
		case (a)
		8'hE0:   prdata_o = pa_dr_o;
		8'hE1:   prdata_o = pb_dr_o;
		8'hE2:   prdata_o = paior;
		8'hE3:   prdata_o = pbior;
		8'hE4:   prdata_o = pacr1;
		8'hE5:   prdata_o = pacr2;
		8'hE6:   prdata_o = pbcr1;
		8'hE7:   prdata_o = pbcr2;
		8'hF7:   prdata_o = cascr;
		default: prdata_o = 16'd0;
		endcase
	end

	// Reading a port returns the pin for an input bit and the register for an
	// output bit.
	wire [15:0] pb_out = (pbdr & ~tpc_en_i) | (tpc_dr_i & tpc_en_i);
	assign pb_out_o = pb_out;
	assign pa_dr_o  = (padr   & pa_dir_o) | (pa_i & ~pa_dir_o);
	assign pb_dr_o  = (pb_out & pb_dir_o) | (pb_i & ~pb_dir_o);
	assign pa_dir_o = paior;
	assign pb_dir_o = pbior;

	// PACR1 carries two-bit fields for PA15-PA9 and a one-bit field for PA8;
	// PACR2 two-bit fields for PA3-PA0 and one-bit fields for PA7-PA4.
	assign pa_fn_o = {pacr1[15:2],                       // PA15..PA9
	                  1'b0, pacr1[0],                    // PA8
	                  1'b0, pacr2[14],                   // PA7
	                  1'b0, pacr2[12],                   // PA6
	                  1'b0, pacr2[10],                   // PA5
	                  1'b0, pacr2[8],                    // PA4
	                  pacr2[7:0]};                       // PA3..PA0
	assign pb_fn_o = {pbcr1, pbcr2};

	assign cs1_fn_o = cascr[15:14];
	assign cs3_fn_o = cascr[13:12];

	function automatic [15:0] merge(input [15:0] old, input [15:0] nw,
	                                input [1:0] lanes);
		merge = {lanes[1] ? nw[15:8] : old[15:8],
		         lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_PFC0, SS_PFC1, SS_PFC2;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1, ss_dout2;

	wire [63:0] SS_PFC0_BACK = {pbior, paior, pbdr,  padr};
	wire [63:0] SS_PFC1_BACK = {pbcr2, pbcr1, pacr2, pacr1};
	wire [63:0] SS_PFC2_BACK = {48'd0, cascr};

	ss_reg #(.ADDR (SSW_PFC_BASE + 0), .DEFAULT (64'd0)) u_ss0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout0),
		.din_i      (SS_PFC0_BACK),
		.dout_o     (SS_PFC0)
	);
	ss_reg #(.ADDR (SSW_PFC_BASE + 1),
	         .DEFAULT ({16'h0000, 16'h0000, 16'hFF95, 16'h3302})) u_ss1 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout1),
		.din_i (SS_PFC1_BACK), .dout_o (SS_PFC1)
	);
	ss_reg #(.ADDR (SSW_PFC_BASE + 2),
	         .DEFAULT ({48'd0, 16'h5FFF})) u_ss2 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout2),
		.din_i (SS_PFC2_BACK), .dout_o (SS_PFC2)
	);
	assign ss_dout = ss_dout0 | ss_dout1 | ss_dout2;

	always @(posedge clk_i) begin
		if (rst_i) begin
			padr  <= SS_PFC0[15:0];  pbdr  <= SS_PFC0[31:16];
			paior <= SS_PFC0[47:32]; pbior <= SS_PFC0[63:48];
			pacr1 <= SS_PFC1[15:0];  pacr2 <= SS_PFC1[31:16];
			pbcr1 <= SS_PFC1[47:32]; pbcr2 <= SS_PFC1[63:48];
			cascr <= SS_PFC2[15:0];
		end else if (ce_i && psel_i && pwr_i) begin
			case (a)
			8'hE0: padr  <= merge(padr,  pwdata_i, pbe_i);
			8'hE1: pbdr  <= merge(pbdr,  pwdata_i, pbe_i);
			8'hE2: paior <= merge(paior, pwdata_i, pbe_i);
			8'hE3: pbior <= merge(pbior, pwdata_i, pbe_i);
			8'hE4: pacr1 <= merge(pacr1, pwdata_i, pbe_i) | 16'h0002;
			8'hE5: pacr2 <= merge(pacr2, pwdata_i, pbe_i) | 16'hAA00;
			8'hE6: pbcr1 <= merge(pbcr1, pwdata_i, pbe_i);
			8'hE7: pbcr2 <= merge(pbcr2, pwdata_i, pbe_i);
			8'hF7: cascr <= merge(cascr, pwdata_i, pbe_i) | 16'h0FFF;
			default: ;
			endcase
		end
	end
endmodule
