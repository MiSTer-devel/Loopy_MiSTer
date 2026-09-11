// Copyright (c) 2026 Jamie Blanks
//
// SH7021 watchdog timer.
//
// TCNT and TCSR share the write address H'5FFFFB8 and are told apart by the
// upper byte of the word written: H'5A selects TCNT, H'A5 selects TCSR. They
// have separate read addresses, and only byte reads are defined. RSTCSR at
// H'5FFFFBA takes H'A5 to clear WOVF and H'5A to set RSTE and RSTS.

module sh7021_wdt (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,          // external reset only

	input  wire        psel_i,
	// Bit 0 picks the byte the BSC returns: TCSR at H'5FFFFB8, TCNT at B9,
	// RSTCSR at BB.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	output wire        iti_o,          // interval timer interrupt
	output wire        wdtovf_n_o,     // 128 clocks low on a watchdog overflow
	output wire        int_reset_o,    // 512 clocks of internal reset
	output wire        int_reset_manual_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	reg [7:0]  tcnt;
	reg        ovf, wtit, tme;
	reg [2:0]  cks;
	reg        wovf, rste, rsts;
	reg [12:0] pre;
	reg [8:0]  ovf_cnt;                // WDTOVF pin, 128 clocks
	reg [9:0]  rst_cnt;                // internal reset, 512 clocks

	wire [7:0] tcsr   = {ovf, wtit, tme, 2'b11, cks};
	wire [7:0] rstcsr = {wovf, rste, rsts, 5'b11111};
	wire [7:0] a      = paddr_i[8:1];

	assign phit_o     = psel_i && ((a == 8'hDC) || (a == 8'hDD));
	assign iti_o      = ovf;
	assign wdtovf_n_o = (ovf_cnt == 9'd0);
	assign int_reset_o = (rst_cnt != 10'd0);
	assign int_reset_manual_o = rsts;

	always @* begin
		case (a)
		8'hDC:   prdata_o = {tcsr, tcnt};       // H'5FFFFB8 / H'5FFFFB9
		default: prdata_o = {8'd0, rstcsr};     // H'5FFFFBA / H'5FFFFBB
		endcase
	end

	reg tick;
	always @* begin
		case (cks)
		3'd0:    tick = (pre[0]    == 1'b0);    // phi/2
		3'd1:    tick = (pre[5:0]  == 6'd0);    // phi/64
		3'd2:    tick = (pre[6:0]  == 7'd0);
		3'd3:    tick = (pre[7:0]  == 8'd0);
		3'd4:    tick = (pre[8:0]  == 9'd0);
		3'd5:    tick = (pre[9:0]  == 10'd0);
		3'd6:    tick = (pre[11:0] == 12'd0);
		default: tick = (pre       == 13'd0);
		endcase
	end

	wire wr = ce_i && psel_i && pwr_i && (pbe_i == 2'b11);
	wire [7:0] wd = pwdata_i[7:0];

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_WDT;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] SS_WDT_BACK = {15'd0, rst_cnt, ovf_cnt, pre,
	                           rsts, rste, wovf, cks, tme, wtit, ovf, tcnt};

	ss_reg #(.ADDR (SSW_WDT_BASE), .DEFAULT (64'd0)) u_ss (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      (SS_WDT_BACK),
		.dout_o     (SS_WDT)
	);

	always @(posedge clk_i) begin
		if (rst_i) begin
			tcnt <= SS_WDT[7:0];   ovf  <= SS_WDT[8];     wtit <= SS_WDT[9];
			tme  <= SS_WDT[10];    cks  <= SS_WDT[13:11];
			wovf <= SS_WDT[14];    rste <= SS_WDT[15];    rsts <= SS_WDT[16];
			pre  <= SS_WDT[29:17]; ovf_cnt <= SS_WDT[38:30];
			rst_cnt <= SS_WDT[48:39];
		end else if (ce_i) begin
			pre <= pre + 13'd1;
			if (ovf_cnt != 9'd0) ovf_cnt <= ovf_cnt - 9'd1;
			if (rst_cnt != 10'd0) rst_cnt <= rst_cnt - 10'd1;

			if (tme && tick) begin
				tcnt <= tcnt + 8'd1;
				if (tcnt == 8'hFF) begin
					if (wtit) begin
						wovf    <= 1'b1;
						ovf_cnt <= 9'd128;
						if (rste) rst_cnt <= 10'd512;
						else begin
							// With RSTE clear only the WDT's own TCNT and
							// TCSR reset.
							tme  <= 1'b0;
							wtit <= 1'b0;
							cks  <= 3'd0;
						end
					end else begin
						ovf <= 1'b1;
					end
				end
			end

			if (wr) begin
				if (a == 8'hDC) begin
					case (pwdata_i[15:8])
					8'h5A: tcnt <= wd;
					8'hA5: begin
						if (!wd[7]) ovf <= 1'b0;   // read as 1 then write 0
						wtit <= wd[6];
						tme  <= wd[5];
						cks  <= wd[2:0];
						if (!wd[5]) tcnt <= 8'h00; // clearing TME clears TCNT
					end
					default: ;
					endcase
				end else if (a == 8'hDD) begin
					case (pwdata_i[15:8])
					8'hA5: if (!wd[7]) wovf <= 1'b0;
					8'h5A: begin rste <= wd[6]; rsts <= wd[5]; end
					default: ;
					endcase
				end
			end
		end
	end
endmodule
