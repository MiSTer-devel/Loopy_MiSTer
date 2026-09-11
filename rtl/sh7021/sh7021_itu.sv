// Copyright (c) 2026 Jamie Blanks
//
// SH7021 16-bit integrated timer pulse unit, five channels. A compare match
// is signalled in the state in which the counter would move off the matching
// value, so a channel cleared on compare match with GR = N has a period of
// N+1 counter clocks. TFCR.CMD and TOCR.OLS (complementary and
// reset-synchronised PWM) are plain registers.

module sh7021_itu (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	// Bit 0 picks the byte of the register pair; the BSC decodes it.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [8:0]  paddr_i,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	input  wire [3:0]  tclk_i,        // TCLKA, TCLKB, TCLKC, TCLKD
	input  wire [4:0]  tioca_i,       // capture inputs
	input  wire [4:0]  tiocb_i,
	output reg  [4:0]  tioca_o,
	output reg  [4:0]  tiocb_o,
	output wire [4:0]  tioca_oe_o,
	output wire [4:0]  tiocb_oe_o,

	output wire [4:0]  imia_o,
	output wire [4:0]  imib_o,
	output wire [4:0]  ovi_o,
	output wire [3:0]  cma_o,         // compare match A, for the TPC
	input  wire [3:0]  dmac_ack_i,    // IMIA cleared by a DMAC start, ch 0-3

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	integer i;

	reg [4:0]  str, sync, pwm;
	reg        mdf, fdir;
	reg [1:0]  cmd;
	reg [3:0]  bf;                    // BFB4, BFA4, BFB3, BFA3
	reg [1:0]  ols;

	reg [7:0]  tcr  [0:4];
	reg [7:0]  tior [0:4];
	reg [2:0]  tier [0:4];
	reg [2:0]  tsr  [0:4];
	reg [15:0] tcnt [0:4];
	reg [15:0] gra  [0:4];
	reg [15:0] grb  [0:4];
	reg [15:0] bra  [0:4];
	reg [15:0] brb  [0:4];

	reg [2:0]  psc;
	reg [3:0]  tclk_d;
	reg [4:0]  tioca_d, tiocb_d;

	wire [7:0] w = paddr_i[8:1];
	assign phit_o = psel_i && (w >= 8'h80) && (w <= 8'h9F);

	// ------------------------------------------------------------- read mux
	always @* begin
		case (w)
		8'h80: prdata_o = {3'b111, str,  3'b111, sync};
		8'h81: prdata_o = {1'b1, mdf, fdir, pwm, 2'b11, cmd, bf};
		8'h82: prdata_o = {tcr[0], tior[0] | 8'h08};
		8'h83: prdata_o = {5'b11111, tier[0], 5'b11111, tsr[0]};
		8'h84: prdata_o = tcnt[0];
		8'h85: prdata_o = gra[0];
		8'h86: prdata_o = grb[0];
		8'h87: prdata_o = {tcr[1], tior[1] | 8'h08};
		8'h88: prdata_o = {5'b11111, tier[1], 5'b11111, tsr[1]};
		8'h89: prdata_o = tcnt[1];
		8'h8A: prdata_o = gra[1];
		8'h8B: prdata_o = grb[1];
		8'h8C: prdata_o = {tcr[2], tior[2] | 8'h08};
		8'h8D: prdata_o = {5'b11111, tier[2], 5'b11111, tsr[2]};
		8'h8E: prdata_o = tcnt[2];
		8'h8F: prdata_o = gra[2];
		8'h90: prdata_o = grb[2];
		8'h91: prdata_o = {tcr[3], tior[3] | 8'h08};
		8'h92: prdata_o = {5'b11111, tier[3], 5'b11111, tsr[3]};
		8'h93: prdata_o = tcnt[3];
		8'h94: prdata_o = gra[3];
		8'h95: prdata_o = grb[3];
		8'h96: prdata_o = bra[3];
		8'h97: prdata_o = brb[3];
		8'h98: prdata_o = {8'hFF, 6'b111111, ols};
		8'h99: prdata_o = {tcr[4], tior[4] | 8'h08};
		8'h9A: prdata_o = {5'b11111, tier[4], 5'b11111, tsr[4]};
		8'h9B: prdata_o = tcnt[4];
		8'h9C: prdata_o = gra[4];
		8'h9D: prdata_o = grb[4];
		8'h9E: prdata_o = bra[4];
		default: prdata_o = brb[4];
		endcase
	end

	// --------------------------------------------------------- clock ticks
	// Internal taps count on the falling edge of the divided clock; the
	// external ones on the edge TCR.CKEG selects.
	wire [3:0] int_tick = {psc[2:0] == 3'd0, psc[1:0] == 2'd0,
	                       psc[0] == 1'b0, 1'b1};
	reg [3:0] ext_edge;
	always @* begin
		for (i = 0; i < 4; i = i + 1)
			ext_edge[i] = (tclk_i[i] & ~tclk_d[i]) | (~tclk_i[i] & tclk_d[i]);
	end

	reg [4:0] tick;
	always @* begin
		for (i = 0; i < 5; i = i + 1) begin
			if (tcr[i][2]) begin
				case (tcr[i][4:3])
				2'b00:   tick[i] =  tclk_i[tcr[i][1:0]] & ~tclk_d[tcr[i][1:0]];
				2'b01:   tick[i] = ~tclk_i[tcr[i][1:0]] &  tclk_d[tcr[i][1:0]];
				default: tick[i] = ext_edge[tcr[i][1:0]];
				endcase
			end else begin
				tick[i] = int_tick[tcr[i][1:0]];
			end
		end
	end

	// Channel 2 in phase counting mode steps on every edge of TCLKA and
	// TCLKB; the direction is the quadrature of the two.
	wire phase_mode = mdf;
	wire phase_step = ext_edge[0] | ext_edge[1];
	wire phase_up   = (tclk_i[0] & ~tclk_d[0] & ~tclk_i[1])  ? 1'b0
	                : (tclk_i[0] & ~tclk_d[0] &  tclk_i[1])  ? 1'b1
	                : (~tclk_i[0] & tclk_d[0] &  tclk_i[1])  ? 1'b0
	                : (~tclk_i[0] & tclk_d[0] & ~tclk_i[1])  ? 1'b1
	                : (tclk_i[1] & ~tclk_d[1] &  tclk_i[0])  ? 1'b0
	                : (tclk_i[1] & ~tclk_d[1] & ~tclk_i[0])  ? 1'b1
	                : (~tclk_i[1] & tclk_d[1] & ~tclk_i[0])  ? 1'b0
	                :                                          1'b1;

	// ------------------------------------------------------------- writes
	wire wr = ce_i && psel_i && pwr_i;
	function automatic [7:0] mrg(input [7:0] old, input [7:0] nw, input lane);
		mrg = lane ? nw : old;
	endfunction
	function automatic [15:0] mrg16(input [15:0] old, input [15:0] nw,
	                                input [1:0] lanes);
		mrg16 = {lanes[1] ? nw[15:8] : old[15:8],
		         lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction

	// Which counter, general register or buffer register this cycle writes.
	// A write and a timer event landing in the same cycle have a fixed
	// winner, so the events are decided against these.
	reg [4:0] wr_tcnt, wr_gra, wr_grb, wr_bra, wr_brb;
	always @* begin
		wr_tcnt = 5'd0; wr_gra = 5'd0; wr_grb = 5'd0;
		wr_bra  = 5'd0; wr_brb = 5'd0;
		if (wr) case (w)
		8'h84: wr_tcnt[0] = 1'b1;
		8'h89: wr_tcnt[1] = 1'b1;
		8'h8E: wr_tcnt[2] = 1'b1;
		8'h93: wr_tcnt[3] = 1'b1;
		8'h9B: wr_tcnt[4] = 1'b1;
		8'h85: wr_gra[0] = 1'b1;
		8'h8A: wr_gra[1] = 1'b1;
		8'h8F: wr_gra[2] = 1'b1;
		8'h94: wr_gra[3] = 1'b1;
		8'h9C: wr_gra[4] = 1'b1;
		8'h86: wr_grb[0] = 1'b1;
		8'h8B: wr_grb[1] = 1'b1;
		8'h90: wr_grb[2] = 1'b1;
		8'h95: wr_grb[3] = 1'b1;
		8'h9D: wr_grb[4] = 1'b1;
		8'h96: wr_bra[3] = 1'b1;
		8'h9E: wr_bra[4] = 1'b1;
		8'h97: wr_brb[3] = 1'b1;
		8'h9F: wr_brb[4] = 1'b1;
		default: ;
		endcase
	end

	// --------------------------------------------------- matches and events
	// A write to a general register in the same cycle inhibits its compare
	// match outright. In PWM mode both general registers are output compare
	// whatever TIOR says, and a match on both at once is inhibited too.
	reg [4:0] step, cma, cmb, cma_r, cmb_r, capa, capb, ovf_ev, clr, cleared;
	always @* begin
		for (i = 0; i < 5; i = i + 1) begin
			step[i] = str[i] && ((i == 2) && phase_mode ? phase_step : tick[i]);
			capa[i] = tior[i][2] && !pwm[i] && edge_sel(tior[i][1:0],
			          tioca_i[i], tioca_d[i]);
			capb[i] = tior[i][6] && !pwm[i] && edge_sel(tior[i][5:4],
			          tiocb_i[i], tiocb_d[i]);
			cma_r[i] = step[i] && (!tior[i][2] || pwm[i]) && !wr_gra[i]
			           && (tcnt[i] == gra[i]);
			cmb_r[i] = step[i] && (!tior[i][6] || pwm[i]) && !wr_grb[i]
			           && (tcnt[i] == grb[i]);
			cma[i]  = cma_r[i] && !(pwm[i] && cmb_r[i]);
			cmb[i]  = cmb_r[i] && !(pwm[i] && cma_r[i]);
			clr[i]  = ((tcr[i][6:5] == 2'b01) && (cma[i] || capa[i]))
			       || ((tcr[i][6:5] == 2'b10) && (cmb[i] || capb[i]));
		end
	end

	// A synchronised channel with CCLR = 11 clears whenever any other
	// synchronised channel clears.
	wire sync_clr = |(clr & sync);

	// A counter that is cleared has not carried out, so it does not overflow.
	always @* begin
		for (i = 0; i < 5; i = i + 1) begin
			cleared[i] = clr[i]
			          || (sync_clr && sync[i] && (tcr[i][6:5] == 2'b11));
			ovf_ev[i] = step[i] && !cleared[i]
			            && (((i == 2) && phase_mode && !phase_up)
			                ? (tcnt[i] == 16'h0000) && !fdir
			                : (tcnt[i] == 16'hFFFF));
		end
	end

	function automatic edge_sel(input [1:0] sel, input now, input was);
		begin
			case (sel)
			2'b00:   edge_sel = now & ~was;
			2'b01:   edge_sel = ~now & was;
			default: edge_sel = (now ^ was);
			endcase
		end
	endfunction

	// A TIOC pin is driven when its general register is output compare with a
	// non-zero output action, or when the channel is in PWM mode.
	reg [4:0] r_imia, r_imib, r_ovi, oc_a, oc_b;
	always @* begin
		for (i = 0; i < 5; i = i + 1) begin
			r_imia[i] = tsr[i][0] & tier[i][0];
			r_imib[i] = tsr[i][1] & tier[i][1];
			r_ovi[i]  = tsr[i][2] & tier[i][2];
			oc_a[i]   = !tior[i][2] && (tior[i][1:0] != 2'b00);
			oc_b[i]   = !tior[i][6] && (tior[i][5:4] != 2'b00);
		end
	end

	assign imia_o     = r_imia;
	assign imib_o     = r_imib;
	assign ovi_o      = r_ovi;
	assign cma_o      = cma[3:0];
	assign tioca_oe_o = pwm | oc_a;
	assign tiocb_oe_o = oc_b;

	// --------------------------------------------------------- savestate
	// Two words per channel, then one for the registers shared by all five.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_CH [0:4];      // {TIOR, TCR, GRB, GRA, TCNT}
	wire [63:0] SS_BR [0:4];      // {TSR, TIER, BRB, BRA}
	wire [63:0] SS_GBL;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_q_ch [0:4];
	wire [63:0] ss_q_br [0:4];
	wire [63:0] ss_q_gbl;

	genvar g;
	generate
		for (g = 0; g < 5; g = g + 1) begin : g_ss_ch
			// GRA, GRB and both buffer registers come up all ones; TIOR's
			// reset value H'08 is GRB as an output compare with no action.
			ss_reg #(.ADDR (SSW_ITU_BASE + g),
			         .DEFAULT ({8'h08, 8'h00, 16'hFFFF, 16'hFFFF, 16'h0000}))
			       u_ss_ch (
				.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren), .bus_rst_i (ss_rst),
				.bus_dout_o (ss_q_ch[g]),
				.din_i ({tior[g], tcr[g], grb[g], gra[g], tcnt[g]}),
				.dout_o (SS_CH[g])
			);
			ss_reg #(.ADDR (SSW_ITU_BASE + 5 + g),
			         .DEFAULT ({26'd0, 3'd0, 3'd0, 16'hFFFF, 16'hFFFF}))
			       u_ss_br (
				.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren), .bus_rst_i (ss_rst),
				.bus_dout_o (ss_q_br[g]),
				.din_i ({26'd0, tsr[g], tier[g], brb[g], bra[g]}),
				.dout_o (SS_BR[g])
			);
		end
	endgenerate

	wire [63:0] SS_GBL_BACK = {12'd0, tiocb_o, tioca_o, tiocb_d, tioca_d,
	                           tclk_d, psc, ols, bf, cmd, fdir, mdf,
	                           pwm, sync, str};

	ss_reg #(.ADDR (SSW_ITU_BASE + 10),
	         .DEFAULT ({12'd0, 5'd0, 5'd0, 5'd0, 5'd0, 4'd0, 3'd0, 2'b11,
	                    4'd0, 2'd0, 1'b0, 1'b0, 5'd0, 5'd0, 5'd0})) u_ss_gbl (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_q_gbl),
		.din_i (SS_GBL_BACK), .dout_o (SS_GBL)
	);

	integer q;
	reg [63:0] ss_or;
	always @* begin
		ss_or = ss_q_gbl;
		for (q = 0; q < 5; q = q + 1)
			ss_or = ss_or | ss_q_ch[q] | ss_q_br[q];
	end
	assign ss_dout = ss_or;

	// A byte write to one half of a synchronised TCNT writes the whole
	// 16-bit result into every synchronised counter.
	reg [15:0] tcnt_wval;
	reg [2:0]  tcnt_ch;
	reg [4:0]  tcnt_wch;
	always @* begin
		tcnt_ch = 3'd0;
		for (i = 0; i < 5; i = i + 1)
			if (wr_tcnt[i]) tcnt_ch = i[2:0];
		tcnt_wval = mrg16(tcnt[tcnt_ch], pwdata_i, pbe_i);
		for (i = 0; i < 5; i = i + 1)
			tcnt_wch[i] = (|wr_tcnt)
			              && (sync[tcnt_ch] ? sync[i] : (tcnt_ch == i[2:0]));
	end

	always @(posedge clk_i) begin
		if (rst_i) begin
			str  <= SS_GBL[4:0];   sync <= SS_GBL[9:5];  pwm <= SS_GBL[14:10];
			mdf  <= SS_GBL[15];    fdir <= SS_GBL[16];   cmd <= SS_GBL[18:17];
			bf   <= SS_GBL[22:19]; ols  <= SS_GBL[24:23];
			psc  <= SS_GBL[27:25]; tclk_d  <= SS_GBL[31:28];
			tioca_d <= SS_GBL[36:32]; tiocb_d <= SS_GBL[41:37];
			tioca_o <= SS_GBL[46:42]; tiocb_o <= SS_GBL[51:47];
			for (i = 0; i < 5; i = i + 1) begin
				tcnt[i] <= SS_CH[i][15:0];
				gra[i]  <= SS_CH[i][31:16];
				grb[i]  <= SS_CH[i][47:32];
				tcr[i]  <= SS_CH[i][55:48];
				tior[i] <= SS_CH[i][63:56];
				bra[i]  <= SS_BR[i][15:0];
				brb[i]  <= SS_BR[i][31:16];
				tier[i] <= SS_BR[i][34:32];
				tsr[i]  <= SS_BR[i][37:35];
			end
		end else if (ce_i) begin
			psc     <= psc + 3'd1;
			tclk_d  <= tclk_i;
			tioca_d <= tioca_i;
			tiocb_d <= tiocb_i;

			for (i = 0; i < 5; i = i + 1) begin
				// A counter clear beats a CPU write, and a CPU write beats
				// the count.
				if (cleared[i])
					tcnt[i] <= 16'h0000;
				else if (tcnt_wch[i])
					tcnt[i] <= tcnt_wval;
				else if (step[i])
					tcnt[i] <= ((i == 2) && phase_mode && !phase_up)
					           ? tcnt[i] - 16'd1 : tcnt[i] + 16'd1;

				// flags
				if (cma[i] || capa[i]) tsr[i][0] <= 1'b1;
				if (cmb[i] || capb[i]) tsr[i][1] <= 1'b1;
				if (ovf_ev[i])         tsr[i][2] <= 1'b1;

				// Capture, with the buffer register taking the displaced
				// value. A capture beats a CPU write to either register.
				if (capa[i]) begin
					gra[i] <= tcnt[i];
					if (bf[(i == 3) ? 0 : 2] && (i >= 3)) bra[i] <= gra[i];
				end else begin
					if (wr_gra[i]) gra[i] <= mrg16(gra[i], pwdata_i, pbe_i);
					// compare match reloads the general register from the buffer
					if (cma[i] && (i >= 3) && bf[(i == 3) ? 0 : 2])
						gra[i] <= bra[i];
					if (wr_bra[i]) bra[i] <= mrg16(bra[i], pwdata_i, pbe_i);
				end
				if (capb[i]) begin
					grb[i] <= tcnt[i];
					if (bf[(i == 3) ? 1 : 3] && (i >= 3)) brb[i] <= grb[i];
				end else begin
					if (wr_grb[i]) grb[i] <= mrg16(grb[i], pwdata_i, pbe_i);
					if (cmb[i] && (i >= 3) && bf[(i == 3) ? 1 : 3])
						grb[i] <= brb[i];
					if (wr_brb[i]) brb[i] <= mrg16(brb[i], pwdata_i, pbe_i);
				end

				// pin actions
				if (pwm[i]) begin
					if (cma[i] && !cmb[i]) tioca_o[i] <= 1'b1;
					if (cmb[i] && !cma[i]) tioca_o[i] <= 1'b0;
				end else if (cma[i]) begin
					case (tior[i][1:0])
					2'b01: tioca_o[i] <= 1'b0;
					2'b10: tioca_o[i] <= 1'b1;
					2'b11: tioca_o[i] <= (i == 2) ? 1'b1 : ~tioca_o[i];
					default: ;
					endcase
				end
				if (cmb[i]) begin
					case (tior[i][5:4])
					2'b01: tiocb_o[i] <= 1'b0;
					2'b10: tiocb_o[i] <= 1'b1;
					2'b11: tiocb_o[i] <= (i == 2) ? 1'b1 : ~tiocb_o[i];
					default: ;
					endcase
				end

				// a DMAC start on IMIA clears the flag, channels 0 to 3
				if ((i < 4) && dmac_ack_i[i[1:0]]) tsr[i][0] <= 1'b0;
			end

			if (wr) case (w)
				8'h80: begin
					if (pbe_i[1]) str  <= pwdata_i[12:8];
					if (pbe_i[0]) sync <= pwdata_i[4:0];
				end
				8'h81: begin
					if (pbe_i[1]) begin
						mdf  <= pwdata_i[14];
						fdir <= pwdata_i[13];
						pwm  <= pwdata_i[12:8];
					end
					if (pbe_i[0]) begin cmd <= pwdata_i[5:4]; bf <= pwdata_i[3:0]; end
				end
				8'h98: if (pbe_i[0]) ols <= pwdata_i[1:0];
				8'h82, 8'h87, 8'h8C, 8'h91, 8'h99: begin
					tcr[ch_of(w)]  <= mrg(tcr[ch_of(w)],  pwdata_i[15:8], pbe_i[1]);
					tior[ch_of(w)] <= mrg(tior[ch_of(w)], pwdata_i[7:0],  pbe_i[0]);
				end
				8'h83, 8'h88, 8'h8D, 8'h92, 8'h9A: begin
					if (pbe_i[1]) tier[ch_of(w)] <= pwdata_i[10:8];
					// only a zero clears a status flag, and only after the
					// software has read it as one
					if (pbe_i[0]) begin
						if (!pwdata_i[0]) tsr[ch_of(w)][0] <= 1'b0;
						if (!pwdata_i[1]) tsr[ch_of(w)][1] <= 1'b0;
						if (!pwdata_i[2]) tsr[ch_of(w)][2] <= 1'b0;
					end
				end
				default: ;
			endcase
		end
	end

	function automatic [2:0] ch_of(input [7:0] wa);
		begin
			case (wa)
			8'h82, 8'h83, 8'h85, 8'h86: ch_of = 3'd0;
			8'h87, 8'h88, 8'h8A, 8'h8B: ch_of = 3'd1;
			8'h8C, 8'h8D, 8'h8F, 8'h90: ch_of = 3'd2;
			8'h91, 8'h92, 8'h94, 8'h95: ch_of = 3'd3;
			default:                    ch_of = 3'd4;
			endcase
		end
	endfunction
endmodule
