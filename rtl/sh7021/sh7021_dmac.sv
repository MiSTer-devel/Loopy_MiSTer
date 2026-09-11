// Copyright (c) 2026 Jamie Blanks
//
// SH7021 direct memory access controller, four channels. Dual address mode
// reads the source in one bus cycle and writes the destination in the next;
// single address mode (channels 0 and 1) drives one address and asserts DACK
// to the other end in the same cycle. Round-robin orders are rotations of
// the ring 0, 3, 2, 1.

module sh7021_dmac (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	input  wire [8:0]  paddr_i,
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	// internal bus master port
	output wire        breq_o,
	input  wire        bgnt_i,
	output reg         req_o,
	output reg  [31:0] addr_o,
	output reg         we_o,
	output wire [1:0]  sz_o,
	output wire [31:0] wdata_o,
	input  wire [31:0] rdata_i,
	input  wire        ack_i,

	input  wire [1:0]  dreq_i,         // DREQ0, DREQ1 pin levels
	output wire [1:0]  dack_o,
	input  wire        nmi_i,          // NMI accepted, sets NMIF
	input  wire        addr_err_i,     // the bus saw a DMAC address error

	// on-chip request sources selected by CHCR.RS
	input  wire [1:0]  rxi_i,
	input  wire [1:0]  txi_i,
	input  wire [3:0]  imia_i,
	output reg  [3:0]  imia_ack_o,     // clears the ITU flag that started us
	output reg  [1:0]  rxi_ack_o,      // and the SCI flag, per channel
	output reg  [1:0]  txi_ack_o,

	output wire [3:0]  dei_o,
	output wire        addr_err_o,     // DMAOR.AE, the DMA address error flag

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	integer c;

	reg [31:0] sar  [0:3];
	reg [31:0] dar  [0:3];
	reg [15:0] tcr  [0:3];
	reg [15:0] chcr [0:3];
	reg [1:0]  pr;
	reg        ae, nmif, dme;
	reg [1:0]  order [0:3];            // round-robin ranking, 0 = highest
	reg        epr_low;                // the lower of ch0/ch1, external-pin mode
	reg [1:0]  dreq_d;
	reg [1:0]  dreq_lat;               // a falling DREQ waits for the bus

	wire [15:0] dmaor = {6'd0, pr, 5'd0, ae, nmif, dme};
	wire [7:0]  w     = paddr_i[8:1];
	assign phit_o = psel_i && (w >= 8'hA0) && (w <= 8'hBF);

	wire [1:0] ch_sel = w[4:3];
	wire [2:0] off    = w[2:0];

	always @* begin
		if (w == 8'hA4) prdata_o = dmaor;
		else case (off)
		3'd0:    prdata_o = sar[ch_sel][31:16];
		3'd1:    prdata_o = sar[ch_sel][15:0];
		3'd2:    prdata_o = dar[ch_sel][31:16];
		3'd3:    prdata_o = dar[ch_sel][15:0];
		3'd5:    prdata_o = tcr[ch_sel];
		3'd7:    prdata_o = chcr[ch_sel];
		default: prdata_o = 16'd0;
		endcase
	end

	// ------------------------------------------------------ per-channel view
	function automatic [3:0] rs(input [1:0] ch); rs = chcr[ch][11:8]; endfunction
	function automatic       de(input [1:0] ch); de = chcr[ch][0];    endfunction
	function automatic       te(input [1:0] ch); te = chcr[ch][1];    endfunction
	function automatic       ie(input [1:0] ch); ie = chcr[ch][2];    endfunction
	function automatic       ts(input [1:0] ch); ts = chcr[ch][3];    endfunction
	function automatic       tm(input [1:0] ch); tm = chcr[ch][4];    endfunction
	function automatic       ds(input [1:0] ch); ds = chcr[ch][5];    endfunction
	function automatic       al(input [1:0] ch); al = chcr[ch][6];    endfunction
	function automatic       am(input [1:0] ch); am = chcr[ch][7];    endfunction

	wire enable_ok = dme && !nmif && !ae;

	// DREQ is a low level or a falling edge, per CHCR.DS.
	wire [1:0] dreq_edge = dreq_d & ~dreq_i;

	// A single-address channel drives one address and DACKs the other end.
	function automatic single(input [1:0] ch);
		single = (ch[1] == 1'b0) && ((rs(ch) == 4'b0010) || (rs(ch) == 4'b0011));
	endfunction

	// The register array is read directly: a function call does not always
	// carry the array into a procedural block's implicit sensitivity list.
	reg [3:0] want;
	always @* begin
		for (c = 0; c < 4; c = c + 1) begin
			// TE says a channel is finished; H'0000 in TCR means 65536
			// transfers.
			want[c] = chcr[c[1:0]][0] && !chcr[c[1:0]][1] && enable_ok;
			case (chcr[c[1:0]][11:8])
			4'b0000, 4'b0010, 4'b0011:
				want[c] = want[c] && (c[1] == 1'b0)
				          && (chcr[c[1:0]][5] ? dreq_lat[c[0]] : ~dreq_i[c[0]]);
			4'b0100: want[c] = want[c] && rxi_i[0] && !rxi_ack_o[0];
			4'b0101: want[c] = want[c] && txi_i[0] && !txi_ack_o[0];
			4'b0110: want[c] = want[c] && rxi_i[1] && !rxi_ack_o[1];
			4'b0111: want[c] = want[c] && txi_i[1] && !txi_ack_o[1];
			4'b1000: want[c] = want[c] && imia_i[0] && !imia_ack_o[0];
			4'b1001: want[c] = want[c] && imia_i[1] && !imia_ack_o[1];
			4'b1010: want[c] = want[c] && imia_i[2] && !imia_ack_o[2];
			4'b1011: want[c] = want[c] && imia_i[3] && !imia_ack_o[3];
			4'b1100: ;                                  // auto request
			default: want[c] = 1'b0;                    // reserved settings
			endcase
		end
	end

	// ---------------------------------------------------------- arbitration
	// Effective ranking, 0 = highest. Only round robin uses the rotating
	// array; external-pin round robin swaps channels 0 and 1 below a fixed
	// 3 > 2 and starts at 3 > 2 > 1 > 0.
	reg [1:0] rank [0:3];
	always @* begin
		for (c = 0; c < 4; c = c + 1) begin
			case (pr)
			2'b00:   rank[c] = (c == 0) ? 2'd0 : (c == 3) ? 2'd1
			                 : (c == 2) ? 2'd2 : 2'd3;
			2'b01:   rank[c] = (c == 1) ? 2'd0 : (c == 3) ? 2'd1
			                 : (c == 2) ? 2'd2 : 2'd3;
			2'b10:   rank[c] = order[c[1:0]];
			default: rank[c] = (c == 3) ? 2'd0 : (c == 2) ? 2'd1
			                 : (c[0] == epr_low) ? 2'd3 : 2'd2;
			endcase
		end
	end

	reg [1:0] pick;
	reg       pick_v;
	always @* begin
		pick   = 2'd0;
		pick_v = 1'b0;
		for (c = 3; c >= 0; c = c - 1)
			if (want[c] && (!pick_v || (rank[c] <= rank[pick]))) begin
				pick   = c[1:0];
				pick_v = 1'b1;
			end
	end

	// The winning channel is registered before it becomes a bus request,
	// costing one state to acquire the bus and keeping the arbitration out
	// of the bus-to-INTC path.
	reg [1:0] pick_r;
	reg       pick_vr;

	// A part select of a function call is rejected by some tools, so the
	// winning channel's resource field gets its own name.
	wire [3:0] rs_pick = chcr[pick_r][11:8];

	localparam [1:0] S_IDLE = 2'd0, S_READ = 2'd1, S_WRITE = 2'd2;
	reg [1:0]  st;
	reg [1:0]  ch;
	reg [31:0] hold;
	// Cycle steal hands the bus back for one state after each transfer unit;
	// burst mode keeps it.
	reg        yield;

	// Everything that reads the CHCR array is computed procedurally, since a
	// continuous assign through a function loses sensitivity to the array.
	reg       dack_now;
	reg [1:0] r_sz;
	reg [1:0] r_dack;
	reg [3:0] r_dei;
	always @* begin
		dack_now = ((ch[1] == 1'b0)
		            && ((chcr[ch][11:8] == 4'b0010) || (chcr[ch][11:8] == 4'b0011)))
		           || (chcr[ch][7] ? (st == S_WRITE) : (st == S_READ));
		r_sz     = chcr[ch][3] ? 2'd1 : 2'd0;
		// CHCR.AL picks the active level, so an idle pin sits at the other
		// one.
		r_dack   = {(((st != S_IDLE) && (ch == 2'd1) && dack_now) ^ chcr[1][6]),
		            (((st != S_IDLE) && (ch == 2'd0) && dack_now) ^ chcr[0][6])};
		for (c = 0; c < 4; c = c + 1)
			r_dei[c] = chcr[c[1:0]][1] & chcr[c[1:0]][2];
	end

	// The winner is a state old by the time it can take the bus, so it is
	// checked against the live want; otherwise the state that sets TE would
	// run one transfer unit past the count.
	assign breq_o = (pick_vr && want[pick_r] && !yield) || (st != S_IDLE);
	assign sz_o   = r_sz;
	assign dack_o = r_dack;
	assign dei_o  = r_dei;
	assign addr_err_o = ae;
	// The word read in the source cycle is what the destination cycle writes.
	assign wdata_o    = hold;

	wire [31:0] step = r_sz[0] ? 32'd2 : 32'd1;

	function automatic [31:0] adv(input [31:0] a, input [1:0] mode,
	                              input [31:0] d);
		adv = (mode == 2'b01) ? a + d : (mode == 2'b10) ? a - d : a;
	endfunction

	function automatic [15:0] mrg(input [15:0] old, input [15:0] nw,
	                              input [1:0] lanes);
		mrg = {lanes[1] ? nw[15:8] : old[15:8],
		       lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction

	// Misaligned word access by the DMAC is an address error.
	wire dm_misaligned = r_sz[0] && (st == S_READ ? sar[ch][0] : dar[ch][0]);

	// --------------------------------------------------------- savestate
	// Two words per channel, then one for the arbiter and the transfer state.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_AD [0:3];        // {DAR, SAR}
	wire [63:0] SS_CT [0:3];        // {CHCR, TCR}
	wire [63:0] SS_HOLD, SS_CTL;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_q_ad [0:3];
	wire [63:0] ss_q_ct [0:3];
	wire [63:0] ss_q_hold, ss_q_ctl;

	genvar g;
	generate
		for (g = 0; g < 4; g = g + 1) begin : g_ss_ch
			ss_reg #(.ADDR (SSW_DMAC_BASE + 2*g), .DEFAULT (64'd0)) u_ss_ad (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				.bus_rst_i  (ss_rst),
				.bus_dout_o (ss_q_ad[g]),
				.din_i      ({dar[g], sar[g]}),
				.dout_o     (SS_AD[g])
			);
			ss_reg #(.ADDR (SSW_DMAC_BASE + 2*g + 1), .DEFAULT (64'd0)) u_ss_ct (
				.clk_i      (clk_i),
				.bus_din_i  (ss_din),
				.bus_addr_i (ss_addr),
				.bus_wren_i (ss_wren),
				.bus_rst_i  (ss_rst),
				.bus_dout_o (ss_q_ct[g]),
				.din_i      ({32'd0, chcr[g], tcr[g]}),
				.dout_o     (SS_CT[g])
			);
		end
	endgenerate

	ss_reg #(.ADDR (SSW_DMAC_BASE + 8), .DEFAULT (64'd0)) u_ss_hold (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_q_hold),
		.din_i      ({addr_o, hold}),
		.dout_o     (SS_HOLD)
	);

	wire [63:0] SS_CTL_BACK = {28'd0, txi_ack_o, rxi_ack_o, dreq_lat, epr_low,
	                           pick_vr, pick_r, imia_ack_o, we_o, req_o,
	                           yield, ch, st, dreq_d,
	                           order[3], order[2], order[1], order[0],
	                           dme, nmif, ae, pr};

	// Round robin starts as the ring 0, 3, 2, 1, and both DREQ pins are
	// inactive high.
	ss_reg #(.ADDR (SSW_DMAC_CTL),
	         .DEFAULT ({28'd0, 2'd0, 2'd0, 2'b00, 1'b0, 1'b0, 2'd0, 4'd0, 1'b0, 1'b0,
	                    1'b0, 2'd0, 2'd0, 2'b11,
	                    2'd1, 2'd2, 2'd3, 2'd0,
	                    1'b0, 1'b0, 1'b0, 2'd0})) u_ss_ctl (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_q_ctl),
		.din_i (SS_CTL_BACK), .dout_o (SS_CTL)
	);

	integer q;
	reg [63:0] ss_or;
	always @* begin
		ss_or = ss_q_hold | ss_q_ctl;
		for (q = 0; q < 4; q = q + 1)
			ss_or = ss_or | ss_q_ad[q] | ss_q_ct[q];
	end
	assign ss_dout = ss_or;

	always @(posedge clk_i) begin
		if (rst_i) begin
			for (c = 0; c < 4; c = c + 1) begin
				sar[c]   <= SS_AD[c][31:0];
				dar[c]   <= SS_AD[c][63:32];
				tcr[c]   <= SS_CT[c][15:0];
				chcr[c]  <= SS_CT[c][31:16];
				order[c] <= SS_CTL[5 + 2*c +: 2];
			end
			pr <= SS_CTL[1:0]; ae <= SS_CTL[2]; nmif <= SS_CTL[3];
			dme <= SS_CTL[4]; dreq_d <= SS_CTL[14:13];
			st <= SS_CTL[16:15]; ch <= SS_CTL[18:17]; yield <= SS_CTL[19];
			req_o <= SS_CTL[20]; we_o <= SS_CTL[21];
			imia_ack_o <= SS_CTL[25:22];
			pick_r <= SS_CTL[27:26]; pick_vr <= SS_CTL[28];
			epr_low <= SS_CTL[29]; dreq_lat <= SS_CTL[31:30];
			rxi_ack_o <= SS_CTL[33:32]; txi_ack_o <= SS_CTL[35:34];
			hold <= SS_HOLD[31:0]; addr_o <= SS_HOLD[63:32];
		end else if (ce_i) begin
			dreq_d     <= dreq_i;
			dreq_lat   <= dreq_lat | dreq_edge;
			imia_ack_o <= 4'd0;
			rxi_ack_o  <= 2'd0;
			txi_ack_o  <= 2'd0;
			pick_r     <= pick;
			pick_vr    <= pick_v;
			if (yield && (st == S_IDLE)) yield <= 1'b0;
			if (nmi_i)      nmif <= 1'b1;
			if (addr_err_i) ae   <= 1'b1;

			case (st)
			S_IDLE: if (pick_vr && want[pick_r] && bgnt_i && !yield) begin
				yield  <= 1'b0;
				ch     <= pick_r;
				st     <= single(pick_r) ? S_WRITE : S_READ;
				req_o  <= 1'b1;
				// RS = 0010 reads external memory for the DACK device;
				// RS = 0011 writes what the DACK device supplies.
				we_o   <= single(pick_r) && (rs_pick == 4'b0011);
				addr_o <= (rs_pick == 4'b0011) ? dar[pick_r] : sar[pick_r];
			end
			S_READ: if (dm_misaligned) begin
				ae <= 1'b1; req_o <= 1'b0; st <= S_IDLE;
			end else if (ack_i) begin
				hold   <= rdata_i;
				st     <= S_WRITE;
				we_o   <= 1'b1;
				addr_o <= dar[ch];
			end
			S_WRITE: if (dm_misaligned) begin
				ae <= 1'b1; req_o <= 1'b0; st <= S_IDLE;
			end else if (ack_i) begin
				req_o <= 1'b0;
				we_o  <= 1'b0;
				st    <= S_IDLE;
				yield <= ~tm(ch);
				sar[ch] <= adv(sar[ch], chcr[ch][13:12], step);
				dar[ch] <= adv(dar[ch], chcr[ch][15:14], step);
				tcr[ch] <= tcr[ch] - 16'd1;
				if (tcr[ch] == 16'd1) chcr[ch][1] <= 1'b1;   // TE
				if (chcr[ch][5]) dreq_lat[ch[0]] <= 1'b0;

				// The request is withdrawn when the transfer is performed;
				// clearing an SCI TDRE earlier would reload TSR from a stale
				// TDR. Cycle steal acknowledges every unit, burst only the last.
				if (chcr[ch][11:10] == 2'b01) begin
					if (chcr[ch][8]) txi_ack_o[chcr[ch][9]] <= 1'b1;
					else             rxi_ack_o[chcr[ch][9]] <= 1'b1;
				end
				if ((chcr[ch][11:10] == 2'b10)
				    && (!chcr[ch][4] || (tcr[ch] == 16'd1)))
					imia_ack_o[chcr[ch][9:8]] <= 1'b1;
				// round robin moves the channel that just ran to the bottom
				if (pr == 2'b10) begin
					for (c = 0; c < 4; c = c + 1)
						if ((c[1:0] != ch) && (order[c[1:0]] > order[ch]))
							order[c[1:0]] <= order[c[1:0]] - 2'd1;
					order[ch] <= 2'd3;
				end else if ((pr == 2'b11) && (ch[1] == 1'b0)) begin
					epr_low <= ch[0];
				end
			end
			default: st <= S_IDLE;
			endcase


			// ------------------------------------------------------ writes
			// Qualify with phit_o: the case below decodes only the low three
			// address bits, so psel_i alone would catch other peripherals.
			if (phit_o && pwr_i) begin
				if (w == 8'hA4) begin
					if (pbe_i[1]) pr <= pwdata_i[9:8];
					if (pbe_i[0]) begin
						dme <= pwdata_i[0];
						if (!pwdata_i[1]) nmif <= 1'b0;
						if (!pwdata_i[2]) ae   <= 1'b0;
					end
				end else case (off)
				3'd0: sar[ch_sel][31:16] <= mrg(sar[ch_sel][31:16], pwdata_i, pbe_i);
				3'd1: sar[ch_sel][15:0]  <= mrg(sar[ch_sel][15:0],  pwdata_i, pbe_i);
				3'd2: dar[ch_sel][31:16] <= mrg(dar[ch_sel][31:16], pwdata_i, pbe_i);
				3'd3: dar[ch_sel][15:0]  <= mrg(dar[ch_sel][15:0],  pwdata_i, pbe_i);
				3'd5: tcr[ch_sel]        <= mrg(tcr[ch_sel],        pwdata_i, pbe_i);
				3'd7: begin
					chcr[ch_sel] <= mrg(chcr[ch_sel], pwdata_i, pbe_i);
					// TE takes only a zero, which clears it
					if (pbe_i[0] && pwdata_i[1]) chcr[ch_sel][1] <= chcr[ch_sel][1];
					// AM, AL and DS exist on channels 0 and 1 only
					if (pbe_i[0] && ch_sel[1]) chcr[ch_sel][7:5] <= 3'd0;
				end
				default: ;
				endcase
			end
		end
	end

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, paddr_i[0], w[7:5]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
endmodule
