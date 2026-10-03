// Copyright (c) 2026 Jamie Blanks
//
// SH7021 bus state controller. One internal request in, one to four external
// sub-accesses out. Area from A26-A24, width from A27 and the per-area rules;
// on-chip ROM and RAM take one state, the register space three, external
// memory one or two plus WCR3 and WAIT, DRAM a precharge, a row and one or
// two column states. PCR, BCR.WARP and WCR2 are plain registers, and the
// on-chip register space is treated as 16 bits wide throughout.

module sh7021_bsc (
	input  wire        clk_i,
	input  wire        ce_i,
	// Second half of the state ce_i started. The DRAM strobes fall inside
	// their state so the chip on the other end can tell consecutive columns
	// apart.
	input  wire        ceh_i,
	input  wire        rst_i,
	input  wire [2:0]  md_i,          // mode pins MD2-MD0

	// internal bus from the CPU or the DMAC
	input  wire        req_i,
	input  wire [31:0] addr_i,
	input  wire        we_i,
	input  wire [1:0]  sz_i,          // 0 byte, 1 word, 2 longword
	input  wire [31:0] wdata_i,
	output reg  [31:0] rdata_o,
	output wire        ack_o,

	// on-chip memories, one state each
	output wire        rom_sel_o,
	output wire        ram_sel_o,
	output wire [31:0] mem_addr_o,
	output wire        ram_we_o,
	output wire [3:0]  ram_be_o,
	output wire [31:0] ram_wd_o,
	input  wire [31:0] rom_q_i,
	input  wire [31:0] ram_q_i,

	// on-chip register bus, three states
	output wire        psel_o,
	output wire [8:0]  paddr_o,
	output wire        pwr_o,
	output wire [1:0]  pbe_o,
	output wire [15:0] pwdata_o,
	input  wire [15:0] prdata_i,

	// external bus
	output wire [21:0] a_o,
	output wire [15:0] d_o,
	output wire        d_oe_o,
	input  wire [15:0] d_i,
	output wire [7:0]  cs_n_o,
	output wire        rd_n_o,
	output wire        wrh_n_o,
	output wire        wrl_n_o,
	output wire        ras_n_o,
	output wire        cash_n_o,
	output wire        casl_n_o,
	output wire        ah_o,
	input  wire        wait_n_i,

	output wire        cmi_o,          // refresh-counter compare match
	// High while an access is in flight and not finishing this state. A bus
	// cycle, including every sub-access a width conversion splits it into,
	// holds the bus to the end.
	output wire        hold_o,
	// The enable ending this state takes a word off the bus.
	output wire        rd_take_o,
	// A CBR refresh is running: no DRAM access can start.
	output wire        cbr_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "sh1_defs.svh"
	`include "ss_map.svh"

	// ----------------------------------------------------------- registers
	reg [15:0] bcr, wcr1, wcr2, wcr3, dcr, pcr;
	reg [7:0]  rcr, rtcsr, rtcnt, rtcor;

	wire        drame = bcr[15];
	wire        ioe   = bcr[14];
	wire        bas   = bcr[11];
	wire [7:0]  rw    = wcr1[15:8];
	wire        ww1   = wcr1[1];
	wire [1:0]  a02lw = wcr3[14:13];
	wire [1:0]  a6lw  = wcr3[12:11];
	wire        cw2   = dcr[15];
	wire        rasd  = dcr[14];
	wire        tpc   = dcr[13];
	wire        be    = dcr[12];
	wire        mxe   = dcr[10];
	wire [1:0]  mxc   = dcr[9:8];
	wire        rfshe = rcr[7];
	wire        rmode = rcr[6];
	wire [1:0]  rlw   = rcr[5:4];
	wire        cmf   = rtcsr[7];
	wire        cmie  = rtcsr[6];
	wire [2:0]  cks   = rtcsr[5:3];

	assign cmi_o = cmf & cmie;

	// ------------------------------------------------------- area decoding
	wire [2:0] area  = addr_i[26:24];
	wire       wide  = addr_i[27];
	// Mode 2 puts the 32 KB on-chip ROM in area 0 with no CS0.
	wire       rom_en = (md_i == 3'b010);

	localparam [2:0] T_ROM = 3'd0, T_RAM = 3'd1, T_PERIPH = 3'd2,
	                 T_EXT = 3'd3, T_DRAM = 3'd4, T_MPX = 3'd5;

	reg [2:0] tgt;
	always @* begin
		if ((area == 3'd0) && rom_en)                   tgt = T_ROM;
		else if ((area == 3'd7) && wide)                tgt = T_RAM;
		else if ((area == 3'd5) && !wide)               tgt = T_PERIPH;
		else if ((area == 3'd1) && drame)               tgt = T_DRAM;
		else if ((area == 3'd6) && !wide && ioe)        tgt = T_MPX;
		else                                            tgt = T_EXT;
	end

	// Bus width in bytes: on-chip spaces are 32 bits, area 0 external takes
	// its width from MD0, area 5 external and everything with A27 set is 16
	// bits, area 6 low half from A14, otherwise 8 bits.
	reg [1:0] width;                 // 1, 2 or 4 bytes
	always @* begin
		if ((tgt == T_ROM) || (tgt == T_RAM))       width = 2'd3;   // 4 bytes
		else if (area == 3'd0)                      width = md_i[0] ? 2'd2 : 2'd1;
		else if ((tgt == T_MPX) || ((area == 3'd6) && !wide))
		                                            width = addr_i[14] ? 2'd2 : 2'd1;
		else if (wide)                              width = 2'd2;
		else if (area == 3'd5)                      width = 2'd2;   // registers
		else                                        width = 2'd1;
	end

	// Number of external sub-accesses this request needs.
	reg [2:0] nsub;
	always @* begin
		if (width == 2'd3)      nsub = 3'd1;
		else if (width == 2'd2) nsub = (sz_i == SZ_L) ? 3'd2 : 3'd1;
		else                    nsub = (sz_i == SZ_L) ? 3'd4
		                             : (sz_i == SZ_W) ? 3'd2 : 3'd1;
	end

	// --------------------------------------------------- long wait lookup
	wire       long_area = (area == 3'd0) || (area == 3'd2) || (area == 3'd6);
	wire [1:0] lw_sel    = (area == 3'd6) ? a6lw : a02lw;
	wire [2:0] long_wait = {1'b0, lw_sel} + 3'd1;         // 1..4 states

	wire rw_bit = rw[area];

	// ------------------------------------------------------- state registers
	reg        busy;
	reg        refreshing;
	reg [3:0]  ref_cnt;
	reg [2:0]  sub;
	reg [3:0]  cnt;
	reg [31:0] rbuf;
	reg        ras_idle;              // RAS level between DRAM accesses
	reg        acc_hit;               // this access is running on an open row
	reg [3:0]  ref_pending;

	// ------------------------------------------------------ DRAM row state
	reg         row_valid;
	reg  [23:0] row_addr;
	reg  [23:0] row_mask;
	always @* begin
		case (mxc)
		2'd0:    row_mask = 24'hFFFF00;
		2'd1:    row_mask = 24'hFFFE00;
		default: row_mask = 24'hFFFC00;
		endcase
	end
	wire [23:0] this_row = addr_i[23:0] & row_mask;
	wire        dram_hit  = be && row_valid && (row_addr == this_row);
	wire        dram_long = we_i ? ww1 : rw_bit;
	// Once an access is running, the open-row state is the one latched when it
	// started; later sub-accesses of the same request stay on that row.
	wire        hit_now   = busy ? acc_hit : dram_hit;
	wire [3:0]  dram_cols = dram_long ? 4'd2 : 4'd1;
	wire [3:0]  dram_full = (tpc ? 4'd2 : 4'd1) + 4'd1 + dram_cols;
	// A short-pitch write on the open row starts with one silent state.
	wire [3:0]  dram_hit_len = dram_cols + {3'd0, we_i && !dram_long};

	// States before any WAIT-driven extension, and whether WAIT is sampled.
	reg [3:0] base_states;
	reg       sample_wait;
	always @* begin
		case (tgt)
		T_ROM, T_RAM: begin base_states = 4'd1; sample_wait = 1'b0; end
		T_PERIPH:     begin base_states = 4'd3; sample_wait = 1'b0; end
		T_MPX:        begin base_states = 4'd4; sample_wait = 1'b1; end
		T_DRAM:       begin
			// Tp (one or two) plus Tr on a row miss, then one or two columns.
			base_states = hit_now ? dram_hit_len : dram_full;
			sample_wait = dram_long;
		end
		default: begin
			if (long_area) begin
				base_states = 4'd1 + {1'b0, long_wait};
				sample_wait = we_i || rw_bit;
			end else begin
				base_states = (we_i || rw_bit) ? 4'd2 : 4'd1;
				sample_wait = we_i || rw_bit;
			end
		end
		endcase
	end

	// --------------------------------------------------------- CBR refresh
	// TRp, TRr, TRc, plus RLW wait states when the area 1 read pitch is long.
	// RAS is always raised for a refresh, dropping the burst-mode row.
	// CBR needs only RAS and CAS, so it runs beside reads of other areas. A
	// DRAM access or external write is never held off by a pending refresh:
	// the write strobe is also the DRAM's WE and must stay high through CBR,
	// so the refresh starts in that access's last state and holds off the
	// next one instead.
	wire [3:0] ref_len   = 4'd3 + (rw[1] ? ({2'd0, rlw} + 4'd1) : 4'd0);
	wire       ref_clash = req_i && ((tgt == T_DRAM)
	                     || (we_i && ((tgt == T_EXT) || (tgt == T_MPX))));

	// ----------------------------------------------------------- sequencer
	// The first state of an access is the one in which the request appears,
	// so a one-state space acknowledges without any register in between.
	wire [3:0] cur        = busy ? cnt : base_states;
	wire       last_state = (cur == 4'd1) && (!sample_wait || wait_n_i);
	// The sub-access counter is only cleared at the end of the state that
	// starts an access, so the first state of a new request has to use zero
	// rather than what the previous request left behind.
	wire [2:0] sub_cur    = busy ? sub : 3'd0;
	wire       last_sub   = (sub_cur == (nsub - 3'd1));
	wire       ref_start  = (ref_pending != 4'd0) && !refreshing
	                        && (!ref_clash || (last_state && last_sub));
	wire       ref_block  = ref_clash && refreshing;
	wire       running    = req_i && !ref_block;
	wire       ext_active  = running && ((tgt == T_EXT) || (tgt == T_MPX));
	wire       dram_active = running && (tgt == T_DRAM);

	assign ack_o  = running && last_state && last_sub;
	assign hold_o = (running && !(last_state && last_sub)) || ref_block;
	assign rd_take_o = running && last_state && !we_i;
	assign cbr_o     = refreshing;

	// DRAM phases within one access: precharge, row, then one or two columns.
	// On a burst hit only the columns run.
	wire dram_row = dram_active && !hit_now && (cur == dram_cols + 4'd1);
	wire dram_col = dram_active && (cur <= dram_cols);

	// RAS or CAS falls half way through its state (DCR.CDTY's 35% duty is
	// drawn as 50%). A long-pitch column holds CAS low through Tc2; a burst
	// pulses CAS once per column so the DRAM latches each one.
	reg  phase2;
	wire col_first  = dram_col && (cur == dram_cols);
	wire ras_strobe = dram_row && phase2;
	wire cas_strobe = dram_col && (phase2 || !col_first);

	// ------------------------------------------------- sub-access address
	// Big-endian order: the most significant part of the operand comes first.
	reg [1:0] sub_off;
	always @* begin
		if (width == 2'd2) sub_off = {sub_cur[0], 1'b0};
		else               sub_off = sub_cur[1:0];
	end
	wire [31:0] sub_addr = (width == 2'd3) ? {addr_i[31:2], 2'b00}
	                     : (sz_i == SZ_L)  ? {addr_i[31:2], 2'b00} + {30'd0, sub_off}
	                     : (sz_i == SZ_W)  ? {addr_i[31:1], 1'b0}  + {30'd0, sub_off}
	                     :                   addr_i;

	// ------------------------------------------------ write data placement
	reg [15:0] wr_word;
	reg [1:0]  wr_be;                 // {upper byte, lower byte} of the 16-bit bus
	always @* begin
		wr_word = 16'd0;
		wr_be   = 2'b00;
		if (width == 2'd2) begin
			case (sz_i)
			// A byte write drives only its own lane; the floating other
			// lane is driven as zero, which is what the real board reads.
			SZ_B: begin
				wr_word = addr_i[0] ? {8'd0, wdata_i[7:0]} : {wdata_i[7:0], 8'd0};
				wr_be   = addr_i[0] ? 2'b01 : 2'b10;
			end
			SZ_W: begin wr_word = wdata_i[15:0]; wr_be = 2'b11; end
			default: begin
				wr_word = sub_cur[0] ? wdata_i[15:0] : wdata_i[31:16];
				wr_be   = 2'b11;
			end
			endcase
		end else begin
			case (sz_i)
			SZ_B:    wr_word = {8'd0, wdata_i[7:0]};
			SZ_W:    wr_word = {8'd0, sub_cur[0] ? wdata_i[7:0] : wdata_i[15:8]};
			default: wr_word = {8'd0, (sub_cur == 3'd0) ? wdata_i[31:24]
			                        : (sub_cur == 3'd1) ? wdata_i[23:16]
			                        : (sub_cur == 3'd2) ? wdata_i[15:8]
			                        :                     wdata_i[7:0]};
			endcase
			wr_be = 2'b01;
		end
	end

	// ------------------------------------------------------ register block
	wire periph_sel = req_i && (tgt == T_PERIPH);
	// The register space is 16 bits wide, so a longword access is two
	// sub-accesses to two different registers; the register cycle hangs off
	// the sub-access address.
	wire [8:0] pa   = sub_addr[8:0];
	// H'5FFFFA0 to H'5FFFFB3 only; the watchdog at H'5FFFFB8 and SBYCR at
	// H'5FFFFBC share the block, so the range stops at RTCOR.
	wire bsc_reg    = periph_sel && (pa[8:5] == 4'hD) && (pa[4:1] <= 4'h9);
	wire [3:0] rsel = pa[4:1];

	// A read selects the register from T2, where its value is taken.
	assign psel_o   = periph_sel && running && ((cur == 4'd1) || (!we_i && (cur == 4'd2)));
	assign paddr_o  = pa;
	assign pwr_o    = we_i;
	assign pbe_o    = (sz_i == SZ_B) ? (sub_addr[0] ? 2'b01 : 2'b10) : 2'b11;
	assign pwdata_o = (sz_i == SZ_B) ? {wdata_i[7:0], wdata_i[7:0]}
	                : (width == 2'd2) && (sz_i == SZ_L) && (sub_cur == 3'd0)
	                  ? wdata_i[31:16] : wdata_i[15:0];

	reg [15:0] bsc_rdata;
	always @* begin
		case (rsel)
		4'h0:    bsc_rdata = bcr;
		4'h1:    bsc_rdata = wcr1;
		4'h2:    bsc_rdata = wcr2;
		4'h3:    bsc_rdata = wcr3;
		4'h4:    bsc_rdata = dcr;
		4'h5:    bsc_rdata = pcr;
		4'h6:    bsc_rdata = {8'd0, rcr};
		4'h7:    bsc_rdata = {8'd0, rtcsr};
		4'h8:    bsc_rdata = {8'd0, rtcnt};
		default: bsc_rdata = {8'd0, rtcor};
		endcase
	end

	wire [15:0] pread = bsc_reg ? bsc_rdata : prdata_i;

	// A register is read as it stands at the end of T2, so an event in T3
	// (a count, a flag, an input capture) is not in the value read.
	reg [15:0] pread_q;
	always @(posedge clk_i) if (ce_i) pread_q <= pread;

	// -------------------------------------------------- read data assembly
	// The register space is 16 bits wide like an external area, so it goes
	// through the same assembly; only the source of the word differs.
	wire [15:0] rd_word = (tgt == T_PERIPH) ? pread_q : d_i;

	reg [31:0] next_rbuf;
	always @* begin
		next_rbuf = rbuf;
		if (width == 2'd2) begin
			case (sz_i)
			SZ_B:    next_rbuf = {24'd0, addr_i[0] ? rd_word[7:0] : rd_word[15:8]};
			SZ_W:    next_rbuf = {16'd0, rd_word};
			default: next_rbuf = sub_cur[0] ? {rbuf[31:16], rd_word} : {rd_word, 16'd0};
			endcase
		end else begin
			next_rbuf = {rbuf[23:0], rd_word[7:0]};
		end
	end

	// On-chip ROM and RAM hand back a whole longword, so the byte or word the
	// instruction asked for is picked out of it here. Big-endian: the byte at
	// a longword address is the most significant one.
	wire [31:0] onchip_q = (tgt == T_ROM) ? rom_q_i : ram_q_i;
	reg [31:0] onchip_rd;
	always @* begin
		case (sz_i)
		SZ_B: case (addr_i[1:0])
			2'd0:    onchip_rd = {24'd0, onchip_q[31:24]};
			2'd1:    onchip_rd = {24'd0, onchip_q[23:16]};
			2'd2:    onchip_rd = {24'd0, onchip_q[15:8]};
			default: onchip_rd = {24'd0, onchip_q[7:0]};
		endcase
		SZ_W:    onchip_rd = addr_i[1] ? {16'd0, onchip_q[15:0]}
		                               : {16'd0, onchip_q[31:16]};
		default: onchip_rd = onchip_q;
		endcase
	end

	// ------------------------------------------------------------- outputs
	assign mem_addr_o = sub_addr;
	assign rom_sel_o  = req_i && (tgt == T_ROM);
	assign ram_sel_o  = req_i && (tgt == T_RAM);
	assign ram_we_o   = ram_sel_o && we_i && running && (cur == 4'd1);
	assign ram_wd_o   = (sz_i == SZ_B) ? {4{wdata_i[7:0]}}
	                  : (sz_i == SZ_W) ? {2{wdata_i[15:0]}} : wdata_i;
	assign ram_be_o   = (sz_i == SZ_B) ? (4'b1000 >> addr_i[1:0])
	                  : (sz_i == SZ_W) ? (addr_i[1] ? 4'b0011 : 4'b1100)
	                  :                  4'b1111;

	// During the row phase of a multiplexed DRAM access the upper internal
	// address is driven on the low pins, shifted by MXC.
	wire [21:0] row_pins = (mxc == 2'd0) ? {6'd0, addr_i[23:8]}
	                     : (mxc == 2'd1) ? {7'd0, addr_i[23:9]}
	                     :                 {8'd0, addr_i[23:10]};

	assign a_o = (mxe && dram_row) ? row_pins : sub_addr[21:0];

	// The external strobes fall half way into the first state and release
	// when the last one ends, so the two back-to-back cycles of a longword
	// on a 16-bit bus reach the device as two strobes.
	wire first_state = (cur == base_states);
	wire ext_strobe  = ext_active && (!first_state || phase2);

	assign cs_n_o  = ext_strobe ? ~(8'd1 << area) : 8'hFF;

	// Byte lanes: a write drives only the bytes it is writing, a read takes
	// the whole width.
	wire sel_hi = we_i ? wr_be[1] : 1'b1;
	wire sel_lo = we_i ? wr_be[0] : 1'b1;
	wire bus8   = (width == 2'd1);

	assign rd_n_o  = ~((ext_strobe || dram_col) && !we_i);
	// BAS swaps WRH for LBS and WRL for WR; the byte strobes are for 16-bit
	// spaces only, and an 8-bit space always uses WRL.
	assign wrh_n_o = ~((ext_strobe || dram_col) && we_i && !bus8
	                   && (bas ? sel_lo : sel_hi) && !(dram_active && !cw2));
	assign wrl_n_o = ~((ext_strobe || dram_col) && we_i
	                   && (bus8 ? 1'b1
	                     : dram_active && !cw2 ? 1'b1
	                     : bas ? 1'b1 : sel_lo));
	assign ah_o    = running && (tgt == T_MPX) && (cur == base_states);
	assign d_o     = wr_word;
	assign d_oe_o  = (ext_active || dram_col) && we_i;

	// CAS before RAS during a refresh, otherwise CAS in the column phase.
	assign ras_n_o  = refreshing ? ~(ref_cnt <= (ref_len - 4'd1))
	                : dram_active ? ~(ras_strobe || dram_col || hit_now)
	                : ras_idle;
	assign cash_n_o = refreshing ? ~(ref_cnt <= ref_len)
	                : ~(cas_strobe && !bus8 && !cw2 && sel_hi);
	assign casl_n_o = refreshing ? ~(ref_cnt <= ref_len)
	                : ~(cas_strobe && (bus8 || cw2 || sel_lo));

	always @* begin
		case (tgt)
		T_ROM, T_RAM: rdata_o = onchip_rd;
		default:      rdata_o = next_rbuf;
		endcase
	end

	// ----------------------------------------------------- password writes
	// RCR, RTCSR, RTCNT and RTCOR take only word writes whose upper byte is
	// the register's password.
	wire        pw_ok_rcr   = (pwdata_o[15:8] == 8'h5A);
	wire        pw_ok_rtcsr = (pwdata_o[15:8] == 8'hA5);
	wire        pw_ok_rtcnt = (pwdata_o[15:8] == 8'h69);
	wire        pw_ok_rtcor = (pwdata_o[15:8] == 8'h96);
	wire        reg_wr = bsc_reg && we_i && running && (cur == 4'd1);
	wire        word_wr = (sz_i != SZ_B);

	// -------------------------------------------------- refresh prescaler
	reg [11:0] ref_div;
	reg        ref_tick;
	always @* begin
		case (cks)
		3'd1:    ref_tick = (ref_div[0]    == 1'b0);
		3'd2:    ref_tick = (ref_div[2:0]  == 3'd0);
		3'd3:    ref_tick = (ref_div[4:0]  == 5'd0);
		3'd4:    ref_tick = (ref_div[6:0]  == 7'd0);
		3'd5:    ref_tick = (ref_div[8:0]  == 9'd0);
		3'd6:    ref_tick = (ref_div[10:0] == 11'd0);
		3'd7:    ref_tick = (ref_div       == 12'd0);
		default: ref_tick = 1'b0;
		endcase
	end

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_BSC0, SS_BSC1, SS_BSC2, SS_BSC3;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1, ss_dout2, ss_dout3;

	wire [63:0] SS_BSC0_BACK = {wcr3, wcr2, wcr1, bcr};
	wire [63:0] SS_BSC1_BACK = {rtcor, rtcnt, rtcsr, rcr, pcr, dcr};
	wire [63:0] SS_BSC2_BACK = {1'b0, cnt, sub, row_addr, rbuf};
	wire [63:0] SS_BSC3_BACK = {38'd0, ref_cnt, phase2, ref_pending, ref_div,
	                            row_valid, acc_hit, ras_idle, refreshing, busy};

	ss_reg #(.ADDR (SSW_BSC_BASE + 0),
	         .DEFAULT ({16'hF800, 16'hFFFF, 16'hFFFF, 16'h0000})) u_ss0 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout0),
		.din_i (SS_BSC0_BACK), .dout_o (SS_BSC0)
	);
	ss_reg #(.ADDR (SSW_BSC_BASE + 1),
	         .DEFAULT ({8'hFF, 8'h00, 8'h00, 8'h00, 16'h0000, 16'h0000}))
	       u_ss1 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout1),
		.din_i (SS_BSC1_BACK), .dout_o (SS_BSC1)
	);
	ss_reg #(.ADDR (SSW_BSC_BASE + 2), .DEFAULT (64'd0)) u_ss2 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout2),
		.din_i      (SS_BSC2_BACK),
		.dout_o     (SS_BSC2)
	);
	// RAS idles high between accesses.
	ss_reg #(.ADDR (SSW_BSC_BASE + 3), .DEFAULT ({61'd0, 1'b1, 2'b00})) u_ss3 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout3),
		.din_i      (SS_BSC3_BACK),
		.dout_o     (SS_BSC3)
	);
	assign ss_dout = ss_dout0 | ss_dout1 | ss_dout2 | ss_dout3;

	// ---------------------------------------------------------------- state
	// Cleared by the enable that starts a state, set by the one half way in.
	always @(posedge clk_i) begin
		if (rst_i)      phase2 <= SS_BSC3[21];
		else if (ce_i)  phase2 <= 1'b0;
		else if (ceh_i) phase2 <= 1'b1;
	end

	always @(posedge clk_i) begin
		if (rst_i) begin
			bcr  <= SS_BSC0[15:0];  wcr1 <= SS_BSC0[31:16];
			wcr2 <= SS_BSC0[47:32]; wcr3 <= SS_BSC0[63:48];
			dcr  <= SS_BSC1[15:0];  pcr  <= SS_BSC1[31:16];
			rcr  <= SS_BSC1[39:32]; rtcsr <= SS_BSC1[47:40];
			rtcnt <= SS_BSC1[55:48]; rtcor <= SS_BSC1[63:56];
			rbuf <= SS_BSC2[31:0];  row_addr <= SS_BSC2[55:32];
			sub  <= SS_BSC2[58:56]; cnt  <= SS_BSC2[62:59];
			busy <= SS_BSC3[0];     refreshing <= SS_BSC3[1];
			ras_idle <= SS_BSC3[2]; acc_hit <= SS_BSC3[3];
			row_valid <= SS_BSC3[4];
			ref_div <= SS_BSC3[16:5]; ref_pending <= SS_BSC3[20:17];
			ref_cnt <= SS_BSC3[25:22];
		end else if (ce_i) begin
			ref_div <= ref_div + 12'd1;
			if (cks != 3'd0 && ref_tick) begin
				// The match clears RTCNT as it happens, so the period is
				// RTCOR counts.
				if (rtcnt + 8'd1 == rtcor) begin
					rtcnt    <= 8'd0;
					rtcsr[7] <= 1'b1;
					if (rfshe && !rmode && (ref_pending != 4'hF))
						ref_pending <= ref_pending + 4'd1;
				end else begin
					rtcnt <= rtcnt + 8'd1;
				end
			end

			if (running) begin
				if (!busy) begin
					rbuf <= 32'd0;
					sub  <= 3'd0;
					acc_hit <= dram_hit;
					if (tgt == T_DRAM) begin
						row_valid <= 1'b1;
						row_addr  <= this_row;
					end else if (!rasd) begin
						ras_idle  <= 1'b1;
						row_valid <= 1'b0;
					end
				end
				if (!last_state) begin
					// A WAIT-extended final state holds the count at one.
					busy <= 1'b1;
					cnt  <= (cur > 4'd1) ? cur - 4'd1 : 4'd1;
				end else begin
					if (!we_i) rbuf <= next_rbuf;
					if (last_sub) begin
						busy <= 1'b0;
						// RAS-down mode holds the row open across other
						// accesses so a later burst can resume.
						if (tgt == T_DRAM) begin
							ras_idle  <= ~(rasd || be);
							row_valid <= rasd || be;
						end
					end else begin
						busy    <= 1'b1;
						sub     <= sub_cur + 3'd1;
						acc_hit <= (tgt == T_DRAM) ? be : 1'b0;
						cnt     <= (tgt == T_DRAM)
						           ? (be ? dram_cols : dram_full) : base_states;
					end
				end
			end else if (!req_i) begin
				busy <= 1'b0;
			end

			if (refreshing) begin
				if (ref_cnt > 4'd1) ref_cnt <= ref_cnt - 4'd1;
				else                refreshing <= 1'b0;
			end else if (ref_start) begin
				// Refresh outranks the CPU and the DMAC for the DRAM.
				refreshing  <= 1'b1;
				ref_cnt     <= ref_len;
				ref_pending <= ref_pending - 4'd1;
				ras_idle    <= 1'b1;
				row_valid   <= 1'b0;
			end

			// register writes take effect in the last state of the access
			if (reg_wr) begin
				case (rsel)
				4'h0: bcr  <= merge(bcr,  pwdata_o, pbe_o) & 16'hF800;
				4'h1: wcr1 <= merge(wcr1, pwdata_o, pbe_o) | 16'h00FD;
				4'h2: wcr2 <= merge(wcr2, pwdata_o, pbe_o);
				4'h3: wcr3 <= merge(wcr3, pwdata_o, pbe_o) & 16'hF800;
				4'h4: dcr  <= merge(dcr,  pwdata_o, pbe_o) & 16'hFF00;
				4'h5: begin
					pcr <= merge(pcr, pwdata_o, pbe_o) & 16'hF800;
					// PEF is cleared by writing 0 after reading it as 1
					if (pbe_o[1] && !pwdata_o[15]) pcr[15] <= 1'b0;
				end
				4'h6: if (word_wr && pw_ok_rcr)   rcr   <= pwdata_o[7:0] & 8'hF0;
				4'h7: if (word_wr && pw_ok_rtcsr) begin
					rtcsr[6:0] <= {pwdata_o[6:3], 3'd0};
					if (!pwdata_o[7]) rtcsr[7] <= 1'b0;
				end
				4'h8: if (word_wr && pw_ok_rtcnt) rtcnt <= pwdata_o[7:0];
				4'h9: if (word_wr && pw_ok_rtcor) rtcor <= pwdata_o[7:0];
				default: ;
				endcase
			end
		end
	end

	function automatic [15:0] merge(input [15:0] old, input [15:0] nw,
	                                input [1:0] lanes);
		merge = {lanes[1] ? nw[15:8] : old[15:8],
		         lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction
endmodule
