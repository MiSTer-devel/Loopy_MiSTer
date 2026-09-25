// Copyright (c) 2026 Jamie Blanks
//
// SH-1 CPU core for the SH7021. Five stages, IF ID EX MA WB, one memory port
// shared by IF and MA. One CPU state is one ce_i pulse. A slot runs its MA
// access first and its fetch second, so a slot needing the bus for both takes
// the sum of their bus cycles (the manual's split-slot rule). Results are
// written at the end of the slot that produced them, so ID holds an
// instruction back one slot when it would use or overwrite the register a
// load in EX is about to fill. Fetch reads a longword at an aligned PC into
// a one-line buffer; a fetch starting at 4n+2 reads only that word.

module sh1_core (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,          // synchronous, held for one or more states
	input  wire        manual_rst_i,   // reset type: vectors 2 and 3 instead of 0 and 1

	// One memory port. Read data is right justified; the bus does the width
	// conversion. bus_ack_i marks the last state of the access.
	output wire        bus_req_o,
	output wire [31:0] bus_addr_o,
	output wire        bus_we_o,
	output wire [1:0]  bus_sz_o,
	output wire [31:0] bus_wdata_o,
	output wire        bus_ifetch_o,
	input  wire [31:0] bus_rdata_i,
	input  wire        bus_ack_i,

	// Interrupt request from the INTC, already prioritised against SR.I3-I0.
	input  wire        int_req_i,
	input  wire [7:0]  int_vec_i,
	input  wire [3:0]  int_level_i,
	output wire        int_ack_o,

	input  wire        dma_addr_err_i, // DMA address error, vector 10
	output wire        sleep_o,
	// TAS.B holds the bus between the read and the write.
	output wire        bus_lock_o,

	output wire [3:0]  sr_mask_o,    // SR.I3-I0, for the interrupt controller
	// The only place a savestate may stop the core: the instruction in EX has
	// finished its last step, nothing is outstanding on the bus, no exception
	// is part way in, and the multiplier is not holding the slot.
	output wire        ss_ready_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "sh1_defs.svh"
	`include "ss_map.svh"

	// exception classes carried in ex_exc_kind
	// The reset sequence holds EX for three states.
	localparam [3:0] HOLD_FOR_RESET = 4'd3;

	localparam [2:0] XC_ILLEGAL = 3'd0;
	localparam [2:0] XC_SLOT    = 3'd1;
	localparam [2:0] XC_ADDR    = 3'd2;
	localparam [2:0] XC_DMAERR  = 3'd3;
	localparam [2:0] XC_INT     = 3'd4;

	// ------------------------------------------------------------- state
	reg  [31:0] pc_f;
	reg  [31:0] gbr, vbr, pr;
	reg         sr_t, sr_s, sr_m, sr_q;
	reg  [3:0]  sr_i;

	reg         fb_v;
	reg  [29:0] fb_tag;
	reg  [31:0] fb_data;

	reg         id_v;
	reg  [15:0] id_ir;
	reg  [31:0] id_pc;
	reg         id_dslot;
	reg  [31:0] id_dtarget;

	// One instruction of slack behind ID. IF carries on while ID is stalled,
	// so the fetch that would otherwise land in the same slot as a later MA
	// has already happened and that slot does not split.
	reg         iq_v;
	reg  [15:0] iq_ir;
	reg  [31:0] iq_pc;

	reg         ex_v;
	reg  [31:0] ex_pc;
	reg         ex_dslot;
	reg  [31:0] ex_dtarget;
	reg  [3:0]  ex_step, ex_hold;
	reg  [3:0]  ex_spc;
	reg  [3:0]  ex_rn, ex_rm;
	reg  [3:0]  ex_a_sel, ex_b_sel;
	reg  [31:0] ex_imm;
	reg  [5:0]  ex_alu_op;
	reg  [3:0]  ex_sh_op;
	reg  [3:0]  ex_wr_sel;
	reg         ex_t_wr, ex_addr_a, ex_uses_mac;
	reg  [2:0]  ex_ma_op;
	reg  [1:0]  ex_ma_sz;
	reg  [3:0]  ex_ld_sel, ex_sd_sel;
	reg  [2:0]  ex_mac_op, ex_br_op;
	reg  [1:0]  ex_lop;
	reg  [2:0]  ex_exc_kind;
	reg  [7:0]  ex_vec;
	reg  [3:0]  ex_newmask;

	reg         ma_v;
	reg  [2:0]  ma_op;
	reg  [1:0]  ma_sz;
	reg  [31:0] ma_addr, ma_wdata;
	reg  [15:0] ma_opb;
	reg  [3:0]  ma_ld_sel, ma_ld_reg;
	reg  [2:0]  ma_mac_op;
	reg         ma_uses_mac, ma_mac_extra, ma_mac_go;
	reg         ma_to_seq, ma_seq_slot;

	reg  [31:0] seq_t0, seq_t1;
	reg  [31:0] ma_rdata_r;
	reg         ph;
	reg         int_block;
	reg         sleeping;
	reg  [31:0] sleep_pc;
	reg         mac_busy_d;
	reg         in_reset_seq;
	reg         manual_rst;      // reset type latched at the reset itself
	reg         dae_pend;
	reg  [31:0] dae_pc;
	reg         dmae_pend, dmae_d;

	// ------------------------------------------------- forward declarations
	wire        slot_done, id_adv, id_free_next, ma_bus, if_bus, do_ma, do_if;
	wire        ma_addr_err, mac_stall;
	wire        if_hit, if_want, if_addr_err;
	wire [31:0] ma_dat;
	wire        mac_busy;
	wire [31:0] mach, macl;
	wire        take_exc;
	// Declared early because the exception arbitration below reads them.
	reg         pc_load, flush_id, set_dslot, go_sleep;

	// --------------------------------------------------------- sub-blocks
	wire [3:0]  seq_r15 = ((ex_spc == SP_RTE) || (ex_spc == SP_TRAPA)
	                    || (ex_spc == SP_EXC) || (ex_spc == SP_RESET)) ? 4'd15 : ex_rn;
	wire [31:0] rn_val, rm_val, r0_val;
	reg         wa_we;  reg [3:0] wa_ra;  reg [31:0] wa_d;
	reg         wb_we;  reg [3:0] wb_ra;  reg [31:0] wb_d;

	// Thirteen words carry the whole pipeline, EX and MA latches included, so
	// a state can be taken part way through a multi-step instruction.
	// manual_rst is latched from the pin again by the restoring reset.
	wire [63:0] ss_q_regs, ss_q_mac;

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_CPU0, SS_CPU1, SS_CPU2, SS_CPU3, SS_CPU4, SS_CPU5, SS_CPU6,
	            SS_CPU7, SS_CPU8, SS_CPU9, SS_CPU10, SS_CPU11, SS_CPU12;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ssq0, ssq1, ssq2, ssq3, ssq4, ssq5, ssq6, ssq7, ssq8, ssq9,
	            ssq10, ssq11, ssq12;

	wire [63:0] SS_CPU0_BACK = {gbr, pc_f};
	wire [63:0] SS_CPU1_BACK = {pr, vbr};
	wire [63:0] SS_CPU2_BACK = {1'b0, fb_v, fb_tag, fb_data};
	wire [63:0] SS_CPU3_BACK = {id_dtarget, id_pc};
	wire [63:0] SS_CPU4_BACK = {iq_pc, iq_ir, id_ir};
	wire [63:0] SS_CPU5_BACK = {ex_dtarget, ex_pc};
	wire [63:0] SS_CPU6_BACK = {sleep_pc, ex_imm};
	wire [63:0] SS_CPU7_BACK = {2'd0, ex_uses_mac, ex_addr_a, ex_t_wr,
	                            ex_ma_sz, ex_ma_op, ex_newmask, ex_sd_sel,
	                            ex_ld_sel, ex_wr_sel, ex_sh_op, ex_alu_op,
	                            ex_b_sel, ex_a_sel, ex_rm, ex_rn, ex_spc,
	                            ex_hold, ex_step};
	wire [63:0] SS_CPU8_BACK = {8'd0, ma_seq_slot, ma_to_seq, ma_mac_go,
	                            ma_mac_extra, ma_uses_mac, ma_opb, ma_mac_op,
	                            ma_ld_reg, ma_ld_sel, ma_sz, ma_op, ex_vec,
	                            ex_exc_kind, ex_lop, ex_br_op, ex_mac_op};
	wire [63:0] SS_CPU9_BACK = {ma_wdata, ma_addr};
	wire [63:0] SS_CPU10_BACK = {seq_t1, seq_t0};
	wire [63:0] SS_CPU11_BACK = {dae_pc, ma_rdata_r};
	wire [63:0] SS_CPU12_BACK = {42'd0, dmae_d, dmae_pend, dae_pend,
	                             in_reset_seq, mac_busy_d, sleeping,
	                             int_block, ph, ma_v, ex_dslot, ex_v, iq_v,
	                             id_dslot, id_v, sr_i, sr_q, sr_m, sr_s, sr_t};

	ss_reg #(.ADDR (SSW_SH1_BASE + 0), .DEFAULT (64'd0)) u_ss0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq0),
		.din_i      (SS_CPU0_BACK),
		.dout_o     (SS_CPU0)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 1), .DEFAULT (64'd0)) u_ss1 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq1),
		.din_i      (SS_CPU1_BACK),
		.dout_o     (SS_CPU1)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 2), .DEFAULT (64'd0)) u_ss2 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq2),
		.din_i      (SS_CPU2_BACK),
		.dout_o     (SS_CPU2)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 3), .DEFAULT (64'd0)) u_ss3 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq3),
		.din_i      (SS_CPU3_BACK),
		.dout_o     (SS_CPU3)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 4),
	         .DEFAULT ({32'd0, 16'h0009, 16'h0009}))
	       u_ss4 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ssq4),
		.din_i (SS_CPU4_BACK), .dout_o (SS_CPU4)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 5), .DEFAULT (64'd0)) u_ss5 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq5),
		.din_i      (SS_CPU5_BACK),
		.dout_o     (SS_CPU5)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 6), .DEFAULT (64'd0)) u_ss6 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq6),
		.din_i      (SS_CPU6_BACK),
		.dout_o     (SS_CPU6)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 7),
	         .DEFAULT ({2'd0, 1'b0, 1'b0, 1'b0, SZ_L, MA_NONE, 4'hF, D_RM,
	                    W_NONE, W_NONE, SH_SHLL, AOP_ADD, B_ZERO, A_ZERO,
	                    4'd0, 4'd0, SP_RESET, HOLD_FOR_RESET, 4'd0}))
	       u_ss7 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ssq7),
		.din_i (SS_CPU7_BACK), .dout_o (SS_CPU7)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 8),
	         .DEFAULT ({8'd0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 16'd0, MACOP_NONE,
	                    4'd0, W_NONE, SZ_L, MA_NONE, 8'd0, XC_ILLEGAL,
	                    LOP_AND, BR_NONE, MACOP_NONE}))
	       u_ss8 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ssq8),
		.din_i (SS_CPU8_BACK), .dout_o (SS_CPU8)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 9), .DEFAULT (64'd0)) u_ss9 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq9),
		.din_i      (SS_CPU9_BACK),
		.dout_o     (SS_CPU9)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 10), .DEFAULT (64'd0)) u_ss10 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq10),
		.din_i      (SS_CPU10_BACK),
		.dout_o     (SS_CPU10)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 11), .DEFAULT (64'd0)) u_ss11 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ssq11),
		.din_i      (SS_CPU11_BACK),
		.dout_o     (SS_CPU11)
	);
	ss_reg #(.ADDR (SSW_SH1_BASE + 12),
	         .DEFAULT ({42'd0, 1'b0, 1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 1'b0,
	                    1'b0, 1'b0, 1'b1, 1'b0, 1'b0, 1'b0, 4'hF, 1'b0, 1'b0,
	                    1'b0, 1'b0}))
	       u_ss12 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ssq12),
		.din_i (SS_CPU12_BACK), .dout_o (SS_CPU12)
	);

	assign ss_dout = ss_q_regs | ss_q_mac
	               | ssq0 | ssq1 | ssq2 | ssq3 | ssq4 | ssq5
	               | ssq6 | ssq7 | ssq8 | ssq9 | ssq10 | ssq11
	               | ssq12;

	sh1_regfile u_regs (
		.clk_i   (clk_i),
		.ce_i    (ce_i & slot_done),
		.rst_i   (rst_i),
		.rn_i    (seq_r15),
		.rm_i    (ex_rm),
		.rn_o    (rn_val),
		.rm_o    (rm_val),
		.r0_o    (r0_val),
		.we_a_i  (wa_we),
		.wa_a_i  (wa_ra),
		.wd_a_i  (wa_d),
		.we_b_i  (wb_we),
		.wa_b_i  (wb_ra),
		.wd_b_i  (wb_d),
		.ss_din  (ss_din),
		.ss_addr (ss_addr),
		.ss_wren (ss_wren),
		.ss_rst  (ss_rst),
		.ss_dout (ss_q_regs)
	);

	wire [31:0] ma_mac_wdata = (ma_op == MA_LOAD) ? ma_dat : ma_wdata;
	wire [15:0] mac_a = (ma_mac_op == MACOP_MACW) ? seq_t0[15:0] : ma_wdata[15:0];
	wire [15:0] mac_b = (ma_mac_op == MACOP_MACW) ? ma_dat[15:0] : ma_opb;
	wire        mac_start = ce_i & slot_done & ma_v & ma_mac_go;

	sh1_mac u_mac (
		.clk_i   (clk_i),
		.ce_i    (ce_i),
		.rst_i   (rst_i),
		.start_i (mac_start),
		.op_i    (ma_mac_op),
		.wdata_i (ma_mac_wdata),
		.mul_a_i (mac_a),
		.mul_b_i (mac_b),
		.s_i     (sr_s),
		.mach_o  (mach),
		.macl_o  (macl),
		.busy_o  (mac_busy),
		.ss_din  (ss_din),
		.ss_addr (ss_addr),
		.ss_wren (ss_wren),
		.ss_rst  (ss_rst),
		.ss_dout (ss_q_mac)
	);

	wire [31:0] sr = {22'd0, sr_m, sr_q, sr_i, 2'd0, sr_s, sr_t};

	// ------------------------------------------------------------- decode
	wire        dec_legal, dec_t_wr, dec_addr_a, dec_uses_mac, dec_int_disable;
	wire [3:0]  dec_a_sel, dec_b_sel, dec_sh_op, dec_wr_sel, dec_ld_sel;
	wire [3:0]  dec_sd_sel, dec_spc;
	wire [31:0] dec_imm;
	wire [5:0]  dec_alu_op;
	wire [2:0]  dec_ma_op, dec_mac_op, dec_br_op;
	wire [1:0]  dec_ma_sz, dec_ex_slots, dec_lop;

	sh1_decode u_dec (
		.ir_i          (id_ir),
		.legal_o       (dec_legal),
		.a_sel_o       (dec_a_sel),
		.b_sel_o       (dec_b_sel),
		.imm_o         (dec_imm),
		.alu_op_o      (dec_alu_op),
		.sh_op_o       (dec_sh_op),
		.wr_sel_o      (dec_wr_sel),
		.t_wr_o        (dec_t_wr),
		.addr_a_o      (dec_addr_a),
		.ma_op_o       (dec_ma_op),
		.ma_sz_o       (dec_ma_sz),
		.ld_sel_o      (dec_ld_sel),
		.sd_sel_o      (dec_sd_sel),
		.mac_op_o      (dec_mac_op),
		.uses_mac_o    (dec_uses_mac),
		.br_op_o       (dec_br_op),
		.spc_o         (dec_spc),
		.ex_slots_o    (dec_ex_slots),
		.lop_o         (dec_lop),
		.int_disable_o (dec_int_disable)
	);

	wire [3:0] dec_rn = id_ir[11:8];
	wire [3:0] dec_rm = id_ir[7:4];

	// ---------------------------------------------------- EX operand path
	wire [31:0] pc_rel = ex_dslot ? (ex_dtarget + 32'd2) : (ex_pc + 32'd4);

	reg [31:0] opa, opb;
	always @* begin
		case (ex_a_sel)
		A_RN:    opa = rn_val;
		A_RM:    opa = rm_val;
		A_R0:    opa = r0_val;
		A_PC:    opa = pc_rel;
		A_PC4:   opa = {pc_rel[31:2], 2'b00};
		A_GBR:   opa = gbr;
		A_VBR:   opa = vbr;
		A_SR:    opa = sr;
		A_PR:    opa = pr;
		default: opa = 32'd0;
		endcase
		case (ex_b_sel)
		B_RN:    opb = rn_val;
		B_RM:    opb = rm_val;
		B_IMM:   opb = ex_imm;
		B_GBR:   opb = gbr;
		default: opb = 32'd0;
		endcase
	end

	wire [31:0] alu_res;
	wire        alu_t, alu_m, alu_q;
	sh1_alu u_alu (
		.a_i   (opa),
		.b_i   (opb),
		.op_i  (ex_alu_op),
		.t_i   (sr_t),
		.m_i   (sr_m),
		.q_i   (sr_q),
		.res_o (alu_res),
		.t_o   (alu_t),
		.m_o   (alu_m),
		.q_o   (alu_q)
	);

	wire [31:0] sh_res;
	wire        sh_t;
	sh1_shifter u_sh (.a_i (opa), .op_i (ex_sh_op), .t_i (sr_t),
	                  .res_o (sh_res), .t_o (sh_t));

	wire        use_sh  = (ex_alu_op == AOP_SHIFT);
	wire [31:0] ex_res  = use_sh ? sh_res : alu_res;
	wire        ex_tnew = use_sh ? sh_t   : alu_t;
	wire [31:0] ex_addr = ex_addr_a ? opa : alu_res;
	wire        ex_mq   = (ex_alu_op == AOP_DIV0S) || (ex_alu_op == AOP_DIV0U)
	                   || (ex_alu_op == AOP_DIV1);

	// The MACH and MACL store forms read the multiplier at the MA stage,
	// where ma_sd picks them up.
	reg [31:0] sd_val;
	always @* begin
		case (ex_sd_sel)
		D_RN:    sd_val = rn_val;
		D_R0:    sd_val = r0_val;
		D_SR:    sd_val = sr;
		D_GBR:   sd_val = gbr;
		D_VBR:   sd_val = vbr;
		D_PR:    sd_val = pr;
		default: sd_val = rm_val;
		endcase
	end

	reg [31:0] br_target;
	reg        br_take, br_delayed;
	always @* begin
		br_target  = 32'd0;
		br_take    = 1'b0;
		br_delayed = 1'b0;
		case (ex_br_op)
		BR_BT:     begin br_target = pc_rel + ex_imm; br_take = sr_t;  end
		BR_BF:     begin br_target = pc_rel + ex_imm; br_take = ~sr_t; end
		BR_DISP,
		BR_DISP_L: begin br_target = pc_rel + ex_imm; br_take = 1'b1; br_delayed = 1'b1; end
		BR_REG,
		BR_REG_L:  begin br_target = rn_val;          br_take = 1'b1; br_delayed = 1'b1; end
		BR_RTS:    begin br_target = pr;              br_take = 1'b1; br_delayed = 1'b1; end
		default: ;
		endcase
	end

	// ------------------------------------------------------- memory port
	// Fetching from on-chip peripheral space (area 5, A27 low) is an address
	// error, as is an odd instruction address.
	assign if_addr_err = pc_f[0] || ((pc_f[27] == 1'b0) && (pc_f[26:24] == 3'd5));
	assign if_hit      = fb_v && (fb_tag == pc_f[31:2]);
	// IF waits while the instruction it fetched last still waits for ID. A
	// delayed branch computing its target holds its slot instruction back
	// the same way, so nothing past the slot is fetched.
	wire   dbr_redirect = ex_v && (ex_spc == SP_NONE) && (ex_step == 4'd0) && br_delayed;
	assign if_want     = !sleeping && !in_reset_seq && !iq_v && !dbr_redirect;
	assign if_bus      = if_want && !if_hit && !if_addr_err;

	// Only the 32-bit on-chip ROM (area 0 in mode 2) and RAM (area 7, A27
	// high) hand over two instructions per fetch. External spaces fetch one
	// instruction per bus cycle.
	wire        if_onchip = (pc_f[26:24] == 3'd0) || (pc_f[27] && (pc_f[26:24] == 3'd7));
	wire        if_pair   = if_onchip && !pc_f[1];
	wire [1:0]  if_sz     = if_pair ? SZ_L : SZ_W;
	wire [31:0] if_addr   = if_pair ? {pc_f[31:2], 2'b00} : pc_f;

	assign ma_bus = ma_v && ((ma_op == MA_LOAD) || (ma_op == MA_STORE));
	// An MA that touches a multiplier still running waits for it before it
	// starts (the manual's extended M--A). It is held off the bus because
	// STS.L takes the data the multiplier is still computing.
	assign do_ma  = (ph == 1'b0) && ma_bus && !mac_stall;
	assign do_if  = ((ph == 1'b1) || ((ph == 1'b0) && !ma_bus)) && if_bus;

	// STS.L MACH/MACL,@-Rn takes its data from the multiplier in the MA stage,
	// the same stage the producing LDS or multiply writes it in, so a value
	// picked up back in EX would be one slot stale.
	wire        ma_sd_mac = (ma_op == MA_STORE) && ma_uses_mac;
	wire [31:0] ma_sd     = ma_sd_mac ? ((ma_mac_op == MACOP_LDH) ? mach : macl)
	                                  : ma_wdata;

	assign bus_req_o    = do_ma | do_if;
	assign bus_addr_o   = do_ma ? ma_addr : if_addr;
	assign bus_we_o     = do_ma && (ma_op == MA_STORE);
	assign bus_sz_o     = do_ma ? ma_sz : if_sz;
	assign bus_wdata_o  = ma_sd;
	assign bus_ifetch_o = do_if;

	assign mac_stall  = ma_v && ma_uses_mac
	                 && (mac_busy || (ma_mac_extra && mac_busy_d));
	wire ma_half_done = !ma_bus || (ph == 1'b1) || (do_ma && bus_ack_i);
	wire if_half_done = !if_bus || (do_if && bus_ack_i);
	assign slot_done  = ma_half_done && if_half_done && !mac_stall;

	assign ma_dat = (do_ma && bus_ack_i) ? bus_rdata_i : ma_rdata_r;

	wire [15:0] if_word = if_hit ? (pc_f[1] ? fb_data[15:0] : fb_data[31:16])
	                             : (if_pair ? bus_rdata_i[31:16] : bus_rdata_i[15:0]);
	wire        if_got  = if_want && !if_addr_err && (if_hit || (do_if && bus_ack_i));

	// --------------------------------------------------- pipeline advance
	wire       ex_last    = (ex_step == ex_hold);
	// STS MACH/MACL,Rn returns its value in WB just like a load, with the
	// same load-use contention, so it interlocks too.
	wire       ex_is_load = ex_v && ex_last
	                     && ((ex_ma_op == MA_LOAD) || (ex_ma_op == MA_MACRD));
	wire [3:0] ex_ld_reg  = (ex_ld_sel == W_R0) ? 4'd0 : ex_rn;
	wire       ex_ld_gpr  = ex_is_load && ((ex_ld_sel == W_RN) || (ex_ld_sel == W_R0));

	// Everything the instruction in ID would read, so the load in EX can be
	// checked against it. The write target counts as a use.
	wire dec_st = (dec_ma_op == MA_STORE);
	wire dec_reads_q =
		  ((dec_a_sel == A_RN) && (dec_rn == ex_ld_reg))
		|| ((dec_a_sel == A_RM) && (dec_rm == ex_ld_reg))
		|| ((dec_a_sel == A_R0) && (ex_ld_reg == 4'd0))
		|| ((dec_b_sel == B_RN) && (dec_rn == ex_ld_reg))
		|| ((dec_b_sel == B_RM) && (dec_rm == ex_ld_reg))
		|| (dec_st && (((dec_sd_sel == D_RM) && (dec_rm == ex_ld_reg))
		            || ((dec_sd_sel == D_RN) && (dec_rn == ex_ld_reg))
		            || ((dec_sd_sel == D_R0) && (ex_ld_reg == 4'd0))))
		|| (((dec_br_op == BR_REG) || (dec_br_op == BR_REG_L))
		    && (dec_rn == ex_ld_reg))
		|| (((dec_spc == SP_RTE) || (dec_spc == SP_TRAPA))
		    && (ex_ld_reg == 4'd15));

	wire dec_writes_q =
		  ((dec_wr_sel == W_RN) && (dec_rn == ex_ld_reg))
		|| ((dec_wr_sel == W_RM) && (dec_rm == ex_ld_reg))
		|| ((dec_wr_sel == W_R0) && (ex_ld_reg == 4'd0));

	// LDS.L @Rm+,PR is an ordinary load: PR arrives at the end of its MA, so
	// an instruction reading PR in the next slot waits for it. LDC.L holds
	// EX for two slots after its MA, so SR, GBR and VBR are already written.
	wire dec_reads_pr = (dec_a_sel == A_PR) || (dec_st && (dec_sd_sel == D_PR))
	                 || (dec_br_op == BR_RTS);

	wire ld_sys_use = ex_is_load && (ex_ld_sel == W_PR) && dec_reads_pr;

	wire load_use = id_v && ((ex_ld_gpr && (dec_reads_q || dec_writes_q))
	                         || ld_sys_use);
	wire ex_free  = !ex_v || ex_last;
	// SLEEP stops the machine in its last slot: the instruction already
	// fetched behind it is neither decoded nor allowed to raise anything.
	wire ex_to_sleep = ex_v && (ex_spc == SP_SLEEP) && ex_last;

	// A data address error stops the instruction behind it from starting, so
	// the address the error stacks really is the next one to run.
	assign id_adv = id_v && ex_free && !load_use && !sleeping && !in_reset_seq
	                && !ex_to_sleep && !ma_addr_err;
	assign id_free_next = id_adv || !id_v;

	// ------------------------------------------------ exception arbitration
	wire dec_rewrites_pc = (dec_br_op != BR_NONE) || (dec_spc == SP_RTE)
	                    || (dec_spc == SP_TRAPA);
	// Neither an address error nor an interrupt may be inserted between a
	// delayed branch and its slot instruction; both wait for the instruction
	// after the slot.
	wire in_dslot  = id_v && id_dslot;
	wire int_ready = int_req_i && !int_block && !id_dslot;

	// A fetch that cannot happen because the address is illegal raises the
	// address error instead of stalling the pipeline for ever.
	wire fetch_err = if_want && if_addr_err;

	// The DMAC's address error flag stays set until software clears it, so the
	// exception is taken from the edge: one error, one exception.
	wire dmae_edge  = dma_addr_err_i && !dmae_d;
	wire xc_dmaerr  = dmae_pend && !in_dslot;
	wire xc_fetch   = (fetch_err || dae_pend) && !in_dslot;
	wire xc_int     = int_ready;
	wire xc_illegal = id_v && !id_dslot && !dec_legal;
	wire xc_slot    = id_v &&  id_dslot && (!dec_legal || dec_rewrites_pc);

	wire xc_any = xc_dmaerr || xc_fetch || xc_int || xc_illegal || xc_slot;

	// Nothing is accepted in the slot that rewrites the PC: ID holds either
	// an instruction the branch is discarding or the delay slot, and an
	// exception against either would stack the wrong return address.
	assign take_exc = ex_free && !in_reset_seq && !ex_to_sleep
	               && !pc_load && !set_dslot
	               && ((sleeping && (xc_int || xc_dmaerr))
	                || (!sleeping && (xc_fetch || xc_dmaerr
	                                  || (id_v && !load_use && xc_any))));

	reg [2:0]  xc_kind;
	reg [7:0]  xc_vec;
	reg [31:0] xc_pc;
	always @* begin
		// Reset outranks everything and is handled by in_reset_seq. Then
		// address error, then interrupt, then the instruction exceptions.
		if (xc_dmaerr)      begin xc_kind = XC_DMAERR;  xc_vec = 8'd10; end
		else if (xc_fetch)  begin xc_kind = XC_ADDR;    xc_vec = 8'd9;  end
		else if (xc_int)    begin xc_kind = XC_INT;     xc_vec = int_vec_i; end
		else if (xc_slot)   begin xc_kind = XC_SLOT;    xc_vec = 8'd6;  end
		else                begin xc_kind = XC_ILLEGAL; xc_vec = 8'd4;  end

		if (sleeping)            xc_pc = sleep_pc;
		else if (dae_pend)       xc_pc = dae_pc;
		else if (fetch_err)      xc_pc = pc_f;
		else if (xc_slot)        xc_pc = id_dtarget;
		else                     xc_pc = id_pc;
	end

	assign int_ack_o  = ce_i && slot_done && take_exc && (xc_kind == XC_INT);
	assign sr_mask_o  = sr_i;
	assign sleep_o    = sleeping;
	assign bus_lock_o = ex_v && (ex_spc == SP_TAS);
	assign ss_ready_o = slot_done && !bus_req_o && !take_exc && !in_reset_seq
	                    && ex_free && !mac_stall;

	// ----------------------------------------------------- MA writeback
	reg [31:0] ext_dat;
	always @* begin
		case (ma_sz)
		SZ_B:    ext_dat = {{24{ma_dat[7]}},  ma_dat[7:0]};
		SZ_W:    ext_dat = {{16{ma_dat[15]}}, ma_dat[15:0]};
		default: ext_dat = ma_dat;
		endcase
	end
	wire [31:0] ma_wbval = (ma_op == MA_MACRD)
	                     ? ((ma_mac_op == MACOP_LDH) ? mach : macl)
	                     : ext_dat;
	wire ma_wb = ma_v && ((ma_op == MA_LOAD) || (ma_op == MA_MACRD)) && !ma_to_seq;

	// -------------------------------------------- EX slot-end action decode
	reg        nma_v;
	reg [2:0]  nma_op;
	reg [1:0]  nma_sz;
	reg [31:0] nma_addr, nma_wdata;
	reg [15:0] nma_opb;
	reg [3:0]  nma_ld_sel, nma_ld_reg;
	reg [2:0]  nma_mac_op;
	reg        nma_uses_mac, nma_mac_extra, nma_mac_go, nma_to_seq, nma_seq_slot;
	reg        ex_wr_en;
	reg [3:0]  ex_wr_tgt, ex_wr_reg;
	reg [31:0] ex_wr_val;
	reg        set_t, t_val, set_mq, set_mask;
	reg [31:0] pc_new;
	reg        set_seq1;
	reg [31:0] seq1_val;

	wire [3:0] ex_rn_idx = seq_r15;

	// Word data at an odd address and longword data off a four-byte boundary
	// are address errors.
	assign ma_addr_err = nma_v && ((nma_op == MA_LOAD) || (nma_op == MA_STORE))
	                  && (((nma_sz == SZ_W) && nma_addr[0])
	                   || ((nma_sz == SZ_L) && (nma_addr[1:0] != 2'b00)));
	// A stack or vector access inside an exception sequence keeps that
	// sequence's saved PC, and one raised while an address error is itself
	// stacking is ignored, which stops an endless chain on a misaligned SP.
	wire in_exc_seq  = (ex_spc == SP_EXC) || (ex_spc == SP_TRAPA)
	                || (ex_spc == SP_RESET);
	wire in_exc_addr = (ex_spc == SP_EXC)
	                && ((ex_exc_kind == XC_ADDR) || (ex_exc_kind == XC_DMAERR));
	wire [7:0] logmem_b  = (ex_lop == LOP_OR)  ? (seq_t0[7:0] | ex_imm[7:0])
	                     : (ex_lop == LOP_XOR) ? (seq_t0[7:0] ^ ex_imm[7:0])
	                     :                       (seq_t0[7:0] & ex_imm[7:0]);

	always @* begin
		nma_v = 1'b0; nma_op = MA_NONE; nma_sz = SZ_L;
		nma_addr = 32'd0; nma_wdata = 32'd0; nma_opb = 16'd0;
		nma_ld_sel = W_NONE; nma_ld_reg = 4'd0; nma_mac_op = MACOP_NONE;
		nma_uses_mac = 1'b0; nma_mac_extra = 1'b0; nma_mac_go = 1'b0;
		nma_to_seq = 1'b0; nma_seq_slot = 1'b0;
		ex_wr_en = 1'b0; ex_wr_tgt = W_NONE; ex_wr_reg = 4'd0; ex_wr_val = 32'd0;
		set_t = 1'b0; t_val = sr_t; set_mq = 1'b0; set_mask = 1'b0;
		pc_load = 1'b0; pc_new = 32'd0; flush_id = 1'b0; set_dslot = 1'b0;
		go_sleep = 1'b0; set_seq1 = 1'b0; seq1_val = 32'd0;

		if (ex_v) case (ex_spc)
		// ------------------------------------------------------- ordinary
		SP_NONE: if (ex_step == 4'd0) begin
			ex_wr_en  = (ex_wr_sel != W_NONE);
			ex_wr_tgt = ex_wr_sel;
			ex_wr_reg = (ex_wr_sel == W_RM) ? ex_rm
			          : (ex_wr_sel == W_R0) ? 4'd0 : ex_rn_idx;
			ex_wr_val = ex_res;
			set_t     = ex_t_wr;  t_val = ex_tnew;
			set_mq    = ex_mq;

			if (ex_ma_op != MA_NONE) begin
				nma_v        = 1'b1;
				nma_op       = ex_ma_op;
				nma_sz       = ex_ma_sz;
				nma_addr     = ex_addr;
				nma_wdata    = (ex_ma_op == MA_STORE) ? sd_val
				             : (ex_ma_op == MA_MUL)   ? opa : ex_res;
				nma_opb      = opb[15:0];
				nma_ld_sel   = ex_ld_sel;
				nma_ld_reg   = (ex_ld_sel == W_R0) ? 4'd0 : ex_rn_idx;
				nma_mac_op   = ex_mac_op;
				nma_uses_mac = ex_uses_mac;
				nma_mac_extra= (ex_ma_op == MA_STORE) && ex_uses_mac;
				nma_mac_go   = (ex_ma_op == MA_MACWR) || (ex_ma_op == MA_MUL)
				            || ((ex_ma_op == MA_LOAD)
				                && ((ex_ld_sel == W_MACH) || (ex_ld_sel == W_MACL)));
			end

			if (br_take) begin
				pc_load = 1'b1;
				pc_new  = br_target;
				if (br_delayed) set_dslot = 1'b1;
				else            flush_id  = 1'b1;
			end
			if ((ex_br_op == BR_DISP_L) || (ex_br_op == BR_REG_L)) begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_PR; ex_wr_val = pc_rel;
			end
		end
		// ------------------------- AND.B/OR.B/XOR.B/TST.B #imm,@(R0,GBR)
		SP_LOGMEM: case (ex_step)
			4'd0: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_B;
				nma_addr = ex_addr; nma_to_seq = 1'b1;
				set_seq1 = 1'b1; seq1_val = ex_addr;
			end
			4'd2: begin
				if (ex_lop == LOP_TST) begin
					set_t = 1'b1; t_val = (logmem_b == 8'd0);
				end else begin
					nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_B;
					nma_addr = seq_t1; nma_wdata = {24'd0, logmem_b};
				end
			end
			default: ;
		endcase
		// ------------------------------------------------------ TAS.B @Rn
		SP_TAS: case (ex_step)
			4'd0: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_B;
				nma_addr = ex_addr; nma_to_seq = 1'b1;
				set_seq1 = 1'b1; seq1_val = ex_addr;
			end
			4'd2: begin
				set_t = 1'b1; t_val = (seq_t0[7:0] == 8'd0);
				nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_B;
				nma_addr = seq_t1; nma_wdata = {24'd0, 1'b1, seq_t0[6:0]};
			end
			default: ;
		endcase
		// ------------------------------------- LDC.L @Rm+,SR/GBR/VBR
		SP_LDCL: if (ex_step == 4'd0) begin
			ex_wr_en = 1'b1; ex_wr_tgt = ex_wr_sel; ex_wr_reg = ex_rn_idx;
			ex_wr_val = ex_res;
			nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
			nma_addr = ex_addr; nma_ld_sel = ex_ld_sel;
		end
		// ----------------------------------------------- MAC.W @Rm+,@Rn+
		SP_MACW: case (ex_step)
			4'd0: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = ex_rn;
				ex_wr_val = rn_val + 32'd2;
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_W;
				nma_addr = rn_val; nma_to_seq = 1'b1;
			end
			4'd1: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RM; ex_wr_reg = ex_rm;
				ex_wr_val = rm_val + 32'd2;
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_W;
				nma_addr = rm_val; nma_to_seq = 1'b1; nma_seq_slot = 1'b1;
				nma_mac_op = MACOP_MACW; nma_mac_go = 1'b1; nma_uses_mac = 1'b1;
			end
			default: ;
		endcase
		// ------------------------------------------------------------ RTE
		SP_RTE: case (ex_step)
			4'd0: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val + 32'd4;
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = rn_val; nma_to_seq = 1'b1;
			end
			4'd1: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val + 32'd4;
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = rn_val; nma_ld_sel = W_SR;
			end
			4'd2: begin
				pc_load = 1'b1; pc_new = seq_t0; set_dslot = 1'b1;
			end
			default: ;
		endcase
		// ---------------------------------------------------------- TRAPA
		SP_TRAPA: case (ex_step)
			4'd1: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val - 32'd4;
				nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_L;
				nma_addr = rn_val - 32'd4; nma_wdata = sr;
			end
			4'd2: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val - 32'd4;
				nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_L;
				nma_addr = rn_val - 32'd4; nma_wdata = ex_pc + 32'd2;
			end
			4'd3: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = vbr + ex_imm; nma_to_seq = 1'b1;
			end
			4'd5: begin
				pc_load = 1'b1; pc_new = seq_t0; flush_id = 1'b1;
			end
			default: ;
		endcase
		// ------------------------------------------ exception / interrupt
		SP_EXC: case (ex_step)
			4'd1: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val - 32'd4;
				nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_L;
				nma_addr = rn_val - 32'd4; nma_wdata = sr;
			end
			4'd2: begin
				ex_wr_en = 1'b1; ex_wr_tgt = W_RN; ex_wr_reg = 4'd15;
				ex_wr_val = rn_val - 32'd4;
				nma_v = 1'b1; nma_op = MA_STORE; nma_sz = SZ_L;
				nma_addr = rn_val - 32'd4; nma_wdata = ex_pc;
				set_mask = (ex_exc_kind == XC_INT);
			end
			4'd4: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = vbr + {22'd0, ex_vec, 2'b00}; nma_to_seq = 1'b1;
			end
			// The handler's first fetch is the slot after the last EX: eight
			// states after an interrupt or address error sequence started,
			// which with the priority decision is the manual's 10 or 11.
			default: if (ex_step == ex_hold) begin
				pc_load = 1'b1; pc_new = seq_t0; flush_id = 1'b1;
			end
		endcase
		// ---------------------------------------------------------- reset
		SP_RESET: case (ex_step)
			4'd0: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = manual_rst ? 32'h00000008 : 32'h00000000;
			end
			4'd1: begin
				nma_v = 1'b1; nma_op = MA_LOAD; nma_sz = SZ_L;
				nma_addr = manual_rst ? 32'h0000000C : 32'h00000004;
				nma_ld_sel = W_RN; nma_ld_reg = 4'd15;
				pc_load = 1'b1; pc_new = ma_dat;
			end
			default: ;
		endcase
		// ---------------------------------------------------------- SLEEP
		SP_SLEEP: if (ex_step == ex_hold) go_sleep = 1'b1;
		default: ;
		endcase
	end

	// register-file write ports, driven from the two writeback paths
	always @* begin
		wa_we = ex_wr_en && ((ex_wr_tgt == W_RN) || (ex_wr_tgt == W_RM)
		                  || (ex_wr_tgt == W_R0));
		wa_ra = ex_wr_reg;
		wa_d  = ex_wr_val;
		wb_we = ma_wb && ((ma_ld_sel == W_RN) || (ma_ld_sel == W_RM)
		                || (ma_ld_sel == W_R0));
		wb_ra = ma_ld_reg;
		wb_d  = ma_wbval;
	end

	// ------------------------------------------------------- the slot clock
	// Interrupts and address errors are the manual's ten-stage sequence;
	// the instruction exceptions are the nine-stage one.
	wire [3:0] hold_for_exc   = ((xc_kind == XC_INT) || (xc_kind == XC_ADDR)
	                          || (xc_kind == XC_DMAERR)) ? 4'd7 : 4'd6;
	wire [3:0] hold_for_trapa = 4'd6;

	always @(posedge clk_i) begin
		if (rst_i) begin
			pc_f <= SS_CPU0[31:0];
			gbr <= SS_CPU0[63:32];
			vbr <= SS_CPU1[31:0];
			pr <= SS_CPU1[63:32];
			fb_data <= SS_CPU2[31:0];
			fb_tag <= SS_CPU2[61:32];
			fb_v <= SS_CPU2[62];
			id_pc <= SS_CPU3[31:0];
			id_dtarget <= SS_CPU3[63:32];
			id_ir <= SS_CPU4[15:0];
			iq_ir <= SS_CPU4[31:16];
			iq_pc <= SS_CPU4[63:32];
			ex_pc <= SS_CPU5[31:0];
			ex_dtarget <= SS_CPU5[63:32];
			ex_imm <= SS_CPU6[31:0];
			sleep_pc <= SS_CPU6[63:32];
			ex_step <= SS_CPU7[3:0];
			ex_hold <= SS_CPU7[7:4];
			ex_spc <= SS_CPU7[11:8];
			ex_rn <= SS_CPU7[15:12];
			ex_rm <= SS_CPU7[19:16];
			ex_a_sel <= SS_CPU7[23:20];
			ex_b_sel <= SS_CPU7[27:24];
			ex_alu_op <= SS_CPU7[33:28];
			ex_sh_op <= SS_CPU7[37:34];
			ex_wr_sel <= SS_CPU7[41:38];
			ex_ld_sel <= SS_CPU7[45:42];
			ex_sd_sel <= SS_CPU7[49:46];
			ex_newmask <= SS_CPU7[53:50];
			ex_ma_op <= SS_CPU7[56:54];
			ex_ma_sz <= SS_CPU7[58:57];
			ex_t_wr <= SS_CPU7[59];
			ex_addr_a <= SS_CPU7[60];
			ex_uses_mac <= SS_CPU7[61];
			ex_mac_op <= SS_CPU8[2:0];
			ex_br_op <= SS_CPU8[5:3];
			ex_lop <= SS_CPU8[7:6];
			ex_exc_kind <= SS_CPU8[10:8];
			ex_vec <= SS_CPU8[18:11];
			ma_op <= SS_CPU8[21:19];
			ma_sz <= SS_CPU8[23:22];
			ma_ld_sel <= SS_CPU8[27:24];
			ma_ld_reg <= SS_CPU8[31:28];
			ma_mac_op <= SS_CPU8[34:32];
			ma_opb <= SS_CPU8[50:35];
			ma_uses_mac <= SS_CPU8[51];
			ma_mac_extra <= SS_CPU8[52];
			ma_mac_go <= SS_CPU8[53];
			ma_to_seq <= SS_CPU8[54];
			ma_seq_slot <= SS_CPU8[55];
			ma_addr <= SS_CPU9[31:0];
			ma_wdata <= SS_CPU9[63:32];
			seq_t0 <= SS_CPU10[31:0];
			seq_t1 <= SS_CPU10[63:32];
			ma_rdata_r <= SS_CPU11[31:0];
			dae_pc <= SS_CPU11[63:32];
			sr_t <= SS_CPU12[0];
			sr_s <= SS_CPU12[1];
			sr_m <= SS_CPU12[2];
			sr_q <= SS_CPU12[3];
			sr_i <= SS_CPU12[7:4];
			id_v <= SS_CPU12[8];
			id_dslot <= SS_CPU12[9];
			iq_v <= SS_CPU12[10];
			ex_v <= SS_CPU12[11];
			ex_dslot <= SS_CPU12[12];
			ma_v <= SS_CPU12[13];
			ph <= SS_CPU12[14];
			int_block <= SS_CPU12[15];
			sleeping <= SS_CPU12[16];
			mac_busy_d <= SS_CPU12[17];
			in_reset_seq <= SS_CPU12[18];
			dae_pend <= SS_CPU12[19];
			dmae_pend <= SS_CPU12[20];
			dmae_d <= SS_CPU12[21];
			// NMI's level while RES is low picks the reset type, so it is
			// taken once rather than sampled through the vector reads.
			manual_rst <= manual_rst_i;
		end else if (ce_i) begin
			mac_busy_d <= mac_busy;
			dmae_d     <= dma_addr_err_i;
			if (dmae_edge) dmae_pend <= 1'b1;
			if (do_ma && bus_ack_i) ma_rdata_r <= bus_rdata_i;

			if (!slot_done) begin
				if (do_ma && bus_ack_i && if_bus) ph <= 1'b1;
			end else begin
				ph <= 1'b0;

				// ---- writeback of the MA that ran in this slot
				if (ma_wb) begin
					case (ma_ld_sel)
					W_SR:  begin
						sr_t <= ma_wbval[0];  sr_s <= ma_wbval[1];
						sr_i <= ma_wbval[7:4]; sr_q <= ma_wbval[8];
						sr_m <= ma_wbval[9];
					end
					W_GBR: gbr <= ma_wbval;
					W_VBR: vbr <= ma_wbval;
					W_PR:  pr  <= ma_wbval;
					W_PC:  begin pc_f <= ma_wbval; fb_v <= 1'b0; end
					default: ;
					endcase
				end
				if (ma_v && ma_to_seq) begin
					if (ma_seq_slot) seq_t1 <= ma_dat;
					else             seq_t0 <= ma_dat;
				end
				if (set_seq1) seq_t1 <= seq1_val;

				// ---- EX system-register writes
				if (ex_wr_en) case (ex_wr_tgt)
					W_SR:  begin
						sr_t <= ex_wr_val[0];  sr_s <= ex_wr_val[1];
						sr_i <= ex_wr_val[7:4]; sr_q <= ex_wr_val[8];
						sr_m <= ex_wr_val[9];
					end
					W_GBR: gbr <= ex_wr_val;
					W_VBR: vbr <= ex_wr_val;
					W_PR:  pr  <= ex_wr_val;
					default: ;
				endcase
				if (set_t)  sr_t <= t_val;
				if (set_mq) begin sr_m <= alu_m; sr_q <= alu_q; end
				if (set_mask) sr_i <= ex_newmask;

				// ---- fetch
				if (do_if && bus_ack_i && if_pair) begin
					fb_v    <= 1'b1;
					fb_tag  <= pc_f[31:2];
					fb_data <= bus_rdata_i;
				end

				// ---- MA stage load. A misaligned access is held back; its
				// address error is taken once the instruction has finished and
				// stacks the next instruction (in a delay slot, the target).
				if (ma_addr_err && !in_exc_addr) begin
					dae_pend <= 1'b1;
					dae_pc   <= in_exc_seq ? ex_pc
					          : ex_dslot   ? ex_dtarget : ex_pc + 32'd2;
				end
				ma_v         <= nma_v && !ma_addr_err;
				ma_op        <= nma_op;
				ma_sz        <= nma_sz;
				ma_addr      <= nma_addr;
				ma_wdata     <= nma_wdata;
				ma_opb       <= nma_opb;
				ma_ld_sel    <= nma_ld_sel;
				ma_ld_reg    <= nma_ld_reg;
				ma_mac_op    <= nma_mac_op;
				ma_uses_mac  <= nma_uses_mac;
				ma_mac_extra <= nma_mac_extra;
				ma_mac_go    <= nma_mac_go;
				ma_to_seq    <= nma_to_seq;
				ma_seq_slot  <= nma_seq_slot;

				// ---- PC and fetch redirect
				if (pc_load) begin
					pc_f <= pc_new;
				end else if (if_got && !in_reset_seq) begin
					pc_f <= pc_f + 32'd2;
				end
				// A store into the buffered line invalidates it, so the SH-1's
				// cacheless behaviour holds.
				if (ma_v && (ma_op == MA_STORE) && (ma_addr[31:2] == fb_tag))
					fb_v <= 1'b0;

				// ---- SLEEP
				if (go_sleep) begin
					sleeping <= 1'b1;
					sleep_pc <= ex_pc + 32'd2;
				end

				// ---- IF to the queue and on into ID. A fetch bypasses the
				// queue when ID is about to be free, so a flush costs no
				// extra slot.
				if (flush_id) begin
					id_v <= 1'b0;
					iq_v <= 1'b0;
				end else if (id_free_next) begin
					if (iq_v) begin
						id_v     <= 1'b1;
						id_ir    <= iq_ir;
						id_pc    <= iq_pc;
						id_dslot <= 1'b0;
						iq_v     <= if_got;
						iq_ir    <= if_word;
						iq_pc    <= pc_f;
					end else if (if_got) begin
						id_v     <= 1'b1;
						id_ir    <= if_word;
						id_pc    <= pc_f;
						id_dslot <= 1'b0;
					end else begin
						id_v <= 1'b0;
					end
				end else if (!iq_v && if_got) begin
					iq_v  <= 1'b1;
					iq_ir <= if_word;
					iq_pc <= pc_f;
				end
				// A branch discards whatever was queued behind it; the delay
				// slot is already in ID and survives.
				if (pc_load) iq_v <= 1'b0;
				if (set_dslot) begin
					id_dslot   <= 1'b1;
					id_dtarget <= pc_new;
				end

				// ---- ID to EX, or an exception in its place
				if (take_exc) begin
					ex_v        <= 1'b1;
					ex_spc      <= SP_EXC;
					ex_step     <= 4'd0;
					ex_hold     <= hold_for_exc;
					ex_pc       <= xc_pc;
					ex_exc_kind <= xc_kind;
					ex_vec      <= xc_vec;
					ex_newmask  <= int_level_i;
					ex_dslot    <= 1'b0;
					ex_ma_op    <= MA_NONE;
					ex_br_op    <= BR_NONE;
					ex_ld_sel   <= W_NONE;
					ex_uses_mac <= 1'b0;
					ex_rn       <= 4'd15;
					int_block   <= 1'b0;
					sleeping    <= 1'b0;
					dae_pend    <= 1'b0;
					if (xc_kind == XC_DMAERR) dmae_pend <= dmae_edge;
				end else if (id_adv && !flush_id) begin
					ex_v        <= 1'b1;
					ex_pc       <= id_pc;
					ex_dslot    <= id_dslot;
					ex_dtarget  <= id_dtarget;
					ex_step     <= 4'd0;
					ex_spc      <= dec_spc;
					ex_hold     <= (dec_spc == SP_TRAPA) ? hold_for_trapa
					             : {2'd0, dec_ex_slots};
					ex_rn       <= dec_rn;
					ex_rm       <= dec_rm;
					ex_a_sel    <= dec_a_sel;
					ex_b_sel    <= dec_b_sel;
					ex_imm      <= dec_imm;
					ex_alu_op   <= dec_alu_op;
					ex_sh_op    <= dec_sh_op;
					ex_wr_sel   <= dec_wr_sel;
					ex_t_wr     <= dec_t_wr;
					ex_addr_a   <= dec_addr_a;
					ex_ma_op    <= dec_ma_op;
					ex_ma_sz    <= dec_ma_sz;
					ex_ld_sel   <= dec_ld_sel;
					ex_sd_sel   <= dec_sd_sel;
					ex_mac_op   <= dec_mac_op;
					ex_uses_mac <= dec_uses_mac;
					ex_br_op    <= dec_br_op;
					ex_lop      <= dec_lop;
					int_block   <= dec_int_disable;
				end else if (ex_v) begin
					if (!ex_last) ex_step <= ex_step + 4'd1;
					else          ex_v    <= 1'b0;
				end

				// ---- reset sequence exit
				if (in_reset_seq && (ex_spc == SP_RESET) && (ex_step == 4'd2))
					in_reset_seq <= 1'b0;
			end
		end
	end
endmodule
