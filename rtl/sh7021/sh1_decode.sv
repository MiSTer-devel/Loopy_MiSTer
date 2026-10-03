// Copyright (c) 2026 Jamie Blanks
//
// SH-1 instruction decoder, one entry per row of the instruction table.
// The SH-2-only rows (BF/S, BT/S, BRAF, BSRF, DMULS.L, DMULU.L, DT, MAC.L,
// MUL.L) and every unassigned code leave `legal` low, which the core turns
// into a general illegal instruction. Purely combinational; the core
// registers the result at the ID/EX boundary.

module sh1_decode (
	input  wire [15:0] ir_i,

	output reg         legal_o,
	output reg  [3:0]  a_sel_o,       // ALU operand A source
	output reg  [3:0]  b_sel_o,       // ALU operand B source
	output reg  [31:0] imm_o,         // immediate or displacement, pre-scaled
	output reg  [5:0]  alu_op_o,
	output reg  [3:0]  sh_op_o,
	output reg  [3:0]  wr_sel_o,      // EX-stage register write target
	output reg         t_wr_o,        // EX stage writes T
	output reg         addr_a_o,      // MA address is operand A itself
	output reg  [2:0]  ma_op_o,
	output reg  [1:0]  ma_sz_o,
	output reg  [3:0]  ld_sel_o,      // MA-stage register write target
	output reg  [3:0]  sd_sel_o,      // store data source
	output reg  [2:0]  mac_op_o,
	output reg         uses_mac_o,    // contends with a multiply already running
	output reg  [2:0]  br_op_o,
	output reg  [3:0]  spc_o,         // multi-step instruction class
	output reg  [1:0]  ex_slots_o,    // extra slots the EX engine is held
	output reg  [1:0]  lop_o,         // SP_LOGMEM operation
	output reg         int_disable_o  // no interrupt between this and the next
);
	`include "sh1_defs.svh"

	wire [3:0]  nf   = ir_i[11:8];
	wire [3:0]  mf   = ir_i[7:4];
	wire [31:0] d4   = {28'd0, ir_i[3:0]};
	wire [31:0] d8   = {24'd0, ir_i[7:0]};
	wire [31:0] i8s  = {{24{ir_i[7]}}, ir_i[7:0]};
	wire [31:0] d8s2 = {{23{ir_i[7]}}, ir_i[7:0], 1'b0};
	wire [31:0] d12s2= {{19{ir_i[11]}}, ir_i[11:0], 1'b0};

	// Common shapes, applied before the per-opcode overrides below.
	task automatic clear_all;
		begin
			legal_o       = 1'b1;
			a_sel_o       = A_ZERO;
			b_sel_o       = B_ZERO;
			imm_o         = 32'd0;
			alu_op_o      = AOP_ADD;
			sh_op_o       = SH_SHLL;
			wr_sel_o      = W_NONE;
			t_wr_o        = 1'b0;
			addr_a_o      = 1'b0;
			ma_op_o       = MA_NONE;
			ma_sz_o       = SZ_L;
			ld_sel_o      = W_NONE;
			sd_sel_o      = D_RM;
			mac_op_o      = MACOP_NONE;
			uses_mac_o    = 1'b0;
			br_op_o       = BR_NONE;
			spc_o         = SP_NONE;
			ex_slots_o    = 2'd0;
			lop_o         = LOP_AND;
			int_disable_o = 1'b0;
		end
	endtask

	// Rn <- op(A, B)
	task automatic alu_rn(input [3:0] a, input [3:0] b, input [5:0] op, input t);
		begin
			a_sel_o = a; b_sel_o = b; alu_op_o = op;
			wr_sel_o = W_RN; t_wr_o = t;
		end
	endtask

	// T <- op(A, B), no register written
	task automatic alu_t(input [3:0] a, input [3:0] b, input [5:0] op);
		begin
			a_sel_o = a; b_sel_o = b; alu_op_o = op; t_wr_o = 1'b1;
		end
	endtask

	// Rn <- shift(Rn)
	task automatic shift_rn(input [3:0] op, input t);
		begin
			a_sel_o = A_RN; alu_op_o = AOP_SHIFT; sh_op_o = op;
			wr_sel_o = W_RN; t_wr_o = t;
		end
	endtask

	// load from A + B into `dst`
	task automatic load(input [3:0] a, input [3:0] b, input [1:0] sz, input [3:0] dst);
		begin
			a_sel_o = a; b_sel_o = b;
			ma_op_o = MA_LOAD; ma_sz_o = sz; ld_sel_o = dst;
		end
	endtask

	// store `src` to A + B
	task automatic store(input [3:0] a, input [3:0] b, input [1:0] sz, input [3:0] src);
		begin
			a_sel_o = a; b_sel_o = b;
			ma_op_o = MA_STORE; ma_sz_o = sz; sd_sel_o = src;
		end
	endtask

	// @Rm+ : the access uses Rm, and Rm advances by the operand size.
	// `hi` picks the bits 11-8 register field, which is where the group 0100
	// instructions carry their source register.
	task automatic postinc(input [1:0] sz, input hi);
		begin
			a_sel_o = hi ? A_RN : A_RM; b_sel_o = B_IMM; addr_a_o = 1'b1;
			wr_sel_o = hi ? W_RN : W_RM;
			imm_o = (sz == SZ_B) ? 32'd1 : (sz == SZ_W) ? 32'd2 : 32'd4;
		end
	endtask

	// @-Rn : Rn retreats by the operand size first, and that is the address
	task automatic predec(input [1:0] sz);
		begin
			a_sel_o = A_RN; b_sel_o = B_IMM; wr_sel_o = W_RN;
			imm_o = (sz == SZ_B) ? 32'hFFFFFFFF : (sz == SZ_W) ? 32'hFFFFFFFE : 32'hFFFFFFFC;
		end
	endtask

	always @* begin
		clear_all();
		case (ir_i[15:12])
		// ------------------------------------------------------------ 0000
		4'h0: case (ir_i[3:0])
			4'h2: begin                                  // STC SRc,Rn
				case (mf)
				4'h0: alu_rn(A_SR,  B_ZERO, AOP_ADD, 1'b0);
				4'h1: alu_rn(A_GBR, B_ZERO, AOP_ADD, 1'b0);
				4'h2: alu_rn(A_VBR, B_ZERO, AOP_ADD, 1'b0);
				default: legal_o = 1'b0;
				endcase
				int_disable_o = legal_o;
			end
			4'h3: legal_o = 1'b0;                        // BSRF/BRAF: SH-2 only
			4'h4: store(A_R0, B_RN, SZ_B, D_RM);         // MOV.B Rm,@(R0,Rn)
			4'h5: store(A_R0, B_RN, SZ_W, D_RM);
			4'h6: store(A_R0, B_RN, SZ_L, D_RM);
			4'h7: legal_o = 1'b0;                        // MUL.L: SH-2 only
			4'h8: begin                                  // CLRT / SETT / CLRMAC
				if (nf != 4'h0) legal_o = 1'b0;
				else case (mf)
					4'h0: begin alu_op_o = AOP_CLRT; t_wr_o = 1'b1; end
					4'h1: begin alu_op_o = AOP_SETT; t_wr_o = 1'b1; end
					4'h2: begin ma_op_o = MA_MACWR; mac_op_o = MACOP_CLR; uses_mac_o = 1'b1; end
					default: legal_o = 1'b0;
				endcase
			end
			4'h9: begin                                  // NOP / DIV0U / MOVT Rn
				if (mf == 4'h2) alu_rn(A_ZERO, B_ZERO, AOP_MOVT, 1'b0);
				else if (nf != 4'h0) legal_o = 1'b0;
				else if (mf == 4'h1) begin alu_op_o = AOP_DIV0U; t_wr_o = 1'b1; end
				else if (mf != 4'h0) legal_o = 1'b0;   // mf = 0 is NOP
			end
			4'hA: case (mf)                              // STS MACH/MACL/PR,Rn
				4'h0: begin ma_op_o = MA_MACRD; ld_sel_o = W_RN; uses_mac_o = 1'b1;
				            mac_op_o = MACOP_LDH; int_disable_o = 1'b1; end
				4'h1: begin ma_op_o = MA_MACRD; ld_sel_o = W_RN; uses_mac_o = 1'b1;
				            mac_op_o = MACOP_LDL; int_disable_o = 1'b1; end
				4'h2: begin alu_rn(A_PR, B_ZERO, AOP_ADD, 1'b0); int_disable_o = 1'b1; end
				default: legal_o = 1'b0;
			endcase
			4'hB: begin
				if (nf != 4'h0) legal_o = 1'b0;
				else case (mf)
					4'h0: begin br_op_o = BR_RTS; ex_slots_o = 2'd1; end
					4'h1: begin spc_o = SP_SLEEP; ex_slots_o = 2'd2; end
					4'h2: begin spc_o = SP_RTE; ex_slots_o = 2'd3; end
					default: legal_o = 1'b0;
				endcase
			end
			4'hC: load(A_R0, B_RM, SZ_B, W_RN);          // MOV.B @(R0,Rm),Rn
			4'hD: load(A_R0, B_RM, SZ_W, W_RN);
			4'hE: load(A_R0, B_RM, SZ_L, W_RN);
			4'hF: legal_o = 1'b0;                        // MAC.L: SH-2 only
			default: legal_o = 1'b0;
		endcase
		// ------------------------------------------------------------ 0001
		4'h1: begin store(A_RN, B_IMM, SZ_L, D_RM); imm_o = d4 << 2; end
		// ------------------------------------------------------------ 0010
		4'h2: case (ir_i[3:0])
			4'h0: store(A_RN, B_ZERO, SZ_B, D_RM);
			4'h1: store(A_RN, B_ZERO, SZ_W, D_RM);
			4'h2: store(A_RN, B_ZERO, SZ_L, D_RM);
			4'h4: begin predec(SZ_B); ma_op_o = MA_STORE; ma_sz_o = SZ_B; sd_sel_o = D_RM; end
			4'h5: begin predec(SZ_W); ma_op_o = MA_STORE; ma_sz_o = SZ_W; sd_sel_o = D_RM; end
			4'h6: begin predec(SZ_L); ma_op_o = MA_STORE; ma_sz_o = SZ_L; sd_sel_o = D_RM; end
			4'h7: alu_t (A_RN, B_RM, AOP_DIV0S);
			4'h8: alu_t (A_RN, B_RM, AOP_TST);
			4'h9: alu_rn(A_RN, B_RM, AOP_AND, 1'b0);
			4'hA: alu_rn(A_RN, B_RM, AOP_XOR, 1'b0);
			4'hB: alu_rn(A_RN, B_RM, AOP_OR,  1'b0);
			4'hC: alu_t (A_RN, B_RM, AOP_CMPSTR);
			4'hD: alu_rn(A_RM, B_RN, AOP_XTRCT, 1'b0);
			4'hE: begin a_sel_o = A_RN; b_sel_o = B_RM; ma_op_o = MA_MUL;
			            mac_op_o = MACOP_MULU; uses_mac_o = 1'b1; end
			4'hF: begin a_sel_o = A_RN; b_sel_o = B_RM; ma_op_o = MA_MUL;
			            mac_op_o = MACOP_MULS; uses_mac_o = 1'b1; end
			default: legal_o = 1'b0;
		endcase
		// ------------------------------------------------------------ 0011
		4'h3: case (ir_i[3:0])
			4'h0: alu_t (A_RN, B_RM, AOP_CMPEQ);
			4'h2: alu_t (A_RN, B_RM, AOP_CMPHS);
			4'h3: alu_t (A_RN, B_RM, AOP_CMPGE);
			4'h4: alu_rn(A_RN, B_RM, AOP_DIV1, 1'b1);
			4'h6: alu_t (A_RN, B_RM, AOP_CMPHI);
			4'h7: alu_t (A_RN, B_RM, AOP_CMPGT);
			4'h8: alu_rn(A_RN, B_RM, AOP_SUB,  1'b0);
			4'hA: alu_rn(A_RN, B_RM, AOP_SUBC, 1'b1);
			4'hB: alu_rn(A_RN, B_RM, AOP_SUBV, 1'b1);
			4'hC: alu_rn(A_RN, B_RM, AOP_ADD,  1'b0);
			4'hE: alu_rn(A_RN, B_RM, AOP_ADDC, 1'b1);
			4'hF: alu_rn(A_RN, B_RM, AOP_ADDV, 1'b1);
			default: legal_o = 1'b0;                     // 5, D: SH-2; 1, 9: unassigned
		endcase
		// ------------------------------------------------------------ 0100
		4'h4: begin
			if (ir_i[3:0] == 4'hF) begin                 // MAC.W @Rm+,@Rn+
				// Naming Rn and Rm here lets the core see the load-use
				// dependency on them.
				a_sel_o = A_RN; b_sel_o = B_RM;
				spc_o = SP_MACW; mac_op_o = MACOP_MACW;
				uses_mac_o = 1'b1; ex_slots_o = 2'd1;
			end else case (ir_i[7:0])
				8'h00: shift_rn(SH_SHLL,  1'b1);
				8'h01: shift_rn(SH_SHLR,  1'b1);
				8'h08: shift_rn(SH_SHLL2, 1'b0);
				8'h09: shift_rn(SH_SHLR2, 1'b0);
				8'h18: shift_rn(SH_SHLL8, 1'b0);
				8'h19: shift_rn(SH_SHLR8, 1'b0);
				8'h28: shift_rn(SH_SHLL16,1'b0);
				8'h29: shift_rn(SH_SHLR16,1'b0);
				8'h04: shift_rn(SH_ROTL,  1'b1);
				8'h05: shift_rn(SH_ROTR,  1'b1);
				8'h20: shift_rn(SH_SHAL,  1'b1);
				8'h21: shift_rn(SH_SHAR,  1'b1);
				8'h24: shift_rn(SH_ROTCL, 1'b1);
				8'h25: shift_rn(SH_ROTCR, 1'b1);
				8'h11: alu_t(A_RN, B_ZERO, AOP_CMPPZ);
				8'h15: alu_t(A_RN, B_ZERO, AOP_CMPPL);
				8'h10: legal_o = 1'b0;                   // DT: SH-2 only
				// STS.L MACH/MACL/PR,@-Rn. The MACH and MACL forms read the
				// multiplier in their MA stage, so they name which register
				// rather than carrying a value picked up in EX.
				8'h02: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_MACH;
				             mac_op_o = MACOP_LDH;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h12: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_MACL;
				             mac_op_o = MACOP_LDL;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h22: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_PR;
				             int_disable_o = 1'b1; end
				// STC.L SR/GBR/VBR,@-Rn: 4 stages, 2 states
				8'h03: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_SR;
				             ex_slots_o = 2'd1; int_disable_o = 1'b1; end
				8'h13: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_GBR;
				             ex_slots_o = 2'd1; int_disable_o = 1'b1; end
				8'h23: begin predec(SZ_L); ma_op_o = MA_STORE; sd_sel_o = D_VBR;
				             ex_slots_o = 2'd1; int_disable_o = 1'b1; end
				// LDS.L @Rm+,MACH/MACL/PR. The MACH and MACL forms hand the
				// loaded longword to the multiplier, so they carry its command
				// as well as the load target.
				8'h06: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_MACH;
				             mac_op_o = MACOP_LDH;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h16: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_MACL;
				             mac_op_o = MACOP_LDL;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h26: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_PR;
				             int_disable_o = 1'b1; end
				// LDC.L @Rm+,SR/GBR/VBR: the manual gives 3 states, the console
				// takes 4
				8'h07: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_SR;
				             spc_o = SP_LDCL; ex_slots_o = 2'd3; int_disable_o = 1'b1; end
				8'h17: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_GBR;
				             spc_o = SP_LDCL; ex_slots_o = 2'd3; int_disable_o = 1'b1; end
				8'h27: begin postinc(SZ_L, 1'b1); ma_op_o = MA_LOAD; ld_sel_o = W_VBR;
				             spc_o = SP_LDCL; ex_slots_o = 2'd3; int_disable_o = 1'b1; end
				// LDS Rm,MACH/MACL/PR
				8'h0A: begin a_sel_o = A_RN; ma_op_o = MA_MACWR; mac_op_o = MACOP_LDH;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h1A: begin a_sel_o = A_RN; ma_op_o = MA_MACWR; mac_op_o = MACOP_LDL;
				             uses_mac_o = 1'b1; int_disable_o = 1'b1; end
				8'h2A: begin a_sel_o = A_RN; alu_op_o = AOP_ADD; wr_sel_o = W_PR;
				             int_disable_o = 1'b1; end
				// LDC Rm,SR/GBR/VBR
				8'h0E: begin a_sel_o = A_RN; alu_op_o = AOP_ADD; wr_sel_o = W_SR;
				             int_disable_o = 1'b1; end
				8'h1E: begin a_sel_o = A_RN; alu_op_o = AOP_ADD; wr_sel_o = W_GBR;
				             int_disable_o = 1'b1; end
				8'h2E: begin a_sel_o = A_RN; alu_op_o = AOP_ADD; wr_sel_o = W_VBR;
				             int_disable_o = 1'b1; end
				8'h0B: begin br_op_o = BR_REG_L; ex_slots_o = 2'd1; end   // JSR @Rm
				8'h2B: begin br_op_o = BR_REG;   ex_slots_o = 2'd1; end   // JMP @Rm
				8'h1B: begin a_sel_o = A_RN; spc_o = SP_TAS; ex_slots_o = 2'd3;
				             ma_op_o = MA_LOAD; ma_sz_o = SZ_B; end
				default: legal_o = 1'b0;
			endcase
		end
		// ------------------------------------------------------------ 0101
		4'h5: begin load(A_RM, B_IMM, SZ_L, W_RN); imm_o = d4 << 2; end
		// ------------------------------------------------------------ 0110
		4'h6: case (ir_i[3:0])
			4'h0: load(A_RM, B_ZERO, SZ_B, W_RN);
			4'h1: load(A_RM, B_ZERO, SZ_W, W_RN);
			4'h2: load(A_RM, B_ZERO, SZ_L, W_RN);
			4'h3: alu_rn(A_RM, B_ZERO, AOP_ADD, 1'b0);   // MOV Rm,Rn
			4'h4: begin postinc(SZ_B, 1'b0); ma_op_o = MA_LOAD; ma_sz_o = SZ_B; ld_sel_o = W_RN; end
			4'h5: begin postinc(SZ_W, 1'b0); ma_op_o = MA_LOAD; ma_sz_o = SZ_W; ld_sel_o = W_RN; end
			4'h6: begin postinc(SZ_L, 1'b0); ma_op_o = MA_LOAD; ma_sz_o = SZ_L; ld_sel_o = W_RN; end
			4'h7: alu_rn(A_RM, B_ZERO, AOP_NOT,  1'b0);
			4'h8: alu_rn(A_RM, B_ZERO, AOP_SWAPB,1'b0);
			4'h9: alu_rn(A_RM, B_ZERO, AOP_SWAPW,1'b0);
			4'hA: alu_rn(A_ZERO, B_RM, AOP_SUBC, 1'b1);  // NEGC
			4'hB: alu_rn(A_ZERO, B_RM, AOP_SUB,  1'b0);  // NEG
			4'hC: alu_rn(A_RM, B_ZERO, AOP_EXTUB,1'b0);
			4'hD: alu_rn(A_RM, B_ZERO, AOP_EXTUW,1'b0);
			4'hE: alu_rn(A_RM, B_ZERO, AOP_EXTSB,1'b0);
			4'hF: alu_rn(A_RM, B_ZERO, AOP_EXTSW,1'b0);
			default: legal_o = 1'b0;
		endcase
		// ------------------------------------------------------------ 0111
		4'h7: begin alu_rn(A_RN, B_IMM, AOP_ADD, 1'b0); imm_o = i8s; end
		// ------------------------------------------------------------ 1000
		4'h8: case (nf)
			4'h0: begin store(A_RM, B_IMM, SZ_B, D_R0); imm_o = d4;      end
			4'h1: begin store(A_RM, B_IMM, SZ_W, D_R0); imm_o = d4 << 1; end
			4'h4: begin load (A_RM, B_IMM, SZ_B, W_R0); imm_o = d4;      end
			4'h5: begin load (A_RM, B_IMM, SZ_W, W_R0); imm_o = d4 << 1; end
			4'h8: begin alu_t(A_R0, B_IMM, AOP_CMPEQ);  imm_o = i8s;     end
			4'h9: begin br_op_o = BR_BT; imm_o = d8s2; end
			4'hB: begin br_op_o = BR_BF; imm_o = d8s2; end
			default: legal_o = 1'b0;                     // D, F: SH-2 BT/S, BF/S
		endcase
		// ------------------------------------------------------------ 1001
		4'h9: begin load(A_PC, B_IMM, SZ_W, W_RN); imm_o = d8 << 1; end
		// ------------------------------------------------------------ 1010
		4'hA: begin br_op_o = BR_DISP;   imm_o = d12s2; ex_slots_o = 2'd1; end
		4'hB: begin br_op_o = BR_DISP_L; imm_o = d12s2; ex_slots_o = 2'd1; end
		// ------------------------------------------------------------ 1100
		4'hC: case (nf)
			4'h0: begin store(A_GBR, B_IMM, SZ_B, D_R0); imm_o = d8;      end
			4'h1: begin store(A_GBR, B_IMM, SZ_W, D_R0); imm_o = d8 << 1; end
			4'h2: begin store(A_GBR, B_IMM, SZ_L, D_R0); imm_o = d8 << 2; end
			4'h3: begin spc_o = SP_TRAPA; imm_o = d8 << 2; end
			4'h4: begin load (A_GBR, B_IMM, SZ_B, W_R0); imm_o = d8;      end
			4'h5: begin load (A_GBR, B_IMM, SZ_W, W_R0); imm_o = d8 << 1; end
			4'h6: begin load (A_GBR, B_IMM, SZ_L, W_R0); imm_o = d8 << 2; end
			4'h7: begin alu_rn(A_PC4, B_IMM, AOP_ADD, 1'b0); wr_sel_o = W_R0;
			            imm_o = d8 << 2; end                            // MOVA
			4'h8: begin alu_t (A_R0, B_IMM, AOP_TST); imm_o = d8; end
			4'h9: begin alu_rn(A_R0, B_IMM, AOP_AND, 1'b0); wr_sel_o = W_R0; imm_o = d8; end
			4'hA: begin alu_rn(A_R0, B_IMM, AOP_XOR, 1'b0); wr_sel_o = W_R0; imm_o = d8; end
			4'hB: begin alu_rn(A_R0, B_IMM, AOP_OR,  1'b0); wr_sel_o = W_R0; imm_o = d8; end
			// #imm,@(R0,GBR): read, modify, write. 6 stages, 3 states.
			4'hC: begin spc_o = SP_LOGMEM; lop_o = LOP_TST; imm_o = d8;
			            a_sel_o = A_R0; b_sel_o = B_GBR; ex_slots_o = 2'd2;
			            ma_op_o = MA_LOAD; ma_sz_o = SZ_B; end
			4'hD: begin spc_o = SP_LOGMEM; lop_o = LOP_AND; imm_o = d8;
			            a_sel_o = A_R0; b_sel_o = B_GBR; ex_slots_o = 2'd2;
			            ma_op_o = MA_LOAD; ma_sz_o = SZ_B; end
			4'hE: begin spc_o = SP_LOGMEM; lop_o = LOP_XOR; imm_o = d8;
			            a_sel_o = A_R0; b_sel_o = B_GBR; ex_slots_o = 2'd2;
			            ma_op_o = MA_LOAD; ma_sz_o = SZ_B; end
			4'hF: begin spc_o = SP_LOGMEM; lop_o = LOP_OR;  imm_o = d8;
			            a_sel_o = A_R0; b_sel_o = B_GBR; ex_slots_o = 2'd2;
			            ma_op_o = MA_LOAD; ma_sz_o = SZ_B; end
			default: legal_o = 1'b0;
		endcase
		// ------------------------------------------------------------ 1101
		4'hD: begin load(A_PC4, B_IMM, SZ_L, W_RN); imm_o = d8 << 2; end
		// ------------------------------------------------------------ 1110
		4'hE: begin alu_rn(A_ZERO, B_IMM, AOP_PASSB, 1'b0); imm_o = i8s; end
		// ------------------------------------------------------------ 1111
		default: legal_o = 1'b0;                         // unassigned on SH-1 and SH-2
		endcase
	end
endmodule
