// Copyright (c) 2026 Jamie Blanks
//
// SH-1 arithmetic and logic unit. T-bit rules follow the programming manual.
// CMP/STR compares the four real byte lanes (the manual's printed C model
// shifts by 12/8/4/0, which cannot). DIV1 Rn,Rn reads both operands before
// the shift, as a register file must. Shifts live in sh1_shifter.

module sh1_alu (
	input  wire [31:0] a_i,
	input  wire [31:0] b_i,
	input  wire [5:0]  op_i,
	input  wire        t_i,
	input  wire        m_i,
	input  wire        q_i,

	output reg  [31:0] res_o,
	output reg         t_o,
	output reg         m_o,
	output reg         q_o
);
	`include "sh1_defs.svh"

	wire [31:0] sum   = a_i + b_i;
	wire [32:0] sumc  = {1'b0, a_i} + {1'b0, b_i} + {32'd0, t_i};
	wire [31:0] dif   = a_i - b_i;
	wire [32:0] difc  = {1'b0, a_i} - {1'b0, b_i} - {32'd0, t_i};

	wire addv = (a_i[31] == b_i[31]) && (sum[31] != a_i[31]);
	wire subv = (a_i[31] != b_i[31]) && (dif[31] != a_i[31]);

	wire [31:0] xr  = a_i ^ b_i;
	wire cmpstr = (xr[31:24] == 8'd0) || (xr[23:16] == 8'd0)
	           || (xr[15:8]  == 8'd0) || (xr[7:0]   == 8'd0);

	wire sa = a_i[31];
	wire sb = b_i[31];
	wire ge_u = (a_i >= b_i);
	wire gt_u = (a_i >  b_i);
	// Signed compare from the unsigned one: differing signs decide it outright.
	wire ge_s = (sa == sb) ? ge_u : (sb & ~sa);
	wire gt_s = (sa == sb) ? gt_u : (sb & ~sa);

	// DIV1: one non-restoring division step. Subtract when old Q equals M,
	// add otherwise; new Q comes from the shifted-out bit and the carry or
	// borrow, inverted when M is set.
	wire        div_q1  = a_i[31];
	wire [31:0] div_sh  = {a_i[30:0], t_i};
	wire [32:0] div_sub = {1'b0, div_sh} - {1'b0, b_i};
	wire [32:0] div_add = {1'b0, div_sh} + {1'b0, b_i};
	wire        div_use_sub = (q_i == m_i);
	wire [31:0] div_res  = div_use_sub ? div_sub[31:0] : div_add[31:0];
	wire        div_cb   = div_use_sub ? div_sub[32] : div_add[32];
	wire        div_base = div_q1 ? ~div_cb : div_cb;
	wire        div_q    = m_i ? ~div_base : div_base;

	always @* begin
		res_o = 32'd0;
		t_o   = t_i;
		m_o   = m_i;
		q_o   = q_i;
		case (op_i)
		AOP_ADD:    res_o = sum;
		AOP_ADDC:   begin res_o = sumc[31:0]; t_o = sumc[32]; end
		AOP_ADDV:   begin res_o = sum;       t_o = addv;     end
		AOP_SUB:    res_o = dif;
		AOP_SUBC:   begin res_o = difc[31:0]; t_o = difc[32]; end
		AOP_SUBV:   begin res_o = dif;       t_o = subv;     end
		AOP_AND:    res_o = a_i & b_i;
		AOP_OR:     res_o = a_i | b_i;
		AOP_XOR:    res_o = xr;
		AOP_NOT:    res_o = ~a_i;
		AOP_CMPEQ:  t_o = (a_i == b_i);
		AOP_CMPHS:  t_o = ge_u;
		AOP_CMPGE:  t_o = ge_s;
		AOP_CMPHI:  t_o = gt_u;
		AOP_CMPGT:  t_o = gt_s;
		AOP_CMPPZ:  t_o = ~sa;
		AOP_CMPPL:  t_o = ~sa && (a_i != 32'd0);
		AOP_CMPSTR: t_o = cmpstr;
		AOP_TST:    t_o = ((a_i & b_i) == 32'd0);
		AOP_XTRCT:  res_o = {a_i[15:0], b_i[31:16]};
		AOP_SWAPB:  res_o = {a_i[31:16], a_i[7:0], a_i[15:8]};
		AOP_SWAPW:  res_o = {a_i[15:0], a_i[31:16]};
		AOP_EXTSB:  res_o = {{24{a_i[7]}},  a_i[7:0]};
		AOP_EXTSW:  res_o = {{16{a_i[15]}}, a_i[15:0]};
		AOP_EXTUB:  res_o = {24'd0, a_i[7:0]};
		AOP_EXTUW:  res_o = {16'd0, a_i[15:0]};
		AOP_DIV0S:  begin q_o = sa; m_o = sb; t_o = sa ^ sb; end
		AOP_DIV0U:  begin q_o = 1'b0; m_o = 1'b0; t_o = 1'b0; end
		AOP_DIV1:   begin res_o = div_res; q_o = div_q; t_o = (div_q == m_i); end
		AOP_PASSB:  res_o = b_i;
		AOP_MOVT:   res_o = {31'd0, t_i};
		AOP_SETT:   t_o = 1'b1;
		AOP_CLRT:   t_o = 1'b0;
		default:    res_o = sum;
		endcase
	end
endmodule
