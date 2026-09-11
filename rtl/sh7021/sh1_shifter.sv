// Copyright (c) 2026 Jamie Blanks
//
// SH-1 shifter and rotator. SHAL is the same operation as SHLL; the
// multi-bit forms leave T alone.

module sh1_shifter (
	input  wire [31:0] a_i,
	input  wire [3:0]  op_i,
	input  wire        t_i,
	output reg  [31:0] res_o,
	output reg         t_o
);
	`include "sh1_defs.svh"

	always @* begin
		res_o = a_i;
		t_o   = t_i;
		case (op_i)
		SH_SHLL,
		SH_SHAL:   begin res_o = {a_i[30:0], 1'b0};  t_o = a_i[31]; end
		SH_SHLR:   begin res_o = {1'b0, a_i[31:1]};  t_o = a_i[0];  end
		SH_SHAR:   begin res_o = {a_i[31], a_i[31:1]}; t_o = a_i[0]; end
		SH_ROTL:   begin res_o = {a_i[30:0], a_i[31]}; t_o = a_i[31]; end
		SH_ROTR:   begin res_o = {a_i[0], a_i[31:1]};  t_o = a_i[0];  end
		SH_ROTCL:  begin res_o = {a_i[30:0], t_i};     t_o = a_i[31]; end
		SH_ROTCR:  begin res_o = {t_i, a_i[31:1]};     t_o = a_i[0];  end
		SH_SHLL2:  res_o = {a_i[29:0], 2'd0};
		SH_SHLR2:  res_o = {2'd0, a_i[31:2]};
		SH_SHLL8:  res_o = {a_i[23:0], 8'd0};
		SH_SHLR8:  res_o = {8'd0, a_i[31:8]};
		SH_SHLL16: res_o = {a_i[15:0], 16'd0};
		SH_SHLR16: res_o = {16'd0, a_i[31:16]};
		default:   ;
		endcase
	end
endmodule
