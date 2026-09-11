// Copyright (c) 2026 Jamie Blanks
//
// SH-1 multiplier and 42-bit multiply-accumulate register pair. Only the low
// ten bits of MACH are real, sign extended to 32 on read, so the accumulator
// is MACH[9:0]:MACL wrapping at bit 41. With SR.S set, MAC.W saturates MACL
// and sets MACH bit 0 as a sticky overflow flag. The multiplier stays busy
// three states after the last MA; a following multiplier instruction holds
// its MA until busy clears.

module sh1_mac (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        start_i,     // one state: carry out op_i
	input  wire [2:0]  op_i,
	input  wire [31:0] wdata_i,     // LDS source
	input  wire [15:0] mul_a_i,
	input  wire [15:0] mul_b_i,
	input  wire        s_i,         // SR.S

	output wire [31:0] mach_o,      // sign extended from bit 9, as read
	output wire [31:0] macl_o,
	output wire        busy_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "sh1_defs.svh"
	`include "ss_map.svh"

	reg  [9:0]  mach;
	reg  [31:0] macl;
	reg  [1:0]  cnt;                // mm states still to run
	reg         sat;                // this multiply saturates
	reg  [15:0] op_a, op_b;         // operands, registered on the start pulse
	reg  [31:0] prod;
	reg  [41:0] acc;
	reg         is_mac;       // MAC.W accumulates; MULS/MULU only load MACL

	assign mach_o = {{22{mach[9]}}, mach};
	assign macl_o = macl;
	assign busy_o = (cnt != 2'd0);

	// One signed 17x17 multiply serves both signed and unsigned 16-bit forms.
	// Its inputs are registered to keep the multiplier off the decode path;
	// the product is taken in the first mm state and read in the last.
	reg                unsigned_op;
	wire signed [16:0] ma_s = $signed({unsigned_op ? 1'b0 : op_a[15], op_a});
	wire signed [16:0] mb_s = $signed({unsigned_op ? 1'b0 : op_b[15], op_b});

	/* verilator lint_off UNUSEDSIGNAL */
	wire signed [33:0] mul_full = ma_s * mb_s;  // both forms fit in the low 32
	/* verilator lint_on UNUSEDSIGNAL */

	// --------------------------------------------------------- savestate
	// MACH is saved as its ten real bits so a restore round trips.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_MAC0, SS_MAC1, SS_MAC2;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1, ss_dout2;

	wire [63:0] SS_MAC0_BACK = {7'd0, acc, is_mac, unsigned_op, sat, cnt, mach};
	wire [63:0] SS_MAC1_BACK = {prod, macl};
	wire [63:0] SS_MAC2_BACK = {32'd0, op_b, op_a};

	ss_reg #(.ADDR (SSW_SH1_MAC + 0), .DEFAULT (64'd0)) u_ss0 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout0),
		.din_i      (SS_MAC0_BACK),
		.dout_o     (SS_MAC0)
	);
	ss_reg #(.ADDR (SSW_SH1_MAC + 1), .DEFAULT (64'd0)) u_ss1 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout1),
		.din_i      (SS_MAC1_BACK),
		.dout_o     (SS_MAC1)
	);
	ss_reg #(.ADDR (SSW_SH1_MAC + 2), .DEFAULT (64'd0)) u_ss2 (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout2),
		.din_i      (SS_MAC2_BACK),
		.dout_o     (SS_MAC2)
	);
	assign ss_dout = ss_dout0 | ss_dout1 | ss_dout2;

	wire [41:0] prod42 = {{10{prod[31]}}, prod};
	wire [41:0] sum42  = acc + prod42;
	wire [32:0] sum33  = {acc[31], acc[31:0]} + {prod[31], prod};
	wire        sat_ov = (sum33[32] != sum33[31]);

	always @(posedge clk_i) begin
		if (rst_i) begin
			mach <= SS_MAC0[9:0];
			cnt  <= SS_MAC0[11:10];
			sat  <= SS_MAC0[12];
			unsigned_op <= SS_MAC0[13];
			is_mac <= SS_MAC0[14];
			acc  <= SS_MAC0[56:15];
			macl <= SS_MAC1[31:0];
			prod <= SS_MAC1[63:32];
			op_a <= SS_MAC2[15:0];
			op_b <= SS_MAC2[31:16];
		end else if (ce_i) begin
			if (start_i) begin
				case (op_i)
				MACOP_CLR: begin mach <= 10'd0; macl <= 32'd0; end
				MACOP_LDH: mach <= wdata_i[9:0];
				MACOP_LDL: macl <= wdata_i;
				MACOP_MULS,
				MACOP_MULU,
				MACOP_MACW: begin
					op_a <= mul_a_i;
					op_b <= mul_b_i;
					unsigned_op <= (op_i == MACOP_MULU);
					acc  <= {mach, macl};
					sat  <= s_i && (op_i == MACOP_MACW);
					is_mac <= (op_i == MACOP_MACW);
					cnt  <= 2'd3;
				end
				default: ;
				endcase
			end else if (cnt != 2'd0) begin
				cnt <= cnt - 2'd1;
				if (cnt == 2'd3) prod <= mul_full[31:0];
				if (cnt == 2'd1) begin
					if (!is_mac) begin
						macl <= prod[31:0];         // MULS.W / MULU.W
					end else if (sat) begin
						if (sat_ov) begin
							macl <= sum33[32] ? 32'h80000000 : 32'h7FFFFFFF;
							mach <= mach | 10'd1;
						end else begin
							macl <= sum33[31:0];
						end
					end else begin
						mach <= sum42[41:32];
						macl <= sum42[31:0];
					end
				end
			end
		end
	end
endmodule
