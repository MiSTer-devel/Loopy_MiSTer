// Copyright (c) 2026 Jamie Blanks

// The cartridge's battery-backed SRAM, on /SRAM (the SH7021's /CS2) at
// 0x02000000: 8 bits wide on D7-D0, three states per access, A16-A0. The
// header's SRAM range gives the size and addresses wrap modulo it.
//
// Pins modelled: A16-A0, /CS, /RD, /WR, D7-D0. Battery backup is the MiSTer
// save slot: port B carries the same block RAM out to `save_slot` and to the
// savestate bulk stream.
//
// The chip is byte wide to the CPU but the block RAM is 16 bits with byte
// enables, because both port B users are 16 bits wide. Byte 2k is the low
// half of word k, so a .sav file's byte N is SRAM byte N.
//
// Data is latched on the trailing edge of /WR, like a real SRAM, so a write
// the CPU abandons mid-cycle leaves nothing behind.

module cart_sram #(
	parameter int ADDR_W = 15                 // 32 KB, the largest retail size
) (
	input  wire              clk_i,
	input  wire              rst_i,

	input  wire [16:0]       a_i,
	input  wire              cs_n_i,          // /SRAM, the CPU's /CS2
	input  wire              rd_n_i,
	input  wire              wr_n_i,
	input  wire [7:0]        d_i,
	output wire [7:0]        d_o,
	output wire              d_oe_o,

	input  wire [16:0]       size_mask_i,     // size minus one, from the header
	output reg               dirty_o,         // one cycle per write

	// Port B: save slot and savestate streaming, 16 bits with byte enables.
	input  wire [ADDR_W-2:0] b_addr_i,
	input  wire              b_wren_i,
	input  wire [1:0]        b_be_i,
	input  wire [15:0]       b_wdata_i,
	output wire [15:0]       b_q_o
);
	// A cartridge with less SRAM than the 128 KB the pins can reach wraps, and
	// the block RAM behind it is only as big as the largest part fitted.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [16:0]       wrapped = a_i & size_mask_i;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [ADDR_W-1:0] addr    = wrapped[ADDR_W-1:0];

	reg [ADDR_W-1:0] addr_q;
	reg [7:0]        data_q;
	reg              wr_n_q, cs_n_q;

	wire wr_edge = wr_n_i & ~wr_n_q & ~cs_n_q;

	// The address the RAM sees this cycle, and the half of the word its output
	// will hold next cycle.
	wire [ADDR_W-1:0] a_applied = wr_edge ? addr_q : addr;
	reg               rd_hi_q;

	always @(posedge clk_i) begin
		rd_hi_q <= a_applied[0];
		if (rst_i) begin
			wr_n_q  <= 1'b1;
			cs_n_q  <= 1'b1;
			dirty_o <= 1'b0;
		end else begin
			addr_q  <= addr;
			data_q  <= d_i;
			wr_n_q  <= wr_n_i;
			cs_n_q  <= cs_n_i;
			dirty_o <= wr_edge;
		end
	end

	wire [15:0] q_a;
	assign d_o = rd_hi_q ? q_a[15:8] : q_a[7:0];

	cache_ram_dp_be #(.ADDR_WIDTH (ADDR_W-1), .DATA_WIDTH (16)) u_ram (
		.clk_i     (clk_i),
		.addr_a_i  (a_applied[ADDR_W-1:1]),
		.wren_a_i  (wr_edge),
		.be_a_i    (a_applied[0] ? 2'b10 : 2'b01),
		.wdata_a_i ({data_q, data_q}),
		.q_a_o     (q_a),
		.addr_b_i  (b_addr_i),
		.wren_b_i  (b_wren_i),
		.be_b_i    (b_be_i),
		.wdata_b_i (b_wdata_i),
		.q_b_o     (b_q_o)
	);

	assign d_oe_o = ~cs_n_i & ~rd_n_i;
endmodule
