// Copyright (c) 2026 Jamie Blanks

// The MSM6653A's speech ROM, in block RAM.
//
// The real chip has 544 Kbit of undumped mask ROM on the die; this holds a
// replacement phrase bank built into the core. The image is zero padded to
// the full depth so the size can be a constant: a block walk that runs past a
// shorter image lands in the padding and stops on the first zero count byte.
//
//   present addr ──► M10K ──► q one clock later ──► data
//                              ready, once the address has stood still
//
//
// The chip may move the address while `req` stays high, so `ready` is
// qualified by the address the answer belongs to.

module cart_exp_bank_rom #(
	parameter     MEM_INIT_FILE = "rtl/cart/wanwan_bank.mif",   // Quartus
	parameter     SIM_INIT_FILE = "rtl/cart/wanwan_bank.hex",   // simulation
	parameter int ADDR_W        = 16
) (
	input  logic        clk_sys,

	input  logic        req,
	// The mapper's addresses are bounded by `size`, so the top bits are always
	// zero here.
	/* verilator lint_off UNUSEDSIGNAL */
	input  logic [23:0] addr,
	/* verilator lint_on UNUSEDSIGNAL */
	output logic [7:0]  data,
	output logic        ready,

	// What the bank mapper takes as its bound on a malformed phrase.
	output logic [23:0] size
);

	assign size = 24'(1 << ADDR_W);

	logic [ADDR_W-1:0] addr_q;
	logic              req_q;

	always_ff @(posedge clk_sys) begin
		addr_q <= addr[ADDR_W-1:0];
		req_q  <= req;
	end

	cache_ram #(
		.ADDR_WIDTH    (ADDR_W),
		.DATA_WIDTH    (8),
		.MEM_INIT_FILE (MEM_INIT_FILE),
		.SIM_INIT_FILE (SIM_INIT_FILE)
	) u_rom (
		.clk_i   (clk_sys),
		.addr_i  (addr[ADDR_W-1:0]),
		.wren_i  (1'b0),
		.wdata_i (8'd0),
		.q_o     (data)
	);

	assign ready = req & req_q & (addr_q == addr[ADDR_W-1:0]);

endmodule
