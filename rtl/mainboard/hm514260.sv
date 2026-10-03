// Copyright (c) 2026 Jamie Blanks

// Hitachi HM514260, 256K x 16 fast-page-mode DRAM: the Loopy's 512 KB of work
// RAM. The board wires chip A8-A0 to CPU A9-A1, so with the SH7021's nine-bit
// row shift (DCR.MXC = 01) the row phase carries address bits 18-10 and the
// column phase bits 9-1. A CAS-before-RAS refresh is decoded and ignored.
// Contents live in the shared SDRAM through mem_line_cache.

module hm514260 (
	input  wire        clk_i,          // clk_sys, not a chip pin
	input  wire        rst_i,

	input  wire [8:0]  a_i,
	input  wire        ras_n_i,
	input  wire        cash_n_i,
	input  wire        casl_n_i,
	input  wire        we_n_i,
	input  wire        oe_n_i,
	input  wire [15:0] d_i,
	output wire [15:0] d_o,
	output wire        d_oe_o,

	output wire        stall_o,

	// Savestate flush: the cache holds bytes SDRAM has not seen yet.
	input  wire        ss_flush_i,
	output wire        ss_idle_o,

	output wire        mem_req_o,
	output wire        mem_we_o,
	output wire [18:3] mem_line_o,
	output wire [63:0] mem_din_o,
	output wire [7:0]  mem_be_o,
	input  wire [63:0] mem_dout_i,
	input  wire        mem_busy_i,
	input  wire        mem_done_i
);
	reg [8:0] row;
	reg       ras_n_q, cash_n_q, casl_n_q;
	reg       refresh;

	wire cas_n   = cash_n_i & casl_n_i;
	wire cas_n_q = cash_n_q & casl_n_q;

	// CAS before RAS is a refresh; the row latch belongs to the other order.
	wire ras_fall = ~ras_n_i &  ras_n_q;
	wire ras_rise =  ras_n_i & ~ras_n_q;
	wire cas_fall = ~cas_n   &  cas_n_q;

	always @(posedge clk_i) begin
		if (rst_i) begin
			row     <= 9'd0;
			ras_n_q <= 1'b1;
			cash_n_q <= 1'b1;
			casl_n_q <= 1'b1;
			refresh <= 1'b0;
		end else begin
			ras_n_q  <= ras_n_i;
			cash_n_q <= cash_n_i;
			casl_n_q <= casl_n_i;

			if (ras_fall) begin
				refresh <= ~cas_n;
				if (cas_n) row <= a_i;
			end else if (ras_rise) begin
				refresh <= 1'b0;
			end
		end
	end

	// `refresh` only knows about a CAS-before-RAS cycle one clock late, so the
	// cycle RAS falls is decided from the pins too; otherwise a refresh looks
	// like a read and the cache fetches it.
	wire cbr = ras_fall ? ~cas_n : refresh;

	wire acc = ~ras_n_i & ~cas_n & ~cbr;
	wire we  = ~we_n_i;

	// 32 KB of lines: 4 KB stalls the CPU about twice as often, 64 KB does not
	// fit the device beside everything else.
	mem_line_cache #(.ADDR_W (19), .INDEX_W (12), .READ_ONLY (1'b0)) u_cache (
		.clk_i      (clk_i),
		.rst_i      (rst_i),
		.addr_i     ({row, a_i}),
		.acc_i      (acc),
		.we_i       (we),
		.be_i       ({~cash_n_i, ~casl_n_i}),
		.wdata_i    (d_i),
		.commit_i   (cas_fall & acc & we),
		.sample_i   (1'b0),
		.rdata_o    (d_o),
		.stall_o    (stall_o),
		.ss_flush_i (ss_flush_i),
		.ss_idle_o  (ss_idle_o),
		.mem_req_o  (mem_req_o),
		.mem_we_o   (mem_we_o),
		.mem_line_o (mem_line_o),
		.mem_din_o  (mem_din_o),
		.mem_be_o   (mem_be_o),
		.mem_dout_i (mem_dout_i),
		.mem_busy_i (mem_busy_i),
		.mem_done_i (mem_done_i)
	);

	assign d_oe_o = acc & ~we & ~oe_n_i;
endmodule
