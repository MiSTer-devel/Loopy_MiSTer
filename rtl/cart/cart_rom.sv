// Copyright (c) 2026 Jamie Blanks

// The cartridge mask ROM, on /ROM (the SH7021's /CS6) at 0x0E000000: 16 bits
// wide, three states per read plus the wait states the BIOS programs into
// WCR3. Pins modelled: A21-A1, /CS, /RD, D15-D0; the CPU always reads whole
// words from it.
//
// The image lives in SDRAM behind mem_line_cache. A cartridge smaller than
// the 4 MB area repeats through it (`mask_i`).
//
// Dual-chip boards (Z544-1: IC104 16 Mbit then IC105 8 Mbit) use A21 as the
// second chip's select, and IC105 repeats inside its half:
//
//   0x000000  IC104, 2 MB
//   0x200000  IC105, 1 MB
//   0x300000  IC105 again, A20 going nowhere
//
// `split_i` turns that decode on and `hi_mask_i` is the second chip's size.

module cart_rom (
	input  wire        clk_i,
	input  wire        rst_i,

	input  wire [21:1] a_i,
	input  wire        cs_n_i,        // /ROM, the CPU's /CS6
	input  wire        rd_n_i,
	output wire [15:0] d_o,
	output wire        d_oe_o,

	input  wire [21:1] mask_i,        // image size rounded up, minus one
	input  wire        split_i,       // two mask ROMs, A21 picks between them
	input  wire [20:1] hi_mask_i,     // second chip's size, minus one

	output wire        stall_o,

	output wire        mem_req_o,
	output wire [21:3] mem_line_o,
	input  wire [63:0] mem_dout_i,
	input  wire        mem_busy_i,
	input  wire        mem_done_i
);
	wire acc = ~cs_n_i & ~rd_n_i;

	// The second chip sits above 2 MB in the image, so its half of the area
	// keeps bit 21 and masks the rest down to the chip.
	wire [21:1] a_eff = (split_i & a_i[21]) ? {1'b1, a_i[20:1] & hi_mask_i}
	                                        : (a_i & mask_i);

	mem_line_cache #(.ADDR_W (22), .INDEX_W (9), .READ_ONLY (1'b1)) u_cache (
		.clk_i      (clk_i),
		.rst_i      (rst_i),
		.addr_i     (a_eff),
		.acc_i      (acc),
		.we_i       (1'b0),
		.be_i       (2'b11),
		.wdata_i    (16'd0),
		.commit_i   (1'b0),
		.rdata_o    (d_o),
		.stall_o    (stall_o),
		.ss_flush_i (1'b0),
		/* verilator lint_off PINCONNECTEMPTY */
		.ss_idle_o  (),
		.mem_we_o   (),
		.mem_din_o  (),
		.mem_be_o   (),
		/* verilator lint_on PINCONNECTEMPTY */
		.mem_req_o  (mem_req_o),
		.mem_line_o (mem_line_o),
		.mem_dout_i (mem_dout_i),
		.mem_busy_i (mem_busy_i),
		.mem_done_i (mem_done_i)
	);

	assign d_oe_o = acc;
endmodule
