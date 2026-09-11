// Copyright (c) 2026 Jamie Blanks
//
// Internal bus master mux. The DMAC outranks the CPU whenever it wants the
// bus; the refresh controller sits inside the BSC and takes it from both.
// The loser simply does not see its acknowledge and stalls where it is.

module sh7021_ibus (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,
	// the bus controller is mid-cycle, or the CPU is holding it for a TAS
	input  wire        hold_i,
	input  wire        lock_i,

	input  wire        cpu_req_i,
	input  wire [31:0] cpu_addr_i,
	input  wire        cpu_we_i,
	input  wire [1:0]  cpu_sz_i,
	input  wire [31:0] cpu_wdata_i,
	input  wire        cpu_ifetch_i,
	output wire [31:0] cpu_rdata_o,
	output wire        cpu_ack_o,

	input  wire        dm_breq_i,
	input  wire        dm_req_i,
	input  wire [31:0] dm_addr_i,
	input  wire        dm_we_i,
	input  wire [1:0]  dm_sz_i,
	input  wire [31:0] dm_wdata_i,
	output wire [31:0] dm_rdata_o,
	output wire        dm_ack_o,
	output wire        dm_gnt_o,

	output wire        req_o,
	output wire [31:0] addr_o,
	output wire        we_o,
	output wire [1:0]  sz_o,
	output wire [31:0] wdata_o,
	output wire        ifetch_o,
	input  wire [31:0] rdata_i,
	input  wire        ack_i,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	`include "ss_map.svh"

	// The grant only moves while nothing is in flight, so a transfer that a
	// width conversion split into several accesses keeps its master.
	reg gnt;

	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_IBUS;
	/* verilator lint_on UNUSEDSIGNAL */
	ss_reg #(.ADDR (SSW_IBUS_BASE), .DEFAULT (64'd0)) u_ss (
		.clk_i      (clk_i),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({63'd0, gnt}),
		.dout_o     (SS_IBUS)
	);

	// A TAS holds the bus only once it has it. While the DMAC still owns the
	// grant the lock must let it fall back, or the CPU deadlocks waiting.
	wire hold_gnt = hold_i || (lock_i && !gnt);

	always @(posedge clk_i) begin
		if (rst_i)                  gnt <= SS_IBUS[0];
		else if (ce_i && !hold_gnt) gnt <= dm_breq_i;
	end

	assign dm_gnt_o = gnt;

	assign req_o    = gnt ? dm_req_i    : cpu_req_i;
	assign addr_o   = gnt ? dm_addr_i   : cpu_addr_i;
	assign we_o     = gnt ? dm_we_i     : cpu_we_i;
	assign sz_o     = gnt ? dm_sz_i     : cpu_sz_i;
	assign wdata_o  = gnt ? dm_wdata_i  : cpu_wdata_i;
	assign ifetch_o = gnt ? 1'b0        : cpu_ifetch_i;

	assign cpu_rdata_o = rdata_i;
	assign dm_rdata_o  = rdata_i;
	assign cpu_ack_o   = ack_i && !gnt;
	assign dm_ack_o    = ack_i &&  gnt;
endmodule
