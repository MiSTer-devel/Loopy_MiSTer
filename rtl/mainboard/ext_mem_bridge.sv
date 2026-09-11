// Everything the Loopy keeps in external memory, sharing one SDRAM: the
// 512 KB work DRAM, the cartridge mask ROM and the CDT109 wave ROM.
//
//   0x0000000  cartridge ROM     up to 4 MB   bank 0
//   0x1000000  work DRAM         512 KB      bank 1
//   0x2000000  synth wave ROM    512 KB      bank 2
//
// One SDRAM bank per client (BA = addr[25:24]) so the open-page controller
// keeps the cartridge and work DRAM rows open across code/data switches.
// Clients are on clk_sys and the controller on clk_ram, four times faster off
// the same PLL, so each port crosses through `sync_xfer`. Every port moves
// whole eight-byte lines; the loader's 16-bit words ride in the lane their
// address selects. Priority is the controller's: p0 work DRAM (the CPU stalls
// on it), p1 cartridge, p2 wave ROM, which reads ahead. The loader borrows p0.
//
// Client contract, all on clk_sys: pulse req for one cycle while busy is low;
// busy stays high until the transfer finishes; done pulses once with dout
// valid.

module ext_mem_bridge
(
	input  wire clk_sys,
	input  wire reset_sys,
	input  wire clk_ram,
	input  wire reset_ram,

	// Work DRAM lines, read and write.
	input  wire        dram_req,
	input  wire        dram_we,
	input  wire [18:3] dram_line,
	input  wire [63:0] dram_din,
	input  wire [7:0]  dram_be,
	output wire [63:0] dram_dout,
	output wire        dram_busy,
	output wire        dram_done,

	// Cartridge ROM lines, read only.
	input  wire        rom_req,
	input  wire [21:3] rom_line,
	output wire [63:0] rom_dout,
	output wire        rom_busy,
	output wire        rom_done,

	// Synth wave ROM lines, read only.
	input  wire        wave_req,
	input  wire [18:3] wave_line,
	output wire [63:0] wave_dout,
	output wire        wave_busy,
	output wire        wave_done,

	// Savestate stream. Owns p0 while ss_own is high, which the pause
	// handshake only raises with the machine stopped and the line cache
	// flushed, so the work DRAM in SDRAM is what the CPU would have seen.
	input  wire        ss_own,
	input  wire        ss_req,
	input  wire        ss_we,
	input  wire [18:3] ss_line,
	input  wire [63:0] ss_din,
	input  wire [7:0]  ss_be,
	output wire [63:0] ss_dout,
	output wire        ss_done,

	// Content loader. Owns p0 while loading is high.
	input  wire        loading,
	input  wire        ld_req,
	// Full SDRAM byte address. Bit 0 is always zero: the loader moves words.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [25:0] ld_addr,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire [15:0] ld_din,
	output wire        ld_busy,
	output wire        ld_done,

	// To the SDRAM controller, on clk_ram.
	output wire        p0_req,
	output wire        p0_we,
	output wire [25:0] p0_addr,
	output wire [63:0] p0_din,
	output wire [7:0]  p0_byte_en,
	input  wire [63:0] p0_dout,
	input  wire        p0_busy,
	input  wire        p0_ready,

	output wire        p1_req,
	output wire [25:0] p1_addr,
	input  wire [63:0] p1_dout,
	input  wire        p1_busy,
	input  wire        p1_ready,

	output wire        p2_req,
	output wire [25:0] p2_addr,
	input  wire [63:0] p2_dout,
	input  wire        p2_busy,
	input  wire        p2_ready
);

	localparam logic [25:0] CART_BASE = 26'h000000;
	localparam logic [25:0] DRAM_BASE = 26'h1000000;
	localparam logic [25:0] WAVE_BASE = 26'h2000000;

	// ---- p0: work DRAM lines, or the loader while it is running ----------

	// A loader word sits in the lane its address picks out of the eight-byte
	// group; the controller masks the low address bits off for a 64-bit port.
	wire [1:0]  ld_lane = ld_addr[2:1];
	wire [63:0] ld_data = {48'd0, ld_din} << {ld_lane, 4'd0};
	wire [7:0]  ld_be   = 8'd3 << {ld_lane, 1'b0};

	// The loader and the savestate cross by handshake, one at a time; neither
	// is waited on. The work DRAM demand path goes direct, see below.
	wire        p0_src_req  = loading ? ld_req  : ss_req & ss_own;
	wire        p0_src_we   = loading ? 1'b1    : ss_we;
	wire [25:0] p0_src_addr = loading ? {ld_addr[25:3], 3'd0}
	                                  : (DRAM_BASE | {7'd0, ss_line, 3'd0});
	wire [63:0] p0_src_din  = loading ? ld_data : ss_din;
	wire [7:0]  p0_src_be   = loading ? ld_be   : ss_be;

	wire        p0_src_busy, p0_src_done;
	wire [63:0] p0_src_resp;

	wire        p0_dst_valid, p0_dst_ack;
	wire [98:0] p0_dst_data;

	sync_xfer #(.WIDTH(99), .RESP_WIDTH(64)) xfer_p0
	(
		.src_clk(clk_sys), .src_reset(reset_sys),
		.src_req(p0_src_req),
		.src_data({p0_src_we, p0_src_addr, p0_src_din, p0_src_be}),
		.src_busy(p0_src_busy), .src_done(p0_src_done), .src_resp(p0_src_resp),
		.dst_clk(clk_ram), .dst_reset(reset_ram),
		.dst_valid(p0_dst_valid), .dst_data(p0_dst_data),
		.dst_ack(p0_dst_ack), .dst_resp(p0_dout)
	);

	// ---- p1: cartridge ROM lines ------------------------------------------

	wire        p1_src_busy, p1_src_done;
	wire [63:0] p1_src_resp;
	wire        p1_dst_valid, p1_dst_ack;
	wire [25:0] p1_dst_data;

	sync_xfer #(.WIDTH(26), .RESP_WIDTH(64)) xfer_p1
	(
		.src_clk(clk_sys), .src_reset(reset_sys),
		.src_req(rom_req & ~loading),
		.src_data(CART_BASE | {4'd0, rom_line, 3'd0}),
		.src_busy(p1_src_busy), .src_done(p1_src_done), .src_resp(p1_src_resp),
		.dst_clk(clk_ram), .dst_reset(reset_ram),
		.dst_valid(p1_dst_valid), .dst_data(p1_dst_data),
		.dst_ack(p1_dst_ack), .dst_resp(p1_dout)
	);

	// ---- p2: wave ROM lines ------------------------------------------------

	wire        p2_src_busy, p2_src_done;
	wire [63:0] p2_src_resp;
	wire        p2_dst_valid, p2_dst_ack;
	wire [25:0] p2_dst_data;

	sync_xfer #(.WIDTH(26), .RESP_WIDTH(64)) xfer_p2
	(
		.src_clk(clk_sys), .src_reset(reset_sys),
		.src_req(wave_req & ~loading),
		.src_data(WAVE_BASE | {7'd0, wave_line, 3'd0}),
		.src_busy(p2_src_busy), .src_done(p2_src_done), .src_resp(p2_src_resp),
		.dst_clk(clk_ram), .dst_reset(reset_ram),
		.dst_valid(p2_dst_valid), .dst_data(p2_dst_data),
		.dst_ack(p2_dst_ack), .dst_resp(p2_dout)
	);

	// ---- clk_ram side: one transfer at a time per port --------------------

	// The controller takes a request on the edge where req is high and busy is
	// low, then raises ready when the data is published. in_flight keeps req
	// from being presented twice for the same transfer. The request lines are
	// registered because they reach the SDRAM_A pins through the controller's
	// arbitration, the longest path in the clk_ram domain.
	reg p0_in_flight, p1_in_flight, p2_in_flight;
	reg p0_req_q, p1_req_q, p2_req_q;

	// The work DRAM's demand path, clk_ram native. The cache's one-clk_sys
	// request pulse is four clk_ram cycles wide, so it is caught on its rising
	// edge and the answer goes back as a toggle the clk_sys side edge-detects.
	//
	//   dram_req (clk_sys) ‾‾‾‾\____          one pulse, four clk_ram wide
	//   d_valid  (clk_ram)   ___/‾‾‾‾‾‾‾‾\__  held until the controller answers
	//   d_done_tgl           ___________/‾‾‾  flips when d_dout is valid
	//
	// `direct` is registered on clk_ram because it reaches the controller's
	// address pins.
	reg         direct;
	reg         dram_req_q;
	reg         d_valid;
	reg         d_we;
	reg  [25:0] d_addr;
	reg  [63:0] d_din;
	reg  [7:0]  d_be;
	reg  [63:0] d_dout;
	reg         d_done_tgl;

	always @(posedge clk_ram) begin
		if (reset_ram) begin
			direct     <= 1'b0;
			dram_req_q <= 1'b0;
			d_valid    <= 1'b0;
			d_done_tgl <= 1'b0;
		end else begin
			direct     <= ~loading & ~ss_own;
			dram_req_q <= dram_req;
			if (dram_req && !dram_req_q && direct) begin
				d_valid <= 1'b1;
				d_we    <= dram_we;
				d_addr  <= DRAM_BASE | {7'd0, dram_line, 3'd0};
				d_din   <= dram_din;
				d_be    <= dram_be;
			end else if (d_valid && p0_in_flight && p0_ready) begin
				d_valid    <= 1'b0;
				d_dout     <= p0_dout;
				d_done_tgl <= ~d_done_tgl;
			end
		end
	end

	assign p0_req = p0_req_q;
	assign p1_req = p1_req_q;
	assign p2_req = p2_req_q;

	assign p0_we      = direct ? d_we   : p0_dst_data[98];
	assign p0_addr    = direct ? d_addr : p0_dst_data[97:72];
	assign p0_din     = direct ? d_din  : p0_dst_data[71:8];
	assign p0_byte_en = direct ? d_be   : p0_dst_data[7:0];
	assign p1_addr    = p1_dst_data;
	assign p2_addr    = p2_dst_data;

	assign p0_dst_ack = p0_in_flight & p0_ready;
	assign p1_dst_ack = p1_in_flight & p1_ready;
	assign p2_dst_ack = p2_in_flight & p2_ready;

	always @(posedge clk_ram) begin
		if (reset_ram) begin
			p0_in_flight <= 1'b0;
			p1_in_flight <= 1'b0;
			p2_in_flight <= 1'b0;
			p0_req_q     <= 1'b0;
			p1_req_q     <= 1'b0;
			p2_req_q     <= 1'b0;
		end else begin
			p0_req_q <= (direct ? d_valid : p0_dst_valid) & ~p0_in_flight;
			p1_req_q <= p1_dst_valid & ~p1_in_flight;
			p2_req_q <= p2_dst_valid & ~p2_in_flight;

			if (p0_req && !p0_busy)      p0_in_flight <= 1'b1;
			else if (p0_in_flight && p0_ready) p0_in_flight <= 1'b0;

			if (p1_req && !p1_busy)      p1_in_flight <= 1'b1;
			else if (p1_in_flight && p1_ready) p1_in_flight <= 1'b0;

			if (p2_req && !p2_busy)      p2_in_flight <= 1'b1;
			else if (p2_in_flight && p2_ready) p2_in_flight <= 1'b0;
		end
	end

	// ---- clk_sys side: client-facing wiring -------------------------------

	// The toggle crosses to clk_sys and becomes the one-cycle done the line
	// cache expects; d_dout is a clk_ram register that holds until the next
	// transfer finishes, so it is stable when done is seen.
	reg d_done_seen;
	always @(posedge clk_sys) begin
		if (reset_sys) d_done_seen <= 1'b0;
		else           d_done_seen <= d_done_tgl;
	end

	assign dram_dout = d_dout;
	assign dram_busy = d_valid | loading | ss_own;
	assign dram_done = d_done_tgl ^ d_done_seen;

	assign ld_busy = p0_src_busy;
	assign ld_done = p0_src_done & loading;

	assign ss_dout = p0_src_resp;
	assign ss_done = p0_src_done & ss_own & ~loading;

	assign rom_dout = p1_src_resp;
	assign rom_busy = p1_src_busy | loading;
	assign rom_done = p1_src_done;

	assign wave_dout = p2_src_resp;
	assign wave_busy = p2_src_busy | loading;
	assign wave_done = p2_src_done;

endmodule
