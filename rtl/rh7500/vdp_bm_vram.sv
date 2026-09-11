// Bitmap VRAM: 128 KB, the two frame-buffer video DRAMs on the board.
//
// The CPU sits on one port of a true dual-port block RAM and the render fetch
// on the other, and the DRAMs' flash-write cycle (a write into the fill
// trigger range paints a whole 256-byte row) is a short state machine on the
// CPU port. Organised 16384 x 64 bits, big-endian: byte 0 of a word is bits
// 63-56. The width keeps a row fill to 32 words and gives a bitmap row fetch
// eight 8bpp pixels per read.
//
// An M10K returns undefined data when the render port reads the word the CPU
// port is writing; the real part returns the new data, so the write is
// forwarded onto the read port byte by byte.
//
// The flash write is a read-modify-write because the mask keeps bits of the
// old value. It is pipelined one deep: read word i, write word i-1. The CPU
// waits on any bitmap access or further fill trigger while it runs.

module vdp_bm_vram
(
	input  wire clk,
	input  wire reset,

	// CPU port. cpu_addr is a halfword address; the byte lanes pick the half.
	input  wire        cpu_sel,
	input  wire        cpu_wr,
	input  wire [16:1] cpu_addr,
	input  wire [1:0]  cpu_be,
	input  wire [15:0] cpu_wdata,
	output wire [15:0] cpu_rdata,

	// Flash write.
	input  wire        fill_trig,
	input  wire [8:0]  fill_row,
	input  wire [7:0]  fill_mask,
	input  wire [7:0]  fill_value,
	output wire        fill_busy,

	// Render fetch port, addressed in 64-bit words.
	input  wire [13:0] rd_addr,
	output wire [63:0] rd_data,

	// Savestate walk, takes port A ahead of everything else.
	input  wire        ss_sel,
	input  wire [16:0] ss_addr,
	input  wire        ss_we,
	input  wire [7:0]  ss_wdata,
	output wire [7:0]  ss_rdata
);


	// ---- flash write sequencer ----------------------------------------------
	// A masked fill is a read then a write per word; a mask of all ones needs
	// no read and runs one cycle a word.

	reg       filling;
	reg       fill_wr;      // 0 = the read cycle, 1 = the write cycle
	reg [4:0] fill_i;
	reg [8:0] fill_y;
	reg [7:0] fill_m, fill_v;

	wire fill_fast = (fill_m == 8'hFF);

	assign fill_busy = filling | fill_trig;

	always @(posedge clk) begin
		if (reset) begin
			filling <= 1'b0;
			fill_wr <= 1'b0;
			fill_i  <= 5'd0;
			fill_y  <= 9'd0;
			fill_m  <= 8'd0;
			fill_v  <= 8'd0;
		end else if (!filling) begin
			if (fill_trig) begin
				filling <= 1'b1;
				fill_i  <= 5'd0;
				fill_y  <= fill_row;
				fill_m  <= fill_mask;
				fill_v  <= fill_value;
				fill_wr <= (fill_mask == 8'hFF);
			end
		end else if (fill_wr) begin
			fill_wr <= fill_fast;
			if (fill_i == 5'd31) begin
				filling <= 1'b0;
				fill_wr <= 1'b0;
			end else begin
				fill_i <= fill_i + 5'd1;
			end
		end else begin
			fill_wr <= 1'b1;
		end
	end

	// ---- port A: the CPU, or the fill while it runs --------------------------

	// Halfword lane within the 64-bit word, big-endian: halfword 0 is bits 63-48.
	wire [2:0] hw  = {1'b0, cpu_addr[2:1]};
	wire [2:0] hwr = 3'd3 - hw;
	wire [7:0] cpu_be64 = {6'd0, cpu_be} << (hwr * 3'd2);
	wire [63:0] cpu_wdata64 = {4{cpu_wdata}};

	wire [13:0] fill_addr = {fill_y, fill_i};

	// Byte 0 is the top byte, so the byte enable counts down from bit 7.
	wire [7:0] ss_be = 8'h80 >> ss_addr[2:0];

	wire [13:0] a_addr = ss_sel ? ss_addr[16:3] : filling ? fill_addr : cpu_addr[16:3];
	wire        a_wren = ss_sel ? ss_we : filling ? fill_wr : (cpu_sel & cpu_wr);
	wire [7:0]  a_be   = ss_sel ? ss_be : filling ? 8'hFF : cpu_be64;

	// The word being masked is the port's own output from the previous cycle's read.
	wire [63:0] a_q;

	reg [2:0] ss_lane_q;
	always @(posedge clk) ss_lane_q <= 3'd7 - ss_addr[2:0];
	assign ss_rdata = a_q[ss_lane_q * 6'd8 +: 8];

	wire [7:0]  fill_set = fill_v & fill_m;
	wire [63:0] fill_wdata;
	genvar b;
	generate
		for (b = 0; b < 8; b = b + 1) begin : g_fill
			assign fill_wdata[b*8 +: 8] = (a_q[b*8 +: 8] & ~fill_m) | fill_set;
		end
	endgenerate

	wire [63:0] a_wdata = ss_sel ? {8{ss_wdata}} : filling ? fill_wdata : cpu_wdata64;

	assign cpu_rdata = a_q[hwr * 6'd16 +: 16];

	// Forward a write onto the read port when both land on the same word, byte
	// by byte. The compare is registered because the RAM output lags its
	// address by one cycle.
	wire [63:0] rd_raw;
	reg         fwd_hit;
	reg [7:0]   fwd_be;
	reg [63:0]  fwd_data;

	always @(posedge clk) begin
		if (reset) begin
			fwd_hit <= 1'b0;
		end else begin
			fwd_hit  <= a_wren & (a_addr == rd_addr);
			fwd_be   <= a_be;
			fwd_data <= a_wdata;
		end
	end

	genvar f;
	generate
		for (f = 0; f < 8; f = f + 1) begin : g_fwd
			assign rd_data[f*8 +: 8] = (fwd_hit & fwd_be[f])
				? fwd_data[f*8 +: 8]
				: rd_raw[f*8 +: 8];
		end
	endgenerate

	cache_ram_dp_be #(
		.ADDR_WIDTH (14),
		.DATA_WIDTH (64)
	) u_ram (
		.clk_i     (clk),
		.addr_a_i  (a_addr),
		.wren_a_i  (a_wren),
		.be_a_i    (a_be),
		.wdata_a_i (a_wdata),
		.q_a_o     (a_q),
		.addr_b_i  (rd_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (8'h00),
		.wdata_b_i (64'd0),
		.q_b_o     (rd_raw)
	);

endmodule
