// Read-during-write forwarding for the VDP's small dual-port RAMs.
// An M10K returns undefined data when one port reads the address the other
// writes in the same cycle; the original parts return the new data, so the
// write is forwarded byte by byte under the byte enables. The compare is
// registered because the RAM output lags its address by one cycle.

module vdp_rdw_fwd #(
	parameter ADDR_WIDTH = 8,
	parameter BYTES      = 2
)
(
	input  wire                  clk,
	input  wire                  reset,

	// The writing port.
	input  wire [ADDR_WIDTH-1:0] wr_addr,
	input  wire                  wr_en,
	input  wire [BYTES-1:0]      wr_be,
	input  wire [BYTES*8-1:0]    wr_data,

	// The reading port: address, raw RAM output, and the corrected word.
	input  wire [ADDR_WIDTH-1:0] rd_addr,
	input  wire [BYTES*8-1:0]    rd_raw,
	output wire [BYTES*8-1:0]    rd_data
);

	reg                hit;
	reg [BYTES-1:0]    be_q;
	reg [BYTES*8-1:0]  data_q;

	always @(posedge clk) begin
		if (reset) begin
			hit <= 1'b0;
		end else begin
			hit    <= wr_en & (wr_addr == rd_addr);
			be_q   <= wr_be;
			data_q <= wr_data;
		end
	end

	genvar b;
	generate
		for (b = 0; b < BYTES; b = b + 1) begin : g_byte
			assign rd_data[b*8 +: 8] = (hit & be_q[b])
				? data_q[b*8 +: 8]
				: rd_raw[b*8 +: 8];
		end
	endgenerate

endmodule
