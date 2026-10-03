// Tile VRAM: 64 KB of tilemaps and character graphics shared by the
// background and object layers. Organised 8192 x 64 bits, big-endian, so one
// read is an 8bpp character row, two 4bpp rows or four tilemap entries.
//
// The real SRAM pair is single ported and shares one write enable: a CPU
// byte write lands as a whole word, reproduced here. The snow a CPU access
// makes in the render fetch is drawn by the background layer.
//
// Render port schedule, eight clk_video cycles per pixel:
//
//   phase 0  BG0 tilemap      phase 4  BG0 character
//   phase 1  BG1 tilemap      phase 5  BG1 character
//   phase 2  objects          phase 6  objects
//   phase 3  objects          phase 7  objects
//
// Outside the render window every phase goes to the object engine. Each engine
// presents its address in its phase and takes rd_data the phase after. Map and
// character slots sit four phases apart to give the character address time.

module vdp_tile_vram
(
	input  wire clk,

	// CPU port.
	input  wire        cpu_sel,
	input  wire        cpu_wr,
	input  wire [15:1] cpu_addr,
	input  wire [15:0] cpu_wdata,
	output wire [15:0] cpu_rdata,

	// Render port.
	input  wire [2:0]  phase,
	input  wire        in_window,
	input  wire [12:0] bg0_map_addr,
	input  wire [12:0] bg0_chr_addr,
	input  wire [12:0] bg1_map_addr,
	input  wire [12:0] bg1_chr_addr,
	input  wire [12:0] obj_addr,
	output wire [63:0] rd_data,

	// Savestate walk, takes port A ahead of the CPU.
	input  wire        ss_sel,
	input  wire [15:0] ss_addr,
	input  wire        ss_we,
	input  wire [7:0]  ss_wdata,
	output wire [7:0]  ss_rdata
);

	wire [2:0] hw  = {1'b0, cpu_addr[2:1]};
	wire [2:0] hwr = 3'd3 - hw;
	wire [7:0] cpu_be64 = 8'd3 << (hwr * 3'd2);

	wire [63:0] a_q;
	assign cpu_rdata = a_q[hwr * 6'd16 +: 16];

	reg [12:0] b_addr;
	always @* begin
		if (!in_window) begin
			b_addr = obj_addr;
		end else begin
			case (phase)
			3'd0:    b_addr = bg0_map_addr;
			3'd1:    b_addr = bg1_map_addr;
			3'd4:    b_addr = bg0_chr_addr;
			3'd5:    b_addr = bg1_chr_addr;
			default: b_addr = obj_addr;
			endcase
		end
	end

	wire [63:0] b_q;

	assign rd_data = b_q;

	wire [7:0] ss_be = 8'h80 >> ss_addr[2:0];
	reg  [2:0] ss_lane_q;
	always @(posedge clk) ss_lane_q <= 3'd7 - ss_addr[2:0];
	assign ss_rdata = a_q[ss_lane_q * 6'd8 +: 8];

	cache_ram_dp_be #(
		.ADDR_WIDTH (13),
		.DATA_WIDTH (64)
	) u_ram (
		.clk_i     (clk),
		.addr_a_i  (ss_sel ? ss_addr[15:3] : cpu_addr[15:3]),
		.wren_a_i  (ss_sel ? ss_we : (cpu_sel & cpu_wr)),
		.be_a_i    (ss_sel ? ss_be : cpu_be64),
		.wdata_a_i (ss_sel ? {8{ss_wdata}} : {4{cpu_wdata}}),
		.q_a_o     (a_q),
		.addr_b_i  (b_addr),
		.wren_b_i  (1'b0),
		.be_b_i    (8'h00),
		.wdata_b_i (64'd0),
		.q_b_o     (b_q)
	);

endmodule
