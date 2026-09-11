// Takes content from the HPS and puts it where the machine expects it.
//
//   cartridge ROM  -> SDRAM, through ext_mem_bridge
//   BIOS           -> the SH7021's internal mask ROM, a block RAM
//   synth wave ROM -> SDRAM, through ext_mem_bridge
//
// OSD index 0 belongs to the framework, so slots number from 1 and boot0.rom
// is the BIOS. hps_io WIDE words are little-endian from the file and the
// SDRAM controller writes din[7:0] to the even byte, so a word passed through
// unchanged lands byte for byte.
//
// Cartridge dumps circulate two ways round. The header long word is
// 0x0E000080 in CPU order, so the first two words off the wire settle it:
//
//   file 0E 00 00 80   ->  words 000E 8000   already in CPU order
//   file 00 0E 80 00   ->  words 0E00 0080   word-swapped, swap each word
//
// Those two words are held until the direction is known; the second is
// written from defer_* while ioctl_wait holds the HPS off.

module loopy_loader
(
	input  wire clk,
	input  wire reset,

	input  wire        ioctl_download,
	input  wire [7:0]  ioctl_index,
	input  wire        ioctl_wr,
	input  wire [26:0] ioctl_addr,
	input  wire [15:0] ioctl_dout,
	output wire        ioctl_wait,

	// Into SDRAM.
	output reg         ld_req,
	output reg  [25:0] ld_addr,
	output reg  [15:0] ld_din,
	input  wire        ld_busy,
	output wire        loading,

	// Into the SH7021's internal ROM.
	output reg         bios_wr,
	output reg  [13:0] bios_addr,     // 16-bit word address inside 32 KB
	output reg  [15:0] bios_data,

	output reg         cart_loaded,

	// ROM repeats through its 4 MB area and SRAM wraps modulo the header size.
	// Anything over 2 MB is a two-chip board: cart_split selects cart_hi_mask
	// for the top half.
	output reg  [21:1] cart_mask,
	output reg         cart_split,
	output reg  [20:1] cart_hi_mask,
	output reg  [16:0] sram_mask
);

	// Round up to the next power of two, minus one.
	function automatic [22:0] smear(input [22:0] v);
		reg [22:0] m;
		begin
			m = v;
			m = m | (m >> 1);
			m = m | (m >> 2);
			m = m | (m >> 4);
			m = m | (m >> 8);
			m = m | (m >> 16);
			smear = m;
		end
	endfunction

	// Menu slots. The OSD's F entries number from 1: index 0 belongs to the
	// framework, which uses it for boot0.rom.
	localparam [7:0] INDEX_CART = 8'd1;    // FS1
	localparam [7:0] INDEX_BIOS = 8'd2;    // F2
	localparam [7:0] INDEX_WAVE = 8'd3;    // F3

	// boot<N>.rom carries N in ioctl_index[15:6] with zero in [5:0], so boot0
	// and boot1 arrive as 8'h00 and 8'h40. Main loads them from the core's
	// games folder at startup.
	localparam [7:0] INDEX_BOOT0 = 8'h00;   // BIOS
	localparam [7:0] INDEX_BOOT1 = 8'h40;   // synth wave ROM

	localparam logic [25:0] CART_BASE = 26'h000000;
	localparam logic [25:0] WAVE_BASE = 26'h2000000;   // bank 2, see ext_mem_bridge

	wire is_cart = (ioctl_index == INDEX_CART);
	wire is_bios = (ioctl_index == INDEX_BIOS) | (ioctl_index == INDEX_BOOT0);
	wire is_wave = (ioctl_index == INDEX_WAVE) | (ioctl_index == INDEX_BOOT1);

	// One write in flight, plus the one slot the header pair needs.
	reg        have_defer;
	reg [25:0] defer_addr;
	reg [15:0] defer_data;

	// Held until the byte order is known.
	reg [15:0] first_word;
	reg        have_first;
	reg        order_known;

	// The bridge's busy stays high for the whole crossing and SDRAM access, not
	// just until the request is taken, so it has to hold the HPS off too.
	assign ioctl_wait = ld_req | have_defer | ld_busy;

	// `loading` holds the machine in reset and outlasts the download so the
	// last word can finish crossing to clk_ram. A fixed drain count (about
	// 8 us) rather than `ld_busy`, which after the download reports work-DRAM
	// traffic on the shared p0 port.
	reg [7:0] ld_drain;
	assign loading = ioctl_download | (ld_drain != 8'd0);

	reg         cart_swapped;
	reg  [31:0] cart_sram_first, cart_sram_last;
	wire [15:0] swapped = {ioctl_dout[7:0], ioctl_dout[15:8]};
	wire [15:0] cart_word = cart_swapped ? swapped : ioctl_dout;

	// SRAM bounds out of the header at 0x10 and 0x14, read once the stream is
	// the right way round. Stored big-endian as the CPU reads them, so the
	// SDRAM word halves are swapped.
	task automatic capture_header(input [26:0] a, input [15:0] w);
		begin
			if (a < 27'h18) begin
				case (a[4:0])
					5'h10: cart_sram_first[31:16] <= w;
					5'h12: cart_sram_first[15:0]  <= w;
					5'h14: cart_sram_last[31:16]  <= w;
					5'h16: cart_sram_last[15:0]   <= w;
					default: ;
				endcase
			end
		end
	endtask

	// A word arrives only when ioctl_wait is low, so the bridge is idle and
	// nothing is in flight when this is called.
	task automatic present(input [25:0] a, input [15:0] d);
		begin
			ld_req  <= 1'b1;
			ld_addr <= a;
			ld_din  <= d;
		end
	endtask

	reg download_d;
	reg [22:0] cart_bytes;

	// The SRAM range is a header field and only counts when it points into
	// area 2; anything else means the cartridge has no SRAM fitted.
	wire        sram_ok    = (cart_sram_first[31:24] == 8'h02)
	                       && (cart_sram_last[31:24] == 8'h02)
	                       && (cart_sram_last >= cart_sram_first);
	wire [22:0] sram_bytes = sram_ok
	                       ? (cart_sram_last[22:0] - cart_sram_first[22:0] + 23'd1)
	                       : 23'd0;

	// Named rather than sliced at the call: Quartus 17 will not take a part
	// select of a function result.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [22:0] cart_span = smear(cart_bytes - 23'd1);
	wire [22:0] sram_span = smear(sram_bytes - 23'd1);
	/* verilator lint_on UNUSEDSIGNAL */

	// Second chip on a two-ROM board: whatever is left above the first chip's
	// 2 MB, rounded up the same way. Little Romance and Loopy Town are 16 Mbit
	// plus 8 Mbit, so 3 MB in and 1 MB here.
	localparam logic [22:0] CHIP1_BYTES = 23'h200000;

	wire        two_chip  = (cart_bytes > CHIP1_BYTES);
	wire [22:0] hi_bytes  = two_chip ? (cart_bytes - CHIP1_BYTES) : 23'd0;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [22:0] hi_span   = smear(hi_bytes - 23'd1);
	/* verilator lint_on UNUSEDSIGNAL */

	always @(posedge clk) begin
		if (reset) begin
			ld_req       <= 1'b0;
			ld_drain     <= 8'd0;
			have_defer   <= 1'b0;
			bios_wr      <= 1'b0;
			have_first   <= 1'b0;
			order_known  <= 1'b0;
			cart_swapped <= 1'b1;
			cart_loaded  <= 1'b0;
			download_d   <= 1'b0;
			cart_bytes   <= 23'd0;
			cart_mask    <= 21'd0;
			cart_split   <= 1'b0;
			cart_hi_mask <= 20'd0;
			sram_mask    <= 17'd0;
			cart_sram_first <= 32'd0;
			cart_sram_last  <= 32'd0;
		end else begin
			bios_wr    <= 1'b0;
			download_d <= ioctl_download;

			if (ioctl_download)         ld_drain <= 8'hFF;
			else if (ld_drain != 8'd0)  ld_drain <= ld_drain - 8'd1;

			// Start of a transfer: forget what the last cartridge said.
			if (ioctl_download && !download_d && is_cart) begin
				cart_bytes  <= 23'd0;
				have_first  <= 1'b0;
				order_known <= 1'b0;
				cart_sram_first <= 32'd0;
				cart_sram_last  <= 32'd0;
			end

			// End of a transfer: work out the cartridge chip sizes once.
			if (!ioctl_download && download_d) begin
				if (is_cart) begin
					// Two chips split the area at A21, so the first one is the
					// whole bottom 2 MB whatever the image size is.
					cart_split   <= two_chip;
					cart_mask    <= two_chip ? {1'b0, 20'hFFFFF} : cart_span[21:1];
					cart_hi_mask <= two_chip ? hi_span[20:1] : 20'd0;
					sram_mask    <= (sram_bytes == 23'd0) ? 17'd0 : sram_span[16:0];
				end
				if (is_cart) cart_loaded <= 1'b1;
			end

			if (ioctl_wr) begin
				if (is_bios) begin
					bios_wr   <= 1'b1;
					bios_addr <= ioctl_addr[14:1];
					bios_data <= ioctl_dout;
				end else if (is_wave) begin
					// 512 KB, so 19 bits of offset.
					present(WAVE_BASE | {7'd0, ioctl_addr[18:0]}, ioctl_dout);
				end else if (is_cart) begin
					cart_bytes <= {1'b0, ioctl_addr[21:0]} + 23'd2;
					if (!order_known) begin
						if (!have_first) begin
							first_word <= ioctl_dout;
							have_first <= 1'b1;
						end else begin
							// Second word decides it.
							if (first_word == 16'h000E && ioctl_dout == 16'h8000) begin
								cart_swapped <= 1'b0;
								present(CART_BASE, first_word);
								defer_data <= ioctl_dout;
							end else begin
								cart_swapped <= 1'b1;
								present(CART_BASE, {first_word[7:0], first_word[15:8]});
								defer_data <= swapped;
							end
							defer_addr <= CART_BASE | 26'd2;
							have_defer <= 1'b1;
							order_known <= 1'b1;
						end
					end else begin
						present(CART_BASE | {4'd0, ioctl_addr[21:0]}, cart_word);
						capture_header(ioctl_addr, {cart_word[7:0], cart_word[15:8]});
					end
				end
			end

			// The bridge takes the request on the edge where req is high and
			// busy is low, the edge after present() ran.
			if (ld_req) begin
				ld_req <= 1'b0;
			end else if (have_defer && !ld_busy) begin
				ld_req     <= 1'b1;
				ld_addr    <= defer_addr;
				ld_din     <= defer_data;
				have_defer <= 1'b0;
			end
		end
	end

endmodule
