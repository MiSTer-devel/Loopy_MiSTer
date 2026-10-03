// RH-7500 CPU bus interface, the /CS4 side of the chip.
//
// The CPU and VDP run from separate crystals. Strobes, address, byte lanes and
// write data all go through the same two synchroniser flops. WAIT asserts
// combinationally from the pins because the CPU samples it about 31 ns after
// the strobe, sooner than the synchroniser; only the release is registered.
//
// WAIT lengths, in clk_video cycles (twice the VDP clock, so half cycles fit):
//
//   registers            2 VDP cycles
//   bitmap read          4 VDP cycles, 3 with FBM
//   bitmap write         1.5 VDP cycles, 0.5 with FBM
//   other VDP memory     3 VDP cycles
//
// An undecoded address reads back the last value on the internal bus.
//
//   0x00000-0x3FFFF  bitmap VRAM, 128 KB with A17 ignored so it mirrors once
//   0x40000-0x4FFFF  tile VRAM, 64 KB
//   0x50000-0x501FF  OAM
//   0x51000-0x511FF  palette
//   0x52000-0x521FF  capture buffer
//   0x58000-0x5FFFF  registers and the flash-write trigger
//   0x60000-0x6FFFF  sync calibration and render debug, write only
//   0x80000-0x9FFFF  sound control, write only, mirrored over the whole range
//   0xA0000-0xFFFFF  expansion strobes 1, 2 and 3, 128 KB each

module vdp_cpu_if
(
	input  wire clk,
	input  wire reset,
	input  wire ce_vdp,

	// Pin side.
	input  wire        cs_n,
	input  wire        rd_n,
	input  wire        wrh_n,
	input  wire        wrl_n,
	input  wire [19:0] a,
	input  wire [15:0] d_i,
	output wire [15:0] d_o,
	output wire        d_oe,
	output wire        wait_n,

	// Fast bitmap access, from BM_MEM_CTRL.
	input  wire        bm_fast,

	// Fast expansion access, from EXP_TIMING. Bits 3-1 are expansion areas 3,
	// 2 and 1; bit 0 is the sound control latch.
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [3:0]  exp_fast,
	/* verilator lint_on UNUSEDSIGNAL */

	// Holds WAIT low past the count: bitmap VRAM during a flash write.
	input  wire        stall,

	// The address is one whose WAIT is counted on the CPU clock; this side
	// then only holds WAIT for a stall.
	input  wire        cpu_timed,

	// The VDP is about to use, or is using, the bitmap VRAM for itself.
	input  wire        bm_hold_rd,
	input  wire        bm_hold_wr,

	// Internal bus, common to every target.
	output reg  [19:1] cpu_addr,
	output reg  [15:0] cpu_wdata,
	output reg  [1:0]  cpu_be,
	output reg         cpu_wr,      // one clk_video cycle
	output reg         cpu_rd,      // one clk_video cycle
	output wire        cpu_busy,    // high for the whole of an access
	// Expansion strobe window. Ends before WAIT is released so the cart latch,
	// clocked on the strobe's trailing edge, still sees the CPU's data.
	output wire        cpu_exp_busy,

	// Target selects, valid with cpu_wr or cpu_rd.
	output reg         sel_bitmap,
	output reg         sel_tile,
	output reg         sel_oam,
	output reg         sel_pal,
	output reg         sel_cap,
	output reg         sel_reg,
	output reg         sel_fill,    // the flash-write trigger range
	output reg         sel_io,
	output reg         sel_sound,
	output reg  [2:0]  sel_exp,     // one hot, EXP1 in bit 0

	// Read data from the targets, one cycle after cpu_rd for the memories and
	// combinational for the registers and IO. `rdata_hit` says the address
	// decoded to something readable.
	input  wire [15:0] rdata,
	input  wire        rdata_hit
);

	// ---- strobe synchronisers ------------------------------------------------

	reg [1:0] cs_s, rd_s, wrh_s, wrl_s;
	reg [19:0] a_s0, a_s1;
	reg [15:0] d_s0, d_s1;

	always @(posedge clk) begin
		if (reset) begin
			cs_s <= 2'b11; rd_s <= 2'b11; wrh_s <= 2'b11; wrl_s <= 2'b11;
			a_s0 <= 20'd0; a_s1 <= 20'd0;
			d_s0 <= 16'd0; d_s1 <= 16'd0;
		end else begin
			cs_s  <= {cs_s[0],  cs_n};
			rd_s  <= {rd_s[0],  rd_n};
			wrh_s <= {wrh_s[0], wrh_n};
			wrl_s <= {wrl_s[0], wrl_n};
			a_s0  <= a;   a_s1 <= a_s0;
			d_s0  <= d_i; d_s1 <= d_s0;
		end
	end

	wire sel     = ~cs_s[1];
	wire rd_act  = sel & ~rd_s[1];
	wire wr_act  = sel & (~wrh_s[1] | ~wrl_s[1]);
	wire acc     = rd_act | wr_act;

	reg acc_d;
	always @(posedge clk) acc_d <= reset ? 1'b0 : acc;
	wire acc_start = acc & ~acc_d;

	// ---- decode --------------------------------------------------------------
	// From the shadowed address, so it describes the access the strobe belongs to.
	wire d_bitmap = (a_s1[19:18] == 2'b00);
	wire d_tile   = (a_s1[19:16] == 4'h4);
	wire p5       = (a_s1[19:16] == 4'h5);
	wire d_oam    = p5 & (a_s1[15:12] == 4'h0) & (a_s1[11:9] == 3'd0);
	wire d_pal    = p5 & (a_s1[15:12] == 4'h1) & (a_s1[11:9] == 3'd0);
	wire d_cap    = p5 & (a_s1[15:12] == 4'h2) & (a_s1[11:9] == 3'd0);
	wire d_io     = p5 & (a_s1[15:12] == 4'hD);
	wire d_fill   = p5 & (a_s1[15:12] == 4'hF);
	wire d_reg    = (p5 & ~d_oam & ~d_pal & ~d_cap & ~d_io)
	              | (a_s1[19:16] == 4'h6);
	wire d_sound  = (a_s1[19:17] == 3'b100);
	wire d_exp1   = (a_s1[19:17] == 3'b101);
	wire d_exp2   = (a_s1[19:17] == 3'b110);
	wire d_exp3   = (a_s1[19:17] == 3'b111);

	// ---- access timing -------------------------------------------------------
	// Length in clk_video cycles, from the first cycle of the access.

	localparam [3:0] W_REG      = 4'd4;   // 2 VDP cycles
	localparam [3:0] W_MEM      = 4'd6;   // 3 VDP cycles
	localparam [3:0] W_BM_RD    = 4'd8;   // 4 VDP cycles
	localparam [3:0] W_BM_RD_F  = 4'd6;   // 3
	localparam [3:0] W_BM_WR    = 4'd3;   // 1.5
	localparam [3:0] W_BM_WR_F  = 4'd1;   // 0.5
	// Expansion: fast is two CPU cycles, slow just under three. Long enough to
	// hold the CPU on the bus for the whole strobe.
	localparam [3:0] W_EXP      = 4'd12;  // slow, read-write
	localparam [3:0] W_EXP_F    = 4'd8;   // fast, write only

	wire       d_exp_any = d_exp1 | d_exp2 | d_exp3;
	wire       exp_is_fast = (d_exp1 & exp_fast[1])
	                       | (d_exp2 & exp_fast[2])
	                       | (d_exp3 & exp_fast[3]);

	wire [3:0] wait_len =
		  d_bitmap ? (rd_act ? (bm_fast ? W_BM_RD_F : W_BM_RD)
		                     : (bm_fast ? W_BM_WR_F : W_BM_WR))
		: (d_tile | d_oam | d_pal | d_cap) ? W_MEM
		: d_exp_any ? (exp_is_fast ? W_EXP_F : W_EXP)
		: W_REG;

	reg [3:0] wcnt;
	reg       busy;
	reg       is_rd;
	reg       wr_pending, rd_pending;

	// Selected and strobed, straight off the pins.
	wire raw_acc = ~cs_n & (~rd_n | ~wrh_n | ~wrl_n);

	// Set one cycle after cpu_rd, when the target's data is on the internal bus.
	reg capture_rd;

	// A read may not finish before its data is in the latch, whatever the
	// counter says.
	reg rd_latched;
	always @(posedge clk) begin
		if (reset)          rd_latched <= 1'b0;
		else if (!raw_acc)  rd_latched <= 1'b0;
		else if (capture_rd) rd_latched <= 1'b1;
	end

	// Tile VRAM takes a write about every six CPU states; an access that
	// comes sooner waits.
	localparam [4:0] TW_GAP = 5'd14;   // clk_video
	reg  [4:0] tw_busy;
	always @(posedge clk) begin
		if (reset)                  tw_busy <= 5'd0;
		else if (cpu_wr & sel_tile) tw_busy <= TW_GAP;
		else if (tw_busy != 5'd0)   tw_busy <= tw_busy - 5'd1;
	end

	// Set once this access has made its transfer; nothing holds it after that.
	reg xfer;
	always @(posedge clk) begin
		if (reset | ~raw_acc)   xfer <= 1'b0;
		else if (cpu_wr | cpu_rd) xfer <= 1'b1;
	end

	// A bitmap access that has not started its VRAM cycle waits while the VDP
	// has the VRAM. WAIT is raised from the pins so the CPU sees it in time.
	wire steal_raw = raw_acc & ~xfer
	               & (((a[19:18] == 2'b00) & (~rd_n ? bm_hold_rd : bm_hold_wr))
	                | ((a[19:16] == 4'h4) & (tw_busy > 5'd3)));
	wire steal     = busy & (rd_pending | wr_pending)
	               & ((sel_bitmap & (is_rd ? bm_hold_rd : bm_hold_wr))
	                | (sel_tile & (tw_busy != 5'd0)));
	wire hold      = stall | steal;

	// Set once this access has had its wait, cleared when the CPU lets go.
	reg acc_done;
	always @(posedge clk) begin
		if (reset)         acc_done <= 1'b0;
		else if (!raw_acc) acc_done <= 1'b0;
		else if (busy & (wcnt == 4'd0) & ~hold & (~is_rd | rd_latched))
			acc_done <= 1'b1;
	end

	// Once stalled, WAIT holds until the access has gone through.
	reg stalled;
	always @(posedge clk) begin
		if (reset || !raw_acc) stalled <= 1'b0;
		else if (busy & hold) stalled <= 1'b1;
	end

	assign wait_n = ~(raw_acc & ~acc_done & (~cpu_timed | hold | stalled | steal_raw));

	// Registered part of the access only, so it lines up with the latched
	// target selects.
	assign cpu_busy = busy;

	// One strobe per access. `acc_done` clears off the pins while `busy` lags
	// two flops behind, so `busy & ~acc_done` could re-arm after the CPU let
	// go; this is set only at access start.
	reg exp_win;
	always @(posedge clk) begin
		if (reset)                exp_win <= 1'b0;
		else if (acc_start)       exp_win <= 1'b1;
		else if (busy & (wcnt == 4'd0) & ~hold & (~is_rd | rd_latched))
			exp_win <= 1'b0;
	end

	assign cpu_exp_busy = exp_win;

	always @(posedge clk) begin
		cpu_wr <= 1'b0;
		cpu_rd <= 1'b0;

		if (reset) begin
			busy <= 1'b0;
			wcnt <= 4'd0;
			is_rd <= 1'b0;
			wr_pending <= 1'b0;
			rd_pending <= 1'b0;
			cpu_addr <= 19'd0; cpu_wdata <= 16'd0; cpu_be <= 2'd0;
			sel_bitmap <= 1'b0; sel_tile <= 1'b0; sel_oam <= 1'b0;
			sel_pal <= 1'b0; sel_cap <= 1'b0; sel_reg <= 1'b0; sel_fill <= 1'b0;
			sel_io <= 1'b0; sel_sound <= 1'b0; sel_exp <= 3'd0;
		end else if (acc_start) begin
			busy  <= 1'b1;
			// WAIT went low on the pins about three clk_video cycles before
			// this point; those count toward the length, so only the
			// remainder is counted here.
			wcnt  <= (wait_len > 4'd4) ? (wait_len - 4'd4) : 4'd0;
			is_rd <= rd_act;

			cpu_addr  <= a_s1[19:1];
			cpu_wdata <= d_s1;
			cpu_be    <= rd_act ? 2'b11 : {~wrh_s[1], ~wrl_s[1]};
			// The transfer waits for any stall to clear; a flash write owns
			// the bitmap CPU port while it runs.
			wr_pending <= wr_act;
			rd_pending <= rd_act;

			sel_bitmap <= d_bitmap;
			sel_tile   <= d_tile;
			sel_oam    <= d_oam;
			sel_pal    <= d_pal;
			sel_cap    <= d_cap;
			sel_reg    <= d_reg;
			sel_fill   <= d_fill;
			sel_io     <= d_io;
			sel_sound  <= d_sound;
			sel_exp    <= {d_exp3, d_exp2, d_exp1};
		end else begin
			if (wcnt != 4'd0) wcnt <= wcnt - 4'd1;
			if (!acc) busy <= 1'b0;
			if (wr_pending & ~hold) begin cpu_wr <= 1'b1; wr_pending <= 1'b0; end
			if (rd_pending & ~hold) begin cpu_rd <= 1'b1; rd_pending <= 1'b0; end
		end
	end

	// ---- read latch ----------------------------------------------------------
	// Memories answer one cycle after cpu_rd, registers and IO combinationally.
	// The latch holds the last value read for the next open-bus read; writes
	// leave it alone.

	reg [15:0] bus_latch;

	always @(posedge clk) begin
		if (reset) begin
			bus_latch  <= 16'd0;
			capture_rd <= 1'b0;
		end else begin
			capture_rd <= cpu_rd;
			if (capture_rd & rdata_hit) bus_latch <= rdata;
		end
	end

	// The expansion areas belong to the cartridge; everything else, decoded or
	// not, is answered from the latch. The enable follows the pins so it drops
	// as soon as the CPU lets go.
	assign d_o  = bus_latch;
	assign d_oe = raw_acc & busy & is_rd & ~(|sel_exp);

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	// A0 has no meaning on a 16-bit port; the byte lanes carry it.
	wire _unused = &{1'b0, ce_vdp, a, d_i, a_s1[0]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

endmodule
