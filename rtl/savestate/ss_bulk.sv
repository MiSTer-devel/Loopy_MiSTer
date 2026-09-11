// Copyright (c) 2026 Jamie Blanks

// The savestate's bulk memory walk: one byte at a time, wherever it lives.
//
//   0  work DRAM    512 KB   SDRAM, through the memory bridge
//   1  bitmap VRAM  128 KB   RH-7500 block RAM, across the video clock
//   2  tile VRAM     64 KB   the same
//   3  OAM             512 B the same
//   4  palette         512 B the same
//   5  cart SRAM     32 KB   block RAM on clk_sys, port B
//   6  on-chip RAM    3 KB   SH7021 RAM then the synth's voice RAM
//
// The transfer engine raises rd or wr, waits at least seven clocks, then
// waits for ready. It walks each region in order, which lets the work DRAM
// group eight bytes into one SDRAM access.
//
// Everything here runs with the machine parked, so no port is contended.

module ss_bulk (
	input  wire        clk,
	input  wire        reset,

	// ---- the transfer engine ---------------------------------------------
	input  wire [24:0] addr_i,
	input  wire [2:0]  type_i,
	input  wire        rd_i,
	input  wire        wr_i,
	input  wire [7:0]  wdata_i,
	output reg  [7:0]  rdata_o,
	output wire        ready_o,

	// ---- work DRAM, through the memory bridge -----------------------------
	output wire        dram_own_o,
	output reg         dram_req_o,
	output reg         dram_we_o,
	output wire [18:3] dram_line_o,
	output reg  [63:0] dram_din_o,
	output wire [7:0]  dram_be_o,
	input  wire [63:0] dram_dout_i,
	input  wire        dram_done_i,

	// ---- RH-7500 memories, across the video clock -------------------------
	output reg         vdp_req_o,
	input  wire        vdp_done_i,
	input  wire [7:0]  vdp_rdata_i,

	// ---- cartridge SRAM, port B -------------------------------------------
	output wire        sram_own_o,
	output wire [13:0] sram_addr_o,
	output wire        sram_we_o,
	output wire [1:0]  sram_be_o,
	output wire [15:0] sram_wdata_o,
	input  wire [15:0] sram_q_i,

	// ---- SH7021 on-chip RAM ------------------------------------------------
	output wire [9:0]  ram_addr_o,
	output wire        ram_we_o,
	output wire [7:0]  ram_wdata_o,
	input  wire [7:0]  ram_q_i,

	// ---- CDT109 voice RAM --------------------------------------------------
	output reg         voice_rd_o,
	output reg         voice_wr_o,
	output wire [10:0] voice_addr_o,
	output wire [7:0]  voice_wdata_o,
	input  wire [7:0]  voice_q_i,
	input  wire        voice_ready_i
);

	`include "ss_map.svh"

	localparam [2:0] S_IDLE  = 3'd0;
	localparam [2:0] S_LOCAL = 3'd1;   // block RAM on this clock: one turnaround
	localparam [2:0] S_VDP   = 3'd2;
	localparam [2:0] S_DRAM  = 3'd3;
	localparam [2:0] S_VOICE = 3'd4;
	localparam [2:0] S_DONE  = 3'd5;

	reg [2:0] st;
	reg       done_q;

	// What the transfer that is finishing was for. The engine holds rd or wr
	// high across the eight bytes of a word and only steps the address, so a
	// finished transfer has to be recognised by the address moving on as well
	// as by the request dropping.
	reg [24:0] served_addr_q;
	reg [2:0]  served_type_q;
	wire       moved_on = (addr_i != served_addr_q) | (type_i != served_type_q);

	wire go = (rd_i | wr_i) & ~done_q & (st == S_IDLE);
	assign ready_o = done_q;

	wire is_vdp   = (type_i == SS_MEM_VRAM) || (type_i == SS_MEM_TILERAM)
	             || (type_i == SS_MEM_OAM)  || (type_i == SS_MEM_PALETTE);
	wire is_dram  = (type_i == SS_MEM_WORKRAM);
	wire is_sram  = (type_i == SS_MEM_CARTSRAM);
	wire is_chip  = (type_i == SS_MEM_ONCHIP);
	// The on-chip region is the SH7021's 1 KB followed by the synth's voice
	// RAM, which is 32 voices of 64 byte slots.
	wire [11:0] chip_off  = addr_i[11:0];
	/* verilator lint_off UNUSEDSIGNAL */
	wire [11:0] voice_off = chip_off - 12'd1024;   // 2 KB, so bit 11 falls away
	/* verilator lint_on UNUSEDSIGNAL */
	wire        is_voice  = is_chip && (chip_off >= 12'd1024);

	// "Own" means a request is in flight, not merely that the type register
	// names this memory: the engine leaves the type behind after a walk, and
	// holding the work DRAM's port on that would stall the CPU for ever.
	wire active = rd_i | wr_i | (st != S_IDLE);

	assign sram_own_o  = is_sram && active;
	assign sram_addr_o = addr_i[14:1];
	assign sram_we_o   = is_sram && wr_i && (st == S_LOCAL);
	assign sram_be_o   = addr_i[0] ? 2'b01 : 2'b10;
	assign sram_wdata_o = {2{wdata_i}};

	assign ram_addr_o  = addr_i[9:0];
	assign ram_we_o    = is_chip && !is_voice && wr_i && (st == S_LOCAL);
	assign ram_wdata_o = wdata_i;

	assign voice_addr_o  = voice_off[10:0];
	assign voice_wdata_o = wdata_i;

	// ---- work DRAM ---------------------------------------------------------

	// One SDRAM access serves eight bytes. The engine walks a region in order,
	// so the read line only has to be refetched when the byte index wraps, and
	// a write only goes out on the last byte of a group.
	reg [63:0] dram_line_q;
	reg [18:3] dram_tag_q;
	reg        dram_tag_v;
	reg [18:3] dram_line_sel;
	reg [7:0]  dram_be_q;

	assign dram_own_o  = is_dram && active;
	assign dram_line_o = dram_line_sel;
	assign dram_be_o   = dram_be_q;

	wire        dram_hit = dram_tag_v && (dram_tag_q == addr_i[18:3]);
	wire [18:3] dram_line_w = addr_i[18:3];
	wire [5:0]  byte_bit = {addr_i[2:0], 3'd0};

	always @(posedge clk) begin
		if (reset) begin
			st         <= S_IDLE;
			done_q     <= 1'b0;
			dram_req_o <= 1'b0;
			dram_we_o  <= 1'b0;
			dram_tag_v <= 1'b0;
			dram_be_q  <= 8'hFF;
			vdp_req_o  <= 1'b0;
			voice_rd_o <= 1'b0;
			voice_wr_o <= 1'b0;
			rdata_o    <= 8'd0;
			served_addr_q <= 25'd0;
			served_type_q <= 3'd0;
		end else begin
			dram_req_o <= 1'b0;
			vdp_req_o  <= 1'b0;

			case (st)
			S_IDLE: begin
				if (go) begin
					served_addr_q <= addr_i;
					served_type_q <= type_i;
					if (is_vdp) begin
						vdp_req_o <= 1'b1;
						st <= S_VDP;
					end else if (is_dram) begin
						if (wr_i) begin
							// Collect the byte; the whole word goes out on the
							// last one. A write also stales the read line.
							dram_din_o[byte_bit +: 8] <= wdata_i;
							if (dram_tag_q == addr_i[18:3]) dram_tag_v <= 1'b0;
							if (addr_i[2:0] == 3'd7) begin
								dram_line_sel <= dram_line_w;
								dram_we_o     <= 1'b1;
								dram_be_q     <= 8'hFF;
								dram_req_o    <= 1'b1;
								st <= S_DRAM;
							end else begin
								done_q <= 1'b1;
								st <= S_DONE;
							end
						end else if (dram_hit) begin
							rdata_o <= dram_line_q[byte_bit +: 8];
							done_q  <= 1'b1;
							st <= S_DONE;
						end else begin
							dram_line_sel <= dram_line_w;
							dram_we_o     <= 1'b0;
							dram_req_o    <= 1'b1;
							st <= S_DRAM;
						end
					end else if (is_voice) begin
						voice_rd_o <= rd_i;
						voice_wr_o <= wr_i;
						st <= S_VOICE;
					end else begin
						st <= S_LOCAL;
					end
				end
			end

			// Block RAM on this clock answers the cycle after the address, and
			// a write needs no answer at all.
			S_LOCAL: begin
				rdata_o <= is_sram ? (addr_i[0] ? sram_q_i[7:0] : sram_q_i[15:8])
				                   : ram_q_i;
				done_q  <= 1'b1;
				st <= S_DONE;
			end

			S_VDP: begin
				if (vdp_done_i) begin
					rdata_o <= vdp_rdata_i;
					done_q  <= 1'b1;
					st <= S_DONE;
				end
			end

			S_DRAM: begin
				if (dram_done_i) begin
					if (!dram_we_o) begin
						dram_line_q <= dram_dout_i;
						dram_tag_q  <= addr_i[18:3];
						dram_tag_v  <= 1'b1;
						rdata_o     <= dram_dout_i[byte_bit +: 8];
					end
					dram_we_o <= 1'b0;
					done_q    <= 1'b1;
					st <= S_DONE;
				end
			end

			S_VOICE: begin
				if (voice_ready_i) begin
					voice_rd_o <= 1'b0;
					voice_wr_o <= 1'b0;
					rdata_o    <= voice_q_i;
					done_q     <= 1'b1;
					st <= S_DONE;
				end
			end

			S_DONE: begin
				if ((!rd_i && !wr_i) || moved_on) begin
					done_q <= 1'b0;
					st <= S_IDLE;
				end
			end

			default: st <= S_IDLE;
			endcase
		end
	end

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, addr_i[24:19]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

endmodule
