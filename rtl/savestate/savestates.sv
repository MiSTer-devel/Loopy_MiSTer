// Copyright (c) 2026 Jamie Blanks
//
// Savestate transfer engine.
//
// Walks the core's register spine (BUS_*, one 64-bit word per address) and
// then each memory region in turn (Save_RAM*, eight bytes packed into one
// word), moving both to and from DDR over bus_out_*. A two-word header
// carries the state size and a running save counter; a load whose header
// size does not match the current build is rejected and the core resumes
// untouched. sleep_savestate holds the core asleep for the whole transfer,
// and a load pulses reset_ss/BUS_rst before the restore writes so blocks
// start from a known state.

module savestates
#(
	parameter integer STATESIZE_PARAM = 346928,
	parameter integer SETTLECOUNT_PARAM = 16,
	parameter integer INTERNALSCOUNT_PARAM = 113,
	parameter integer SAVETYPESCOUNT_PARAM = 6,
	parameter [24:0] SAVETYPE0_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE1_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE2_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE3_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE4_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE5_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE6_OFFSET = 25'd0,
	parameter [24:0] SAVETYPE7_OFFSET = 25'd0,
	parameter integer SAVETYPE0_SIZE = 65536,
	parameter integer SAVETYPE1_SIZE = 131072,
	parameter integer SAVETYPE2_SIZE = 131072,
	parameter integer SAVETYPE3_SIZE = 16384,
	parameter integer SAVETYPE4_SIZE = 288,
	parameter integer SAVETYPE5_SIZE = 1664,
	parameter integer SAVETYPE6_SIZE = 0,
	parameter integer SAVETYPE7_SIZE = 0,
	// Bit n set = save type n's memory writes are dropped on a rewind load
	// (the reads on the save side always run, so the region walk keeps its
	// shape). See skip_ram_write_w.
	parameter [7:0] REWIND_SKIP_TYPES = 8'h00
)
(
	input  wire clk,
	input  wire reset_in,
	output reg  reset_ss = 1'b0,
	output reg  reset_delay = 1'b0,
	output reg  restore_begin = 1'b0,
	output reg  load_done = 1'b0,
	// A cartridge-backed state restores its sparse flash overlay while the
	// machine is still held in reset. The wrapper raises ready only after that
	// phase; failed aborts before any internals or RAM are broadcast.
	input  wire restore_prepare_ready_i,
	input  wire restore_prepare_failed_i,

	input  wire increaseSSHeaderCount,
	input  wire save,
	input  wire load,
	input  wire [31:0] state_size_i,
	input  wire [24:0] savetype3_size_i,
	// 1 = this request belongs to the rewind ring. Latched with the two size
	// inputs at request acceptance, so it must be valid in the same cycle as
	// the save/load pulse. Tie low if rewind is not wired up.
	input  wire is_rewind_i,
	input  integer savestate_address,
	output wire savestate_busy,
	input  wire paused,

	output reg [63:0] BUS_Din = 64'd0,
	output reg [9:0]  BUS_Adr = 10'd0,
	output reg BUS_wren = 1'b0,
	output reg BUS_rst = 1'b0,
	input  wire [63:0] BUS_Dout,

	output reg loading_savestate = 1'b0,
	output reg saving_savestate = 1'b0,
	output reg sleep_savestate = 1'b0,

	output reg [24:0] Save_RAMAddr = 25'd0,
	output reg Save_RAMRdEn = 1'b0,
	output reg Save_RAMWrEn = 1'b0,
	output reg [7:0] Save_RAMWriteData = 8'd0,
	input  wire [7:0] Save_RAMReadData,
	input  wire Save_RAMReady,
	output reg [2:0] Save_RAMType = 3'd0,

	output reg [63:0] bus_out_Din = 64'd0,
	input  wire [63:0] bus_out_Dout,
	output reg [25:0] bus_out_Adr = 26'd0,
	output reg bus_out_rnw = 1'b0,
	output reg bus_out_ena = 1'b0,
	output reg [7:0] bus_out_be = 8'hff,
	input  wire bus_out_done
);

	localparam integer HEADERCOUNT = 2;
	localparam [3:0] SAVETYPESCOUNT = SAVETYPESCOUNT_PARAM[3:0];

	localparam [3:0] ST_IDLE = 4'd0;
	localparam [3:0] ST_SAVE_WAIT_SETTLE = 4'd1;
	localparam [3:0] ST_SAVE_INTERNAL_WAIT = 4'd2;
	localparam [3:0] ST_SAVE_INTERNAL_WRITE = 4'd3;
	localparam [3:0] ST_SAVE_MEMORY_NEXT = 4'd4;
	localparam [3:0] ST_SAVE_MEMORY_READ = 4'd5;
	localparam [3:0] ST_SAVE_MEMORY_WRITE = 4'd6;
	localparam [3:0] ST_SAVE_HEADER = 4'd7;
	localparam [3:0] ST_LOAD_WAIT_SETTLE = 4'd8;
	localparam [3:0] ST_LOAD_HEADER = 4'd9;
	localparam [3:0] ST_LOAD_INTERNAL_READ = 4'd10;
	localparam [3:0] ST_LOAD_INTERNAL_WRITE = 4'd11;
	localparam [3:0] ST_LOAD_MEMORY_RESET = 4'd12;
	localparam [3:0] ST_LOAD_MEMORY_NEXT = 4'd13;
	localparam [3:0] ST_LOAD_MEMORY_READ = 4'd14;
	localparam [3:0] ST_LOAD_MEMORY_WRITE = 4'd15;

	reg [3:0] state_q = ST_IDLE;
	reg [3:0] save_type_q = 4'd0;
	integer count_q = 0;
	integer max_count_q = 0;
	integer settle_count_q = 0;
	reg [2:0] byte_index_q = 3'd0;
	reg [24:0] next_ram_addr_q = 25'd0;
	reg [3:0] memory_wait_q = 4'd0;
	reg       memory_request_issued_q = 1'b0;
	reg [31:0] header_count_q = 32'd1;
	// The header check asserts reset_ss, which unparks the machine, so the
	// load re-enters the settle wait before streaming registers. This flag
	// tells the shared settle state which side of the header it is on.
	reg header_done_q = 1'b0;
	reg [31:0] active_state_size_q = STATESIZE_PARAM[31:0];
	reg [24:0] active_savetype3_size_q = SAVETYPE3_SIZE[24:0];
	reg active_is_rewind_q = 1'b0;

	// A rewind snapshot carries these regions as an 8-byte stub. The load
	// still walks the DDR payload beat for beat to stay in step; only the RAM
	// write and its Save_RAMReady handshake are dropped.
	// Indexed through a wire because a variable bit-select of a parameter
	// upsets some Quartus versions.
	wire [7:0] rewind_skip_types_w = REWIND_SKIP_TYPES;
	wire skip_ram_write_w = active_is_rewind_q &&
		rewind_skip_types_w[save_type_q[2:0]];

	assign savestate_busy = state_q != ST_IDLE;

	function [24:0] savetype_offset;
		input [3:0] index_i;
		begin
			case (index_i)
				4'd0: savetype_offset = SAVETYPE0_OFFSET;
				4'd1: savetype_offset = SAVETYPE1_OFFSET;
				4'd2: savetype_offset = SAVETYPE2_OFFSET;
				4'd3: savetype_offset = SAVETYPE3_OFFSET;
				4'd4: savetype_offset = SAVETYPE4_OFFSET;
				4'd5: savetype_offset = SAVETYPE5_OFFSET;
				4'd6: savetype_offset = SAVETYPE6_OFFSET;
				4'd7: savetype_offset = SAVETYPE7_OFFSET;
				default: savetype_offset = 25'd0;
			endcase
		end
	endfunction

	function integer savetype_size;
		input [3:0] index_i;
		input [24:0] size3_i;
		begin
			case (index_i)
				4'd0: savetype_size = SAVETYPE0_SIZE;
				4'd1: savetype_size = SAVETYPE1_SIZE;
				4'd2: savetype_size = SAVETYPE2_SIZE;
				4'd3: savetype_size = {7'd0, size3_i};
				4'd4: savetype_size = SAVETYPE4_SIZE;
				4'd5: savetype_size = SAVETYPE5_SIZE;
				4'd6: savetype_size = SAVETYPE6_SIZE;
				4'd7: savetype_size = SAVETYPE7_SIZE;
				default: savetype_size = 0;
			endcase
		end
	endfunction

	/* verilator lint_off UNUSEDSIGNAL */
	function [25:0] addr26;
		input integer value_i;
		begin
			addr26 = value_i[25:0];
		end
	endfunction
	/* verilator lint_on UNUSEDSIGNAL */

	function [7:0] select_byte64;
		input [63:0] value_i;
		input [2:0] index_i;
		begin
			case (index_i)
				3'd0: select_byte64 = value_i[7:0];
				3'd1: select_byte64 = value_i[15:8];
				3'd2: select_byte64 = value_i[23:16];
				3'd3: select_byte64 = value_i[31:24];
				3'd4: select_byte64 = value_i[39:32];
				3'd5: select_byte64 = value_i[47:40];
				3'd6: select_byte64 = value_i[55:48];
				default: select_byte64 = value_i[63:56];
			endcase
		end
	endfunction

	function [63:0] replace_byte64;
		input [63:0] word_i;
		input [2:0] index_i;
		input [7:0] value_i;
		begin
			case (index_i)
				3'd0: replace_byte64 = {word_i[63:8], value_i};
				3'd1: replace_byte64 = {word_i[63:16], value_i, word_i[7:0]};
				3'd2: replace_byte64 = {word_i[63:24], value_i, word_i[15:0]};
				3'd3: replace_byte64 = {word_i[63:32], value_i, word_i[23:0]};
				3'd4: replace_byte64 = {word_i[63:40], value_i, word_i[31:0]};
				3'd5: replace_byte64 = {word_i[63:48], value_i, word_i[39:0]};
				3'd6: replace_byte64 = {word_i[63:56], value_i, word_i[47:0]};
				default: replace_byte64 = {value_i, word_i[55:0]};
			endcase
		end
	endfunction

	always @(posedge clk) begin
		bus_out_ena <= 1'b0;
		BUS_wren <= 1'b0;
		BUS_rst <= 1'b0;
		reset_ss <= 1'b0;
		reset_delay <= 1'b0;
		restore_begin <= 1'b0;
		load_done <= 1'b0;
		bus_out_be <= 8'hff;

		if (memory_wait_q != 4'hf) begin
			memory_wait_q <= memory_wait_q + 4'd1;
		end

		case (state_q)
			ST_IDLE: begin
				save_type_q <= 4'd0;
				Save_RAMRdEn <= 1'b0;
				Save_RAMWrEn <= 1'b0;
				memory_request_issued_q <= 1'b0;
				header_done_q <= 1'b0;
				if (reset_in) begin
					reset_delay <= 1'b1;
					reset_ss <= 1'b1;
					BUS_rst <= 1'b1;
				end else if (save) begin
					active_state_size_q <= state_size_i;
					active_savetype3_size_q <= savetype3_size_i;
					active_is_rewind_q <= is_rewind_i;
					state_q <= ST_SAVE_WAIT_SETTLE;
					settle_count_q <= 0;
					sleep_savestate <= 1'b1;
					header_count_q <= header_count_q + 32'd1;
				end else if (load) begin
					active_state_size_q <= state_size_i;
					active_savetype3_size_q <= savetype3_size_i;
					active_is_rewind_q <= is_rewind_i;
					state_q <= ST_LOAD_WAIT_SETTLE;
					settle_count_q <= 0;
					sleep_savestate <= 1'b1;
				end
			end

			ST_SAVE_WAIT_SETTLE: begin
				if (!paused) begin
					settle_count_q <= 0;
				end else if (settle_count_q < SETTLECOUNT_PARAM) begin
					settle_count_q <= settle_count_q + 1;
				end else begin
					state_q <= ST_SAVE_INTERNAL_WAIT;
					bus_out_Adr <= addr26(savestate_address + HEADERCOUNT);
					bus_out_rnw <= 1'b0;
					BUS_Adr <= 10'd0;
					count_q <= 1;
					saving_savestate <= 1'b1;
				end
			end

			ST_SAVE_INTERNAL_WAIT: begin
				bus_out_Din <= BUS_Dout;
				bus_out_ena <= 1'b1;
				state_q <= ST_SAVE_INTERNAL_WRITE;
			end

			ST_SAVE_INTERNAL_WRITE: begin
				if (bus_out_done) begin
					bus_out_Adr <= bus_out_Adr + 26'd2;
					if (count_q < INTERNALSCOUNT_PARAM) begin
						state_q <= ST_SAVE_INTERNAL_WAIT;
						count_q <= count_q + 1;
						BUS_Adr <= BUS_Adr + 10'd1;
					end else begin
						state_q <= ST_SAVE_MEMORY_NEXT;
						count_q <= 8;
					end
				end
			end

			ST_SAVE_MEMORY_NEXT: begin
				if (save_type_q < SAVETYPESCOUNT) begin
					state_q <= ST_SAVE_MEMORY_READ;
					byte_index_q <= 3'd0;
					count_q <= 8;
					max_count_q <= savetype_size(save_type_q, active_savetype3_size_q);
					Save_RAMAddr <= savetype_offset(save_type_q);
					Save_RAMRdEn <= 1'b1;
					memory_wait_q <= 4'd0;
					Save_RAMType <= save_type_q[2:0];
				end else begin
					state_q <= ST_SAVE_HEADER;
					bus_out_Adr <= addr26(savestate_address);
					bus_out_Din <= {active_state_size_q, header_count_q};
					bus_out_ena <= 1'b1;
					if (!increaseSSHeaderCount) begin
						bus_out_be <= 8'hf0;
					end
				end
			end

			ST_SAVE_MEMORY_READ: begin
				if ((memory_wait_q >= 4'd7) && Save_RAMReady) begin
					Save_RAMRdEn <= 1'b0;
					bus_out_Din <= replace_byte64(bus_out_Din, byte_index_q, Save_RAMReadData);
					if (byte_index_q < 3'd7) begin
						byte_index_q <= byte_index_q + 3'd1;
						Save_RAMAddr <= Save_RAMAddr + 25'd1;
						Save_RAMRdEn <= 1'b1;
						memory_wait_q <= 4'd0;
					end else begin
						state_q <= ST_SAVE_MEMORY_WRITE;
						bus_out_ena <= 1'b1;
					end
				end
			end

			ST_SAVE_MEMORY_WRITE: begin
				if (bus_out_done) begin
					bus_out_Adr <= bus_out_Adr + 26'd2;
					if (count_q < max_count_q) begin
						state_q <= ST_SAVE_MEMORY_READ;
						byte_index_q <= 3'd0;
						count_q <= count_q + 8;
						Save_RAMAddr <= Save_RAMAddr + 25'd1;
						Save_RAMRdEn <= 1'b1;
						memory_wait_q <= 4'd0;
					end else begin
						save_type_q <= save_type_q + 4'd1;
						state_q <= ST_SAVE_MEMORY_NEXT;
					end
				end
			end

			ST_SAVE_HEADER: begin
				if (bus_out_done) begin
					state_q <= ST_IDLE;
					saving_savestate <= 1'b0;
					sleep_savestate <= 1'b0;
				end
			end

			ST_LOAD_WAIT_SETTLE: begin
				if (header_done_q && restore_prepare_failed_i) begin
						// Overlay application can still fail after the header check.
						// Abort before touching CPU or RAM state.
					state_q <= ST_IDLE;
					header_done_q <= 1'b0;
					loading_savestate <= 1'b0;
					sleep_savestate <= 1'b0;
					Save_RAMRdEn <= 1'b0;
					Save_RAMWrEn <= 1'b0;
				end else if (header_done_q && !restore_prepare_ready_i) begin
						// The destructive reset was one cycle; the parent holds a
						// non-destructive restore hold while the overlay is applied.
						// Stretching reset here would erase earlier restore writes.
					settle_count_q <= 0;
				end else if (!paused) begin
					settle_count_q <= 0;
				end else if (settle_count_q < SETTLECOUNT_PARAM) begin
					settle_count_q <= settle_count_q + 1;
				end else if (!header_done_q) begin
					state_q <= ST_LOAD_HEADER;
					bus_out_Adr <= addr26(savestate_address);
					bus_out_rnw <= 1'b1;
					bus_out_ena <= 1'b1;
				end else begin
						// Second pass: the machine has been reset and re-parked,
						// so the broadcast can start.
					state_q <= ST_LOAD_INTERNAL_READ;
					bus_out_Adr <= addr26(savestate_address + HEADERCOUNT);
					bus_out_rnw <= 1'b1;
					bus_out_ena <= 1'b1;
					BUS_Adr <= 10'd0;
					count_q <= 1;
				end
			end

			ST_LOAD_HEADER: begin
				if (bus_out_done) begin
					if (bus_out_Dout[63:32] == active_state_size_q) begin
						header_count_q <= bus_out_Dout[31:0];
						// reset_ss starts this cycle and stays asserted until the
						// wrapper reports reset-held preparation complete; only
						// then does the machine re-park and the broadcast start.
						state_q <= ST_LOAD_WAIT_SETTLE;
						header_done_q <= 1'b1;
						settle_count_q <= 0;
						loading_savestate <= 1'b1;
						restore_begin <= 1'b1;
						reset_ss <= 1'b1;
						BUS_rst <= 1'b1;
					end else begin
						state_q <= ST_IDLE;
						sleep_savestate <= 1'b0;
					end
				end
			end

			ST_LOAD_INTERNAL_READ: begin
				if (bus_out_done) begin
					state_q <= ST_LOAD_INTERNAL_WRITE;
					BUS_Din <= bus_out_Dout;
					BUS_wren <= 1'b1;
				end
			end

			ST_LOAD_INTERNAL_WRITE: begin
				bus_out_Adr <= bus_out_Adr + 26'd2;
				if (count_q < INTERNALSCOUNT_PARAM) begin
					state_q <= ST_LOAD_INTERNAL_READ;
					count_q <= count_q + 1;
					bus_out_ena <= 1'b1;
					BUS_Adr <= BUS_Adr + 10'd1;
				end else begin
					state_q <= ST_LOAD_MEMORY_RESET;
				end
			end

			ST_LOAD_MEMORY_RESET: begin
				state_q <= ST_LOAD_MEMORY_NEXT;
				count_q <= 8;
				memory_wait_q <= 4'd0;
			end

			ST_LOAD_MEMORY_NEXT: begin
				if (memory_wait_q >= 4'd7) begin
					if (save_type_q < SAVETYPESCOUNT) begin
						state_q <= ST_LOAD_MEMORY_READ;
						count_q <= 8;
						max_count_q <= savetype_size(save_type_q, active_savetype3_size_q);
						next_ram_addr_q <= savetype_offset(save_type_q);
						byte_index_q <= 3'd0;
						bus_out_ena <= 1'b1;
						Save_RAMType <= save_type_q[2:0];
					end else begin
						state_q <= ST_IDLE;
						loading_savestate <= 1'b0;
						sleep_savestate <= 1'b0;
						load_done <= 1'b1;
					end
				end
			end

			ST_LOAD_MEMORY_READ: begin
				if (bus_out_done) begin
					state_q <= ST_LOAD_MEMORY_WRITE;
					memory_wait_q <= 4'd0;
					memory_request_issued_q <= 1'b0;
				end
			end

			ST_LOAD_MEMORY_WRITE: begin
				if (!skip_ram_write_w && !memory_request_issued_q && (memory_wait_q >= 4'd7)) begin
					next_ram_addr_q <= next_ram_addr_q + 25'd1;
					Save_RAMAddr <= next_ram_addr_q;
					Save_RAMWrEn <= 1'b1;
					Save_RAMWriteData <= select_byte64(bus_out_Dout, byte_index_q);
					memory_wait_q <= 4'd0;
					memory_request_issued_q <= 1'b1;
				end else if (skip_ram_write_w || (memory_request_issued_q && Save_RAMReady)) begin
					Save_RAMWrEn <= 1'b0;
					memory_request_issued_q <= 1'b0;
					memory_wait_q <= 4'd0;
					if (byte_index_q < 3'd7) begin
						byte_index_q <= byte_index_q + 3'd1;
					end else begin
						bus_out_Adr <= bus_out_Adr + 26'd2;
						if (count_q < max_count_q) begin
							state_q <= ST_LOAD_MEMORY_READ;
							count_q <= count_q + 8;
							byte_index_q <= 3'd0;
							bus_out_ena <= 1'b1;
						end else begin
							save_type_q <= save_type_q + 4'd1;
							state_q <= ST_LOAD_MEMORY_NEXT;
						end
					end
				end
			end
		endcase
	end

endmodule
