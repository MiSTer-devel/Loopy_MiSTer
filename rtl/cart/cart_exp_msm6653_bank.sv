// Copyright (c) 2026 Jamie Blanks

// Phrase-code to speech-data mapper for cart_exp_msm6653. The chip's 544 Kbit
// mask ROM is undumped, so a replacement bank in this container format stands
// in for it:
//
//   0x000  128 entries x 4 bytes: 24-bit big-endian byte offset, then a pad
//          byte.  Offset 0 means the slot is empty.
//   0x200  phrases, 16-byte aligned, each one block-framed:
//
//              phrase := block* 0x00
//              block  := count, `count` data bytes
//
// The count bytes are framing. Fed to the decoder they inject two large
// positive nibbles and slam the step pointer to maximum once per block, a
// loud pop every 32 ms at 8 kHz. So the chip asks for a linear byte offset
// into the phrase and never sees a count byte or the padding between phrases.
//
// A phrase ends at its 0x00 terminator. The lookup walks the blocks to total
// the payload, one read per block. One mapper context per voice, since both
// channels can stream at once from different phrases.
//
// A real mask ROM dump would replace this module alone; the chip model stays.

module cart_exp_msm6653_bank (
	input  logic        clk_sys,

	// Size of the bank image in bytes, a bound on a malformed phrase.
	input  logic [23:0] bank_size,

	// Chip side
	input  logic [6:0]  phrase_req,
	input  logic        phrase_ch,
	input  logic        phrase_req_stb,
	output logic [23:0] phrase_start,
	output logic [23:0] phrase_len,
	output logic [2:0]  phrase_rate,
	output logic        phrase_valid,

	input  logic        chip_rom_req,
	input  logic        chip_rom_ch,
	input  logic [23:0] chip_rom_addr,
	output logic [7:0]  chip_rom_data,
	output logic        chip_rom_ready,

	// Memory side
	output logic        rom_req,
	output logic [23:0] rom_addr,
	input  logic [7:0]  rom_data,
	input  logic        rom_ready
);

	localparam int MAX_BLOCKS = 4096;

	localparam logic [1:0] S_IDLE  = 2'd0;
	localparam logic [1:0] S_ENTRY = 2'd1;
	localparam logic [1:0] S_WALK  = 2'd2;
	localparam logic [1:0] S_DONE  = 2'd3;

	// Power-up values. No reset: Cyclone V registers come up holding these,
	// and until a lookup has run there is nothing valid to serve.
	/* verilator lint_off PROCASSINIT */
	logic [1:0] state   = S_IDLE;
	logic       m_count = 1'b0;
	logic [1:0] m_valid = 2'b00;
	/* verilator lint_on PROCASSINIT */

	logic [6:0]  code;
	logic        look_ch;
	logic [1:0]  byte_sel;
	logic [15:0] acc;          // the two high offset bytes, being assembled
	logic [23:0] look_pos;     // next count byte to read
	logic [23:0] look_len;     // payload bytes totalled so far
	logic [23:0] look_start;   // first count byte of the phrase
	logic [7:0]  look_first;   // count of the first block
	logic [12:0] blocks;

	// Per-voice mapper: which linear payload offset is resolved, where it sits
	// in the image, and how much of the current block is left.
	logic [23:0] m_base  [0:1];   // physical address of the phrase's first count byte
	logic [23:0] m_off   [0:1];   // linear payload offset currently resolved
	logic [23:0] m_phys  [0:1];   // physical address of that byte
	logic [7:0]  m_left  [0:1];   // data bytes remaining in this block, this one included
	logic [7:0]  m_first [0:1];   // count of the first block, for a cheap rewind
	                              // m_count / m_valid are declared above

	wire        ch   = chip_rom_ch;
	wire [23:0] want = chip_rom_addr - m_base[ch];

	wire look_busy = (state == S_ENTRY) || (state == S_WALK);
	wire aligned   = m_valid[ch] && (want == m_off[ch]);

	wire [23:0] look_addr = (state == S_ENTRY) ? ({15'd0, code, 2'd0} + {22'd0, byte_sel})
	                                           : look_pos;

	assign rom_req        = look_busy | m_count | chip_rom_req;
	// Only present an address the mapper has resolved.
	assign rom_addr       = look_busy    ? look_addr
	                      : !m_valid[ch] ? 24'd0
	                      : m_count      ? (m_phys[ch] + 24'd1)
	                      :                m_phys[ch];
	assign chip_rom_data  = rom_data;
	assign chip_rom_ready = chip_rom_req & ~look_busy & ~m_count & aligned & rom_ready;

	assign phrase_valid   = (state == S_DONE) && !phrase_req_stb;
	assign phrase_start   = look_start;
	assign phrase_len     = look_len;
	assign phrase_rate    = 3'd3;   // 8 kHz; the container does not say

	always_ff @(posedge clk_sys) begin
		if (phrase_req_stb) begin
			state      <= S_ENTRY;
			code       <= phrase_req;
			look_ch    <= phrase_ch;
			byte_sel   <= 2'd0;
			acc        <= 16'd0;
			look_len   <= 24'd0;
			look_first <= 8'd0;
			blocks     <= 13'd0;
			m_valid[phrase_ch] <= 1'b0;
		end else begin
			case (state)
			S_ENTRY:
				if (rom_ready) begin
					acc <= {acc[7:0], rom_data};
					if (byte_sel == 2'd2) begin
						look_start <= {acc, rom_data};
						look_pos   <= {acc, rom_data};
						byte_sel   <= 2'd0;
						// Offset 0 is an empty slot: not a user phrase.
						state <= ({acc, rom_data} == 24'd0) ? S_DONE : S_WALK;
					end else begin
						byte_sel <= byte_sel + 2'd1;
					end
				end

			S_WALK:
				if (rom_ready) begin
					if (rom_data == 8'd0 || blocks == 13'(MAX_BLOCKS)
					    || look_pos >= bank_size) begin
						// Terminator, or a phrase that does not terminate.
						state <= S_DONE;
						m_base [look_ch] <= look_start;
						m_off  [look_ch] <= 24'd0;
						m_phys [look_ch] <= look_start + 24'd1;
						m_left [look_ch] <= look_first;
						m_first[look_ch] <= look_first;
						m_valid[look_ch] <= (look_len != 24'd0);
					end else begin
						if (blocks == 13'd0) look_first <= rom_data;
						look_len <= look_len + {16'd0, rom_data};
						look_pos <= look_pos + {16'd0, rom_data} + 24'd1;
						blocks   <= blocks + 13'd1;
					end
				end

			default: ;
			endcase

			// Mapper.  The chip walks a phrase forwards one byte at a time and
			// only ever jumps back to its start, on a repeat.
			if (m_count) begin
				if (rom_ready && !look_busy) begin
					m_count      <= 1'b0;
					m_left [ch]  <= rom_data;
					m_phys [ch]  <= m_phys[ch] + 24'd2;
					m_off  [ch]  <= m_off[ch] + 24'd1;
				end
			end else if (chip_rom_req && !look_busy && !aligned && m_valid[ch]) begin
				if (want < m_off[ch]) begin
					m_off [ch] <= 24'd0;
					m_phys[ch] <= m_base[ch] + 24'd1;
					m_left[ch] <= m_first[ch];
				end else if (m_left[ch] > 8'd1) begin
					m_phys[ch] <= m_phys[ch] + 24'd1;
					m_left[ch] <= m_left[ch] - 8'd1;
					m_off [ch] <= m_off[ch] + 24'd1;
				end else begin
					// Last byte of the block: the next one is a count byte.
					m_count <= 1'b1;
				end
			end
		end
	end

endmodule
