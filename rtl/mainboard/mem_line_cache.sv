// Copyright (c) 2026 Jamie Blanks

// Block-RAM line cache between a memory chip model and the shared SDRAM. A
// miss raises `stall_o`, which freezes the CPU-domain clock enables so the CPU
// just sees a longer bus cycle.
//
//   state    |<--------- one CPU state --------->|
//   clk_sys  __/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\__/‾‾\
//   address  ==X       stable for the whole state      X==
//   port A       ^ RAM samples    ^ q, rdata_o and hit are valid here
//   strobe   ______________/‾‾‾‾‾‾‾‾‾‾‾\______   falls half way in
//
// Port A runs free off the address pins and `rdata_o`/`stall_o` are
// combinational from q, so the address must hold for the whole access. Lines
// carry per-byte dirty bits, so a write allocates without a fill and a later
// fill merges under the mask. Victims drain through a one-line writeback
// buffer; the next lines are prefetched after a read miss.
//
// For a savestate hold `ss_flush_i` and wait for `ss_idle_o`; reset clears
// every tag.

module mem_line_cache #(
	parameter int  ADDR_W    = 19,      // byte address width of the device
	parameter int  INDEX_W   = 9,       // 2**INDEX_W lines of eight bytes
	parameter bit  READ_ONLY = 1'b0
) (
	input  wire                clk_i,
	input  wire                rst_i,

	// Chip side, 16-bit words, big-endian: the even byte is D15-D8.
	input  wire [ADDR_W-1:1]   addr_i,     // stable while acc_i is high
	input  wire                acc_i,      // an access is on the pins now
	input  wire                we_i,
	input  wire [1:0]          be_i,       // {even byte, odd byte}
	input  wire [15:0]         wdata_i,
	input  wire                commit_i,   // one cycle: take the write
	output wire [15:0]         rdata_o,
	output wire                stall_o,

	// Savestate: write every dirty byte back to SDRAM and say when it is done.
	input  wire                ss_flush_i,
	output wire                ss_idle_o,

	// Line port to ext_mem_bridge.
	output reg                 mem_req_o,
	output reg                 mem_we_o,
	output reg  [ADDR_W-1:3]   mem_line_o,
	output reg  [63:0]         mem_din_o,
	output reg  [7:0]          mem_be_o,
	input  wire [63:0]         mem_dout_i,
	input  wire                mem_busy_i,
	input  wire                mem_done_i
);
	localparam int LINE_W = ADDR_W - 3;
	localparam int TAG_W  = LINE_W - INDEX_W;
	localparam int META_W = TAG_W + 9;

	// ---------------------------------------------------------- addressing
	wire [LINE_W-1:0]  line = addr_i[ADDR_W-1:3];
	wire [INDEX_W-1:0] idx  = line[INDEX_W-1:0];
	wire [TAG_W-1:0]   tag  = line[LINE_W-1:INDEX_W];
	wire [1:0]         woff = addr_i[2:1];

	// The byte lanes this access touches, placed in the line.
	wire [1:0] lane = {be_i[0], be_i[1]};
	wire [7:0] need = {6'd0, lane} << {woff, 1'b0};
	wire [63:0] wdata64 = {48'd0, wdata_i[7:0], wdata_i[15:8]} << {woff, 4'd0};

	// ------------------------------------------------------------- the RAMs
	// Port A runs free off the pins. Port B belongs to the fill machine and to
	// the one-cycle write commit.
	wire [63:0]       data_qa, data_qb;
	wire [META_W-1:0] meta_qa, meta_qb;

	reg                pb_we_data, pb_we_meta;
	reg [INDEX_W-1:0]  pb_addr;
	reg [7:0]          pb_be;
	reg [63:0]         pb_data;
	reg [META_W-1:0]   pb_meta;

	cache_ram_dp_be #(.ADDR_WIDTH (INDEX_W), .DATA_WIDTH (64)) u_data (
		.clk_i     (clk_i),
		.addr_a_i  (idx),
		.wren_a_i  (1'b0),
		.be_a_i    (8'd0),
		.wdata_a_i (64'd0),
		.q_a_o     (data_qa),
		.addr_b_i  (pb_addr),
		.wren_b_i  (pb_we_data),
		.be_b_i    (pb_be),
		.wdata_b_i (pb_data),
		.q_b_o     (data_qb)
	);

	cache_ram_dp #(.ADDR_WIDTH (INDEX_W), .DATA_WIDTH (META_W)) u_meta (
		.clk_i     (clk_i),
		.addr_a_i  (idx),
		.wren_a_i  (1'b0),
		.wdata_a_i ({META_W{1'b0}}),
		.q_a_o     (meta_qa),
		.addr_b_i  (pb_addr),
		.wren_b_i  (pb_we_meta),
		.wdata_b_i (pb_meta),
		.q_b_o     (meta_qb)
	);

	// The offset that produced this port A read, so the word mux matches it.
	reg [1:0] woff_q;
	// The RAM read takes a cycle, so the first cycle of an access carries the
	// answer to the previous address. Nothing may be judged until the address
	// has stood still for a cycle.
	reg [ADDR_W-1:1] addr_q;
	// Port B wrote the line port A was reading, so this cycle's q means
	// nothing. One cycle of stall and the next read is clean.
	reg q_stale;
	// A fill just installed: for one cycle port A still shows the old line,
	// so the CPU is answered from the merged line held here instead.
	reg        fwd_q;
	reg [63:0] fwd_data;
	// Port B's registered write lands on the next edge. The old read must
	// already be blocked then, before q_stale reports the RAM collision.
	wire q_pending = (pb_we_data || pb_we_meta) && (pb_addr == idx);
	always @(posedge clk_i) begin
		woff_q  <= woff;
		addr_q  <= addr_i;
		q_stale <= (pb_we_data || pb_we_meta) && (pb_addr == idx);
	end

	wire [TAG_W-1:0] a_tag     = meta_qa[META_W-1:9];
	wire             a_fetched = meta_qa[8];
	wire [7:0]       a_dirty   = meta_qa[7:0];

	// An empty line reads back as zero, so its tag field is zero and nothing
	// is fetched or dirty: a read of it misses, and a write into it is right
	// if the tag matches, because a tag match is what says this is that line.
	wire tag_match = (a_tag == tag);
	wire read_ok   = a_fetched || ((a_dirty & need) == need);
	wire addr_ok   = (addr_q == addr_i);
	wire match     = tag_match && (we_i || read_ok);
	wire acc_v     = acc_i && addr_ok;
	wire hit       = (match && !q_stale && !q_pending && addr_ok)
	               || (fwd_q && acc_v && !we_i);

	wire [63:0] rd_line = fwd_q ? fwd_data : data_qa;
	assign rdata_o = {rd_line[{woff_q, 4'd0} +: 8],
	                  rd_line[{woff_q, 4'd0} + 5'd8 +: 8]};

	// ------------------------------------------------------- the fill machine
	localparam [3:0] S_INIT = 4'd0, S_IDLE = 4'd1, S_WB = 4'd2,
	                 S_FILL = 4'd3, S_SETTLE = 4'd4, S_PF_PROBE = 4'd5,
	                 S_PF_FILL = 4'd6, S_PF_WRITE = 4'd7,
	                 S_FL_PROBE = 4'd8, S_FL_WB = 4'd9,
	                 S_WAIT_Q = 4'd10, S_FL_ADV = 4'd11;

	reg [3:0]         st;
	// Port B's read takes a cycle like port A's, so a probe waits one state
	// after setting the address before it may look at meta_qb.
	reg [3:0]         next_st;
	reg [INDEX_W:0]   init_cnt;
	reg [INDEX_W-1:0] fl_cnt;
	reg               flush_done;
	reg               busy;
	// A request is out on the line port. The bridge drops busy in the same
	// cycle it pulses done, so without this the request state would see an
	// idle port and issue the transfer a second time.
	reg               issued;
	reg               wr_pend;       // a write commit that could not be taken
	reg [LINE_W-1:0]  pf_line;
	reg               pf_armed;
	reg               pf_cancel;
	// A read miss arms a run of prefetches, not just one line: with the row
	// open at the SDRAM each is a CAS-only fill on an otherwise idle port,
	// and a sequential read stream then misses once at its head.
	localparam [1:0]  PF_RUN = 2'd3;
	reg [1:0]         pf_left;
	reg [63:0]        fill_data;
	reg [7:0]         keep;          // dirty bytes a merge must not overwrite
	// The fetched line merged under those dirty bytes, as it is installed.
	wire [63:0] keep_mask = {{8{keep[7]}}, {8{keep[6]}}, {8{keep[5]}}, {8{keep[4]}},
	                         {8{keep[3]}}, {8{keep[2]}}, {8{keep[1]}}, {8{keep[0]}}};
	wire [63:0] fill_merged = (data_qa & keep_mask) | (mem_dout_i & ~keep_mask);

	// The one-line writeback buffer. A victim goes in here and the CPU carries
	// on; the drain happens whenever the line port is otherwise idle.
	reg               wb_full;
	reg [LINE_W-1:0]  wb_line;
	reg [63:0]        wb_data;
	reg [7:0]         wb_be;

	// The access being serviced. The pins hold still while the CPU is frozen,
	// so these are the same values; naming them makes the machine readable.
	reg [LINE_W-1:0]  m_line;
	reg [INDEX_W-1:0] m_idx;
	reg [TAG_W-1:0]   m_tag;
	reg               m_we;
	reg [7:0]         m_need;
	reg [63:0]        m_wdata;

	// A miss that needs the writeback buffer has to wait for it: either this
	// line's slot has dirty bytes of its own to displace, or the line being
	// fetched is the one still waiting to be written back.
	wire needs_evict = !tag_match && !READ_ONLY && (a_dirty != 8'd0);
	wire wb_blocks   = wb_full && (needs_evict || (wb_line == line));

	// A write to a resident line goes straight in. One whose slot holds nothing
	// dirty goes straight in too, as a fresh line: this tag, no bytes fetched,
	// these bytes dirty. Only a write whose slot has dirty bytes to displace is
	// remembered until the fill machine has made room.
	wire wr_want  = !READ_ONLY && ((commit_i && we_i) || wr_pend);
	// If the slot's old line has dirty bytes they go into the writeback buffer
	// in the same cycle, which wb_blocks guarantees is free; a write that finds
	// the buffer busy waits for the fill machine.
	wire wr_fresh = !READ_ONLY && commit_i && we_i && acc_v && !q_stale && !q_pending
	                && !tag_match && !wb_blocks && !wr_pend;
	reg  wr_fresh_q;
	wire cpu_write = (wr_want && hit) || wr_fresh;
	wire miss_now  = acc_v && !match && !q_stale && !q_pending && !wr_fresh;

	// The fresh write's own access is not held: not on the commit cycle, and
	// not on the next one while the write lands. we_i and the unchanged
	// address keep this from ever masking a read.
	wire wr_open = we_i && (wr_fresh || wr_fresh_q);
	assign stall_o  = busy || wr_pend || (acc_v && !hit && !wr_open) || (st == S_INIT);
	assign ss_idle_o = (st == S_IDLE) && !wb_full
	                   && (!ss_flush_i || flush_done);

	wire [TAG_W-1:0] b_tag   = meta_qb[META_W-1:9];
	wire [7:0]       b_dirty = meta_qb[7:0];
	wire             b_fetched = meta_qb[8];
	wire [TAG_W-1:0] pf_tag  = pf_line[LINE_W-1:INDEX_W];

	always @(posedge clk_i) begin
		if (rst_i) begin
			st         <= S_INIT;
			next_st    <= S_IDLE;
			wb_full    <= 1'b0;
			wb_line    <= '0;
			wb_data    <= 64'd0;
			wb_be      <= 8'd0;
			init_cnt   <= '0;
			fl_cnt     <= '0;
			flush_done <= 1'b0;
			busy       <= 1'b0;
			issued     <= 1'b0;
			wr_pend    <= 1'b0;
			wr_fresh_q <= 1'b0;
			mem_req_o  <= 1'b0;
			mem_we_o   <= 1'b0;
			mem_line_o <= '0;
			mem_din_o  <= 64'd0;
			mem_be_o   <= 8'd0;
			pb_we_data <= 1'b0;
			pb_we_meta <= 1'b0;
			pb_addr    <= '0;
			pb_be      <= 8'd0;
			pb_data    <= 64'd0;
			pb_meta    <= '0;
			pf_armed   <= 1'b0;
			pf_cancel  <= 1'b0;
			pf_line    <= '0;
			pf_left    <= 2'd0;
			keep       <= 8'd0;
			fill_data  <= 64'd0;
			fwd_q      <= 1'b0;
			fwd_data   <= 64'd0;
			m_line     <= '0;
			m_idx      <= '0;
			m_tag      <= '0;
			m_we       <= 1'b0;
			m_need     <= 8'd0;
			m_wdata    <= 64'd0;
		end else begin
			mem_req_o  <= 1'b0;
			pb_we_data <= 1'b0;
			pb_we_meta <= 1'b0;
			wr_fresh_q <= wr_fresh;
			fwd_q      <= 1'b0;

			if (commit_i && we_i && !READ_ONLY && !hit && !wr_fresh) wr_pend <= 1'b1;
			if (!ss_flush_i) flush_done <= 1'b0;

			// The write commit owns port B for its one cycle; the fill machine
			// gives way to it.
			if (cpu_write) begin
				pb_addr    <= idx;
				pb_we_data <= 1'b1;
				pb_be      <= need;
				pb_data    <= wdata64;
				pb_we_meta <= 1'b1;
				pb_meta    <= tag_match ? {tag, a_fetched, a_dirty | need}
				                        : {tag, 1'b0, need};
				wr_pend    <= 1'b0;
				if (wr_fresh && needs_evict) begin
					wb_full <= 1'b1;
					wb_line <= {a_tag, idx};
					wb_data <= data_qa;
					wb_be   <= a_dirty;
				end
				// A write can change the victim or redirect its metadata probe.
				// Keep that write; an outstanding prefetch may finish but not install.
				if ((idx == pf_line[INDEX_W-1:0])
				    || (st == S_WAIT_Q) || (st == S_PF_PROBE))
					pf_cancel <= 1'b1;
			end

			case (st)
			// Clear the tags. Block RAM powers up zeroed, but a core reset is
			// not a power-up and a stale dirty line would later be written
			// back over something else.
			S_INIT: begin
				pb_addr    <= init_cnt[INDEX_W-1:0];
				pb_we_meta <= 1'b1;
				pb_meta    <= '0;
				init_cnt   <= init_cnt + 1'b1;
				if (init_cnt[INDEX_W]) st <= S_IDLE;
			end

			S_IDLE: begin
				if (miss_now && !wb_blocks) begin
					busy    <= 1'b1;
					m_line  <= line;
					m_idx   <= idx;
					m_tag   <= tag;
					m_we    <= we_i;
					m_need  <= need;
					m_wdata <= wdata64;
					keep    <= tag_match ? a_dirty : 8'd0;
					if (needs_evict) begin
						// Into the buffer; the CPU only waits for the fill.
						wb_full <= 1'b1;
						wb_line <= {a_tag, idx};
						wb_data <= data_qa;
						wb_be   <= a_dirty;
					end
					if (we_i && !READ_ONLY)
						st <= S_SETTLE;              // a write needs no fetch
					else
						st <= S_FILL;
				end else if (wb_full) begin
					st <= S_WB;                      // drain, then take the miss
				end else if (ss_flush_i && !flush_done) begin
					fl_cnt  <= '0;
					pb_addr <= '0;
					next_st <= S_FL_PROBE;
					st      <= S_WAIT_Q;
				end else if (pf_armed && !cpu_write) begin
					pf_cancel <= 1'b0;
					pb_addr <= pf_line[INDEX_W-1:0];
					next_st <= S_PF_PROBE;
					st      <= S_WAIT_Q;
				end
			end

			// Draining the writeback buffer. The CPU is only here if it asked
			// for something the buffer was in the way of.
			S_WB: begin
				if (issued && mem_done_i) begin
					issued  <= 1'b0;
					wb_full <= 1'b0;
					st      <= S_IDLE;
				end else if (!issued && !mem_busy_i) begin
					issued     <= 1'b1;
					mem_req_o  <= 1'b1;
					mem_we_o   <= 1'b1;
					mem_line_o <= wb_line;
					mem_din_o  <= wb_data;
					mem_be_o   <= wb_be;
				end
			end

			S_FILL: begin
				if (issued && mem_done_i) begin
					// Install now and answer the CPU next cycle from the
					// merged line while port A still shows the old one.
					issued     <= 1'b0;
					pb_addr    <= m_idx;
					pb_we_data <= 1'b1;
					pb_we_meta <= 1'b1;
					pb_be      <= ~keep;
					pb_data    <= mem_dout_i;
					pb_meta    <= {m_tag, 1'b1, keep};
					fwd_data   <= fill_merged;
					fwd_q      <= 1'b1;
					busy       <= 1'b0;
					pf_armed   <= 1'b1;
					pf_line    <= m_line + 1'b1;
					pf_left    <= PF_RUN;
					st         <= S_IDLE;
				end else if (!issued && !mem_busy_i) begin
					issued     <= 1'b1;
					mem_req_o  <= 1'b1;
					mem_we_o   <= 1'b0;
					mem_line_o <= m_line;
					mem_be_o   <= 8'd0;
				end
			end

			// Install what was fetched or written. q_stale covers the cycle
			// where port A reads the line port B is writing, so the CPU is
			// still held for one more cycle after this.
			S_SETTLE: begin
				if (!cpu_write) begin
					pb_addr    <= m_idx;
					pb_we_data <= 1'b1;
					pb_we_meta <= 1'b1;
					if (m_we && !READ_ONLY) begin
						pb_be   <= m_need;
						pb_data <= m_wdata;
						pb_meta <= {m_tag, 1'b0, keep | m_need};
						wr_pend <= 1'b0;
					end else begin
						pb_be   <= ~keep;
						pb_data <= fill_data;
						pb_meta <= {m_tag, 1'b1, keep};
					end
					busy     <= 1'b0;
					// Only a read miss arms the prefetch. A stream of writes
					// is going to overwrite the next line anyway, and fetching
					// it would cost an SDRAM read for nothing.
					pf_armed <= !(m_we && !READ_ONLY);
					pf_line  <= m_line + 1'b1;
					st       <= S_IDLE;
				end
			end

			// Background fetch of the line after the one that just missed.
			S_PF_PROBE: begin
				pf_armed <= 1'b0;
				// meta_qb belongs to the address presented last cycle.
				if (pf_cancel || cpu_write)
					st <= S_IDLE;                    // the probe no longer belongs to the victim
				else if (((b_tag == pf_tag) && (b_fetched || (b_dirty != 8'd0)))
				         || (b_dirty != 8'd0)) begin
					// Already here, or somebody else's and dirty: on to the
					// next line of the run.
					if (pf_left > 2'd1) begin
						pf_left  <= pf_left - 2'd1;
						pf_line  <= pf_line + 1'b1;
						pf_armed <= 1'b1;
					end
					st <= S_IDLE;
				end else
					st <= S_PF_FILL;
			end

			S_PF_FILL: begin
				if (issued && mem_done_i) begin
					issued    <= 1'b0;
					fill_data <= mem_dout_i;
					st        <= S_PF_WRITE;
				end else if (issued) begin
					// Let a transfer that is already out finish rather than
					// leaving its done pulse for the next state to swallow.
				end else if (pf_cancel || miss_now) begin
					st <= S_IDLE;                    // a demand access wants the port
				end else if (!mem_busy_i) begin
					issued     <= 1'b1;
					mem_req_o  <= 1'b1;
					mem_we_o   <= 1'b0;
					mem_line_o <= pf_line;
					mem_be_o   <= 8'd0;
				end
			end

			S_PF_WRITE: begin
				if (pf_cancel) begin
					st <= S_IDLE;
				end else if (!cpu_write) begin
					pb_addr    <= pf_line[INDEX_W-1:0];
					pb_we_data <= 1'b1;
					pb_be      <= 8'hFF;
					pb_data    <= fill_data;
					pb_we_meta <= 1'b1;
					pb_meta    <= {pf_tag, 1'b1, 8'd0};
					if (pf_left > 2'd1) begin
						pf_left  <= pf_left - 2'd1;
						pf_line  <= pf_line + 1'b1;
						pf_armed <= 1'b1;
					end
					st         <= S_IDLE;
				end
			end

			S_WAIT_Q: st <= next_st;

			// Savestate flush: walk the tags and put every dirty byte back.
			S_FL_PROBE: begin
				if (!READ_ONLY && (b_dirty != 8'd0)) st <= S_FL_WB;
				else                                 st <= S_FL_ADV;
			end

			S_FL_WB: begin
				if (!issued && !mem_busy_i) begin
					issued     <= 1'b1;
					mem_req_o  <= 1'b1;
					mem_we_o   <= 1'b1;
					mem_line_o <= {b_tag, fl_cnt};
					mem_din_o  <= data_qb;
					mem_be_o   <= b_dirty;
				end else if (issued && mem_done_i) begin
					issued     <= 1'b0;
					// pb_addr still points at this line, so the clear lands
					// here and the walk moves on afterwards.
					pb_we_meta <= 1'b1;
					pb_meta    <= {b_tag, b_fetched, 8'd0};
					st         <= S_FL_ADV;
				end
			end

			S_FL_ADV: begin
				if (fl_cnt == {INDEX_W{1'b1}}) begin
					flush_done <= 1'b1;
					st         <= S_IDLE;
				end else begin
					fl_cnt  <= fl_cnt + 1'b1;
					pb_addr <= fl_cnt + 1'b1;
					next_st <= S_FL_PROBE;
					st      <= S_WAIT_Q;
				end
			end

			default: st <= S_IDLE;
			endcase
		end
	end

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, init_cnt[INDEX_W-1:0]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
endmodule
