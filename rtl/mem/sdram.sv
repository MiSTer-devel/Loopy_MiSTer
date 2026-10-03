`timescale 1ns/1ps
`default_nettype none

// MiSTer SDR SDRAM controller: three busy/request ports, open-page, with
// bounded priority. `reset` is asynchronous and busy stays high until the
// JEDEC init sequence completes.
//
// Client contract: `req` is a level. Hold it until `busy` rises, which is the
// only acknowledgement; `busy` also covers init, refresh, close timing and a
// higher-priority port, so a request that misses its window is simply not
// taken. `ready` reports completion only: it falls on acceptance and rises
// with `dout` valid or on the final write beat. The three `dout` buses share
// one read register, so latch `dout` on the edge `ready` rises; it holds only
// until another port's read data lands. Priority is p0, p1, p2; a port passed
// over PRIORITY_PATIENCE times takes the next grant.
//
// PORTx_SIZE: 0 = 8, 1 = 16, 2 = 32, 3 = 64 bits. Data rides the 64-bit
// buses; narrow reads are zero-extended, narrow writes use the low bits of
// `din`, byte_en covers successive bytes of `din`, and for an 8-bit port
// addr[0] picks the lane. `addr` is a byte address: BA = addr[25:24],
// row = addr[23:11], column = addr[10:1], low bits masked to the port width.
//
// MiSTer boards have no DQM pins, so DQML/DQMH alias SDRAM_A[11]/A[12]. All
// pin outputs come from registers, and read capture allows one extra cycle
// for that path. SDRAM_TIMING_GRADE 7 = -7TIN 143 MHz part (default),
// 6 = -6TIN 166 MHz part; CL2 up to 100 MHz, CL3 above.
module sdram #(
	parameter int unsigned CLK_FREQ_HZ = 100_000_000,
	parameter int unsigned SDRAM_TIMING_GRADE = 7,
	// Cycles from the input cell's capture edge to the edge that consumes the
	// beat. Re-measure after a clock rate change; see CAS_READ_CYCLES.
	parameter int unsigned DQ_CAPTURE_PIPELINE = 2,
	parameter int unsigned PORT0_SIZE = 1,
	parameter int unsigned PORT1_SIZE = 1,
	parameter int unsigned PORT2_SIZE = 1,
	parameter bit AUTO_REFRESH = 1'b1,
	// Refresh while `refresh_window` is high, and elsewhere only once two are
	// owed, instead of at the first idle slot.
	parameter bit REFRESH_WINDOW = 1'b0
) (
	input  wire        clk,
	input  wire        reset,

	input  wire        refresh,
	// A span where the busiest port will not ask, for REFRESH_WINDOW.
	input  wire        refresh_window,

	// Debug override for the read capture pipeline. 0 uses the parameter; any
	// other value replaces it. Sampled only while the controller is idle, so it
	// can be driven from a menu bit and swept on hardware without a rebuild.
	input  wire [2:0]  dq_pipe_sel,

	input  wire        p0_req,
	input  wire        p0_we,
	input  wire [25:0] p0_addr,
	input  wire [63:0] p0_din,
	input  wire [7:0]  p0_byte_en,
	output logic [63:0] p0_dout,
	output wire        p0_busy,
	output logic       p0_ready,

	input  wire        p1_req,
	input  wire        p1_we,
	input  wire [25:0] p1_addr,
	input  wire [63:0] p1_din,
	input  wire [7:0]  p1_byte_en,
	output logic [63:0] p1_dout,
	output wire        p1_busy,
	output logic       p1_ready,

	input  wire        p2_req,
	input  wire        p2_we,
	input  wire [25:0] p2_addr,
	input  wire [63:0] p2_din,
	input  wire [7:0]  p2_byte_en,
	output logic [63:0] p2_dout,
	output wire        p2_busy,
	output logic       p2_ready,

	output wire        SDRAM_CLK,
	output logic       SDRAM_CKE,
	output logic [12:0] SDRAM_A,
	output logic [1:0] SDRAM_BA,
	inout  wire [15:0] SDRAM_DQ,
	output wire        SDRAM_DQML,
	output wire        SDRAM_DQMH,
	output logic       SDRAM_nCS,
	output logic       SDRAM_nCAS,
	output logic       SDRAM_nRAS,
	output logic       SDRAM_nWE
);

	localparam logic [3:0] CMD_DESELECT      = 4'b1111;
	localparam logic [3:0] CMD_NOP           = 4'b0111;
	localparam logic [3:0] CMD_ACTIVE        = 4'b0011;
	localparam logic [3:0] CMD_READ          = 4'b0101;
	localparam logic [3:0] CMD_WRITE         = 4'b0100;
	localparam logic [3:0] CMD_PRECHARGE     = 4'b0010;
	localparam logic [3:0] CMD_AUTO_REFRESH  = 4'b0001;
	localparam logic [3:0] CMD_LOAD_MODE     = 4'b0000;

	// Elaboration-time timing helper; the division constant-folds into
	// localparams.
	function automatic [15:0] cycles_for_ns;
		input [31:0] freq_hz;
		input [31:0] ns;
		reg [63:0] num;
		reg [63:0] cycles;
		begin
			num = ({32'd0, freq_hz} * {32'd0, ns}) + 64'd999_999_999;
			cycles = num / 64'd1_000_000_000;
			if (cycles < 64'd1) cycles_for_ns = 16'd1;
			else cycles_for_ns = cycles[15:0];
		end
	endfunction

	// After issuing a command in the current enabled cycle, wait this many
	// additional enabled cycles before the next dependent command/action.
	function automatic [15:0] count_after_command;
		input [15:0] cycles;
		begin
			if (cycles > 16'd1) count_after_command = cycles - 16'd1;
			else count_after_command = 16'd0;
		end
	endfunction

	function automatic [15:0] max16;
		input [15:0] a;
		input [15:0] b;
		begin
			if (a > b) max16 = a;
			else max16 = b;
		end
	endfunction

	function automatic [2:0] size_to_beats;
		input [1:0] size;
		begin
			unique case (size)
				2'd2: size_to_beats = 3'd2;
				2'd3: size_to_beats = 3'd4;
				default: size_to_beats = 3'd1;
			endcase
		end
	endfunction

	function automatic [2:0] max3;
		input [2:0] a;
		input [2:0] b;
		input [2:0] c;
		reg [2:0] m;
		begin
			m = a;
			if (b > m) m = b;
			if (c > m) m = c;
			max3 = m;
		end
	endfunction

	function automatic [2:0] burst_code;
		input [2:0] beats;
		begin
			unique case (beats)
				3'd4: burst_code = 3'b010;
				3'd2: burst_code = 3'b001;
				default: burst_code = 3'b000;
			endcase
		end
	endfunction

	function automatic [25:0] align_addr;
		input [1:0] size;
		input [25:0] addr;
		begin
			unique case (size)
				2'd0: align_addr = addr;
				2'd1: align_addr = {addr[25:1], 1'b0};
				2'd2: align_addr = {addr[25:2], 2'b00};
				default: align_addr = {addr[25:3], 3'b000};
			endcase
		end
	endfunction

	// The write data and byte enables are shifted down a beat at a time, so
	// each beat is always the low word and the DQM pins see two flop bits.
	function automatic [15:0] write_beat_data;
		input [1:0] size;
		input [15:0] data;
		begin
			if (size == 2'd0) write_beat_data = {data[7:0], data[7:0]};
			else write_beat_data = data;
		end
	endfunction

	function automatic [1:0] write_beat_dqm;
		input [1:0] size;
		input addr_bit;
		input [1:0] byte_en;
		begin
			if (size == 2'd0) begin
				if (addr_bit) write_beat_dqm = {~byte_en[0], 1'b1};
				else write_beat_dqm = {1'b1, ~byte_en[0]};
			end else begin
				write_beat_dqm = ~byte_en;
			end
		end
	endfunction

	// Column-command word: DQM for a write beat, low for reads, A10 low so
	// the row stays open. Also serves a single-bank precharge, which only
	// needs A10 low.
	function automatic [12:0] col_word;
		input we;
		input [1:0] size;
		input [10:0] addr;
		input [1:0] byte_en;
		begin
			col_word = {we ? write_beat_dqm(size, addr[0], byte_en) : 2'b00, 1'b0, addr[10:1]};
		end
	endfunction

	function automatic [63:0] format_read_data;
		input [1:0] size;
		input addr_bit;
		input [63:0] data;
		begin
			unique case (size)
				2'd0: begin
					if (addr_bit) format_read_data = {56'd0, data[15:8]};
					else format_read_data = {56'd0, data[7:0]};
				end
				2'd1: format_read_data = {48'd0, data[15:0]};
				2'd2: format_read_data = {32'd0, data[31:0]};
				default: format_read_data = data;
			endcase
		end
	endfunction

	function automatic [7:0] cache_read_byte;
		input addr_bit;
		input [15:0] data;
		begin
			if (addr_bit) cache_read_byte = data[15:8];
			else cache_read_byte = data[7:0];
		end
	endfunction

	localparam logic [2:0] PORT0_BEATS = size_to_beats(PORT0_SIZE[1:0]);
	localparam logic [2:0] PORT1_BEATS = size_to_beats(PORT1_SIZE[1:0]);
	localparam logic [2:0] PORT2_BEATS = size_to_beats(PORT2_SIZE[1:0]);

	// Program the chip for the widest read port. Write-burst mode stays
	// disabled: wider writes are adjacent single WRITE commands.
	localparam logic [2:0] MODE_BEATS = max3(PORT0_BEATS, PORT1_BEATS, PORT2_BEATS);
	localparam logic [2:0] MODE_BURST = burst_code(MODE_BEATS);
	// AS4C32M16SB supports CL2 or CL3. For both -6TIN and -7TIN, CL2 is
	// valid through 100 MHz; above that CL3 is programmed.
	localparam logic [2:0] CAS_LATENCY = (CLK_FREQ_HZ <= 100_000_000) ? 3'd2 : 3'd3;
	localparam logic MODE_WRITE_SINGLE = 1'b1;
	localparam logic [12:0] MODE_REG = {3'b000, MODE_WRITE_SINGLE, 2'b00, CAS_LATENCY, 1'b0, MODE_BURST};
	// Read capture pipeline, in state-clock cycles between the edge that
	// registers the READ command and the edge on which the state machine may
	// consume the first beat:
	//
	//   T0        the controller registers READ
	//   T0+0.5    SDRAM samples it on the rising pin edge (SDRAM_CLK is ~clk)
	//   T0+0.5+CL SDRAM launches beat 0
	//   T0+1.5+CL the input cell captures it on the next rising pin edge,
	//             which is a falling clk edge, after tAC and before tOH
	//   T0+2+CL   the state machine samples the capture flop
	//
	// DQ_CAPTURE_PIPELINE is 2 while the round trip through the forwarded
	// clock, tAC and the input cell fits one clock period. At a new clock
	// rate, reads one beat early mean it needs raising by one.
	localparam logic [15:0] CAS_READ_CYCLES =
		{13'd0, CAS_LATENCY} + DQ_CAPTURE_PIPELINE[15:0];


	// AS4C32M16SB timing values in nanoseconds, rounded up to cycles below.
	// Grade 7 is the default -7TIN/-7TCN board, 6 the faster -6TIN; any other
	// value falls back to the -7 profile.
	localparam logic TIMING_GRADE_6 = (SDRAM_TIMING_GRADE == 6);
	localparam logic [31:0] T_RCD_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RP_NS = TIMING_GRADE_6 ? 32'd18 : 32'd21;
	localparam logic [31:0] T_RFC_NS = TIMING_GRADE_6 ? 32'd60 : 32'd63;
	localparam logic [31:0] T_WR_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;
	localparam logic [31:0] T_MRD_NS = TIMING_GRADE_6 ? 32'd12 : 32'd14;
	localparam logic [31:0] T_RAS_NS = 32'd42;

`ifndef SYNTHESIS
	// Elaboration guards: catch a clock the part cannot sustain at the chosen
	// CAS latency here rather than as intermittent read corruption on a board.
	localparam int unsigned CL2_MAX_HZ = 100_000_000;
	localparam int unsigned CL3_MAX_HZ = TIMING_GRADE_6 ? 166_000_000 : 143_000_000;
	initial begin
		if ((CAS_LATENCY == 3'd2) && (CLK_FREQ_HZ > CL2_MAX_HZ)) begin
			$error("sdram: CL2 is not valid above %0d Hz", CL2_MAX_HZ);
		end
		if ((CAS_LATENCY == 3'd3) && (CLK_FREQ_HZ > CL3_MAX_HZ)) begin
			$error("sdram: CLK_FREQ_HZ %0d exceeds the CL3 ceiling %0d for the configured speed grade",
				CLK_FREQ_HZ, CL3_MAX_HZ);
		end
		if ((SDRAM_TIMING_GRADE != 6) && (SDRAM_TIMING_GRADE != 7)) begin
			$display("sdram: SDRAM_TIMING_GRADE %0d is unknown; using the conservative -7 profile",
				SDRAM_TIMING_GRADE);
		end
	end
`endif

	// Keep these as localparams so synthesis sees constants.
	// CKE stays low for the full 200 us datasheet startup interval; one NOP
	// cycle after CKE rises is sufficient before PRECHARGE ALL.
	localparam logic [15:0] T_POWER_WAIT = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd200_000);
	localparam logic [15:0] T_CKE_NOP = 16'd1;
	localparam logic [15:0] T_RCD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RCD_NS);
	localparam logic [15:0] T_RP = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RP_NS);
	localparam logic [15:0] T_RFC = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RFC_NS);
	localparam logic [15:0] T_MRD = cycles_for_ns(CLK_FREQ_HZ[31:0], T_MRD_NS);
	localparam logic [15:0] T_RAS = cycles_for_ns(CLK_FREQ_HZ[31:0], T_RAS_NS);
	localparam logic [15:0] T_WR = cycles_for_ns(CLK_FREQ_HZ[31:0], T_WR_NS);
	// Slightly below the 64 ms / 8192-row average of 7.8125 us.
	localparam logic [15:0] T_REFI = cycles_for_ns(CLK_FREQ_HZ[31:0], 32'd7_800);
	localparam logic [15:0] T_REFI_COUNT = count_after_command(T_REFI);
	// One count below the interval: `auto_refresh_due_q` is registered, so it
	// is decided from the counter value one cycle before the one it reports
	// on. The flag still rises on the cycle `refresh_ctr_q` reaches T_REFI_COUNT.
	localparam logic [15:0] T_REFI_PRECOUNT =
		(T_REFI_COUNT > 16'd0) ? (T_REFI_COUNT - 16'd1) : 16'd0;
	// Counter widths. The power-up interval gets its own register; every other
	// wait is a handful of cycles, and a narrow counter keeps the zero tests in
	// front of the command and address pin registers a few gates wide.
	localparam logic [15:0] WAIT_MAX =
		max16(max16(T_RFC, T_RCD), max16(T_RP, max16(T_MRD, CAS_READ_CYCLES)));
	localparam int unsigned WAIT_W = 6;

	// The value actually in force, and the wait count derived from it. Both are
	// registered, so selecting a different pipeline costs the command path
	// nothing: ST_RCD still loads a register.
	logic [2:0] dq_pipe_q;
	logic [WAIT_W-1:0] cas_wait_q;
	wire [15:0] cas_read_cycles_w = {13'd0, CAS_LATENCY} + {13'd0, dq_pipe_q};

`ifndef SYNTHESIS
	// The wait values scale with the clock; catch a counter that cannot hold
	// its own interval here rather than as a truncated wait on a board.
	initial begin
		if (WAIT_MAX > ((16'd1 << WAIT_W) - 16'd1)) begin
			$error("sdram: WAIT_W is too narrow for a %0d cycle wait", WAIT_MAX);
		end
	end
`endif

	// Refresh is tracked as small debt. Low-priority ports are blocked first
	// as debt rises; p0 is blocked only when refresh becomes more urgent.
	// With a window, one or two owed is normal between windows, so blocking
	// starts later; p0 still forces one before eight are owed.
	localparam logic [4:0] REFRESH_BLOCK_P2 = REFRESH_WINDOW ? 5'd3 : 5'd1;
	localparam logic [4:0] REFRESH_BLOCK_P1 = REFRESH_WINDOW ? 5'd4 : 5'd2;
	localparam logic [4:0] REFRESH_BLOCK_P0 = REFRESH_WINDOW ? 5'd6 : 5'd4;
	localparam logic [4:0] REFRESH_DEBT_MAX = 5'd15;

	function automatic [4:0] add_refresh_debt;
		input [4:0] debt;
		input [1:0] add_count;
		reg [5:0] sum;
		begin
			sum = {1'b0, debt} + {4'd0, add_count};
			if (sum > {1'b0, REFRESH_DEBT_MAX}) add_refresh_debt = REFRESH_DEBT_MAX;
			else add_refresh_debt = sum[4:0];
		end
	endfunction

	function automatic [4:0] consume_refresh_debt;
		input [4:0] debt;
		begin
			if (debt != 5'd0) consume_refresh_debt = debt - 5'd1;
			else consume_refresh_debt = 5'd0;
		end
	endfunction

	typedef enum logic [4:0] {
		ST_POWER_WAIT,
		ST_NOP_WAIT,
		ST_INIT_TRP,
		ST_INIT_RFC1,
		ST_INIT_RFC2,
		ST_INIT_MRD,
		ST_IDLE,
		ST_RCD,
		ST_READ_LATENCY,
		ST_READ_BEATS,
		ST_READ_DRAIN,
		ST_WRITE_BEATS,
		ST_RAS_WAIT,
		ST_DECIDE,
		ST_PRE,
		ST_PRE_ALL,
		ST_REFRESH_WAIT
	} state_e;

	state_e state_q;
	logic init_done_q;
	logic [WAIT_W-1:0] wait_q;
	// Mirrors wait_q == 0 so the command issued at the end of a wait reads
	// one flop instead of a compare. Every write to wait_q updates both.
	logic wait_zero_q;
	logic [15:0] init_wait_q;
	logic [1:0] active_port_q;
	logic [1:0] active_size_q;
	logic active_we_q;
	logic [25:0] active_addr_q;
	logic [63:0] active_din_q;
	logic [7:0] active_byte_en_q;
	logic [2:0] active_beats_q;
	logic [2:0] beat_q;
	logic [63:0] read_data_q;
	logic client_done_q;

	// Open page. Each bank remembers the row it holds open; a row stays open
	// until another row in the same bank needs it or a refresh closes them
	// all, which is how the FPM DRAM this serves behaves under RAS-down and
	// CBR refresh. The per-bank countdown says when that bank may legally be
	// precharged again: tRAS after its ACTIVATE, tWR after its last write.
	logic [3:0] bank_open_q;
	logic [12:0] bank_row_q [0:3];
	logic [WAIT_W-1:0] bank_pre_wait_q [0:3];
	// The arbitration is run every clock from the held request inputs and
	// registered: which port would be taken, its aligned address, and
	// whether its row is open or its bank holds another row. A request that
	// was already up last clock is then accepted from these flops alone, so
	// the port select, the address and the ACTIVATE decision reach the pins
	// without the arbitration in front of them. A request seen for the first
	// time is accepted on the live arbitration, touches no pin, and decides
	// next clock from the same flops, which by then describe it.
	logic       grant_valid_q;
	logic [2:0] grant_q;
	logic [1:0] grant_port_q;
	logic [25:0] grant_addr_q;
	logic       grant_hit_q;
	logic       grant_open_q;
	// Refresh decided a clock ahead for the same reason. A request that
	// arrives on the clock an idle-slot refresh was decided waits it out.
	logic       refresh_go_q;
	// Every command's address is known when the one before it issues, so it
	// is staged a clock ahead and the pins copy this one register. Between
	// transfers the stage holds the row the next accept would open; a column
	// command stages the beat after it; a precharge stages the row to reopen.
	logic [12:0] a_stage_q;
	logic [1:0]  ba_stage_q;
	// Whether each bank may be precharged this clock, registered a clock
	// ahead so the precharge decision is a flop output at the pins.
	logic [3:0] bank_pre_ok_q;
	logic pre_done_q;

	logic pending_valid_q;
	logic [1:0] pending_port_q;
	logic [1:0] pending_size_q;
	logic pending_we_q;
	logic [25:0] pending_addr_q;
	logic [63:0] pending_din_q;
	logic [7:0] pending_byte_en_q;
	logic [2:0] pending_beats_q;

	// One 16-bit word cache per port for 8-bit reads. A hit on either byte
	// of the cached word completes through a one-cycle busy pulse without
	// spending an SDRAM command slot.
	logic [24:0] cache0_tag_q;
	logic [15:0] cache0_data_q;
	logic cache0_valid_q;
	logic [24:0] cache1_tag_q;
	logic [15:0] cache1_data_q;
	logic cache1_valid_q;
	logic [24:0] cache2_tag_q;
	logic [15:0] cache2_data_q;
	logic cache2_valid_q;
	// A cache hit answers from its own byte so the shared read register is
	// left to the transfer that may still be landing.
	logic p0_from_cache_q, p1_from_cache_q, p2_from_cache_q;
	logic [7:0] p0_cache_byte_q, p1_cache_byte_q, p2_cache_byte_q;

	// Fixed priority on its own lets a client that holds req high keep a lower
	// port off the bus indefinitely; p2 carries the wave ROM, so audio is the
	// first thing that would go quiet. Each lower port counts the acceptance
	// windows it was passed over for while requesting, and once it runs out of
	// patience it takes the next grant ahead of the ports above it.
	localparam logic [3:0] PRIORITY_PATIENCE = 4'd8;
	logic [3:0] starve1_q;
	logic [3:0] starve2_q;
	logic promote_valid_q;
	logic [1:0] promote_port_q;

	logic [15:0] refresh_ctr_q;
	logic auto_refresh_due_q;
	logic [4:0] refresh_debt_q;
	logic refresh_pending_q;
	logic refresh_behind_q;
	logic refresh_window_q;
	logic refresh_block_p0_q;
	logic refresh_block_p1_q;
	logic refresh_block_p2_q;
	logic refresh_old_q;

	logic [15:0] dq_out_q;
	logic dq_oe_q;
	wire [15:0] dq_in;
	wire [15:0] dq_aligned_w;

	logic p0_busy_q;
	logic p1_busy_q;
	logic p2_busy_q;
	logic p0_accept_i;
	logic p1_accept_i;
	logic p2_accept_i;
	logic p0_cache_done_q;
	logic p1_cache_done_q;
	logic p2_cache_done_q;

	wire p0_cache_hit = p0_req && !p0_we && (PORT0_SIZE[1:0] == 2'd0) && cache0_valid_q && (cache0_tag_q == p0_addr[25:1]);
	wire p1_cache_hit = p1_req && !p1_we && (PORT1_SIZE[1:0] == 2'd0) && cache1_valid_q && (cache1_tag_q == p1_addr[25:1]);
	wire p2_cache_hit = p2_req && !p2_we && (PORT2_SIZE[1:0] == 2'd0) && cache2_valid_q && (cache2_tag_q == p2_addr[25:1]);
	wire [25:0] p0_aligned_addr = align_addr(PORT0_SIZE[1:0], p0_addr);
	wire [25:0] p1_aligned_addr = align_addr(PORT1_SIZE[1:0], p1_addr);
	wire [25:0] p2_aligned_addr = align_addr(PORT2_SIZE[1:0], p2_addr);

	// `auto_refresh_due_q` registers the interval compare to keep it off the
	// head of the longest path: refresh compare -> debt add -> port blocking
	// -> command select -> SDRAM_A, which crosses the die to the pins.
	wire refresh_edge_due = init_done_q && refresh && !refresh_old_q;
	wire [1:0] refresh_add_count = {1'b0, auto_refresh_due_q} + {1'b0, refresh_edge_due};
	wire [4:0] refresh_debt_with_add = add_refresh_debt(refresh_debt_q, refresh_add_count);
	wire [4:0] refresh_debt_after_service = consume_refresh_debt(refresh_debt_with_add);
	wire refresh_pending = refresh_pending_q;

	// Same value as count_after_command, returned already narrowed so the
	// truncation is explicit and not a bit-select on a call result.
	function automatic [WAIT_W-1:0] wait_after;
		input [15:0] cycles;
		reg [15:0] full;
		begin
			full = count_after_command(cycles);
			wait_after = full[WAIT_W-1:0];
		end
	endfunction

	wire idle_accept = init_done_q && (state_q == ST_IDLE) && !pending_valid_q;
	wire queue_accept = init_done_q && client_done_q && !pending_valid_q && (state_q != ST_IDLE) && (state_q != ST_REFRESH_WAIT);
	wire accept_window = idle_accept || queue_accept;
	wire pending_hit_w = bank_open_q[pending_addr_q[25:24]]
		&& (bank_row_q[pending_addr_q[25:24]] == pending_addr_q[23:11]);
	wire refresh_block_p0 = refresh_block_p0_q;
	wire refresh_block_p1 = refresh_block_p1_q;
	wire refresh_block_p2 = refresh_block_p2_q;
	// A promoted port only holds the higher ones back while it could take the
	// grant itself. The busy term stops a client that keeps req high across
	// its own transfer from blocking everyone; the refresh term stops a
	// refresh-blocked promoted port from deadlocking the controller.
	wire promote_p1 = promote_valid_q && (promote_port_q == 2'd1)
		&& !p1_busy_q && !refresh_block_p1;
	wire promote_p2 = promote_valid_q && (promote_port_q == 2'd2)
		&& !p2_busy_q && !refresh_block_p2;

	wire refresh_idle_slot = !p0_req && !p1_req && !p2_req;
	wire refresh_blocks_requested_port = (refresh_block_p0 && p0_req) ||
		(refresh_block_p1 && !p0_req && p1_req) ||
		(refresh_block_p2 && !p0_req && !p1_req && p2_req);
	wire refresh_service_now = refresh_pending && (REFRESH_WINDOW
		? ((refresh_window_q && !p0_req) || (refresh_idle_slot && refresh_behind_q)
		   || refresh_blocks_requested_port)
		: (refresh_idle_slot || refresh_blocks_requested_port));

	// The output DDIO cell drives an exact inverted copy of clk to the pad. The
	// input DDIO cell's low-phase result samples DQ on the next rising SDRAM
	// pin edge, after the preceding beat has had the full tAC interval to arrive.
`ifdef SYNTHESIS
	altddio_out #(
		.extend_oe_disable("OFF"),
		.intended_device_family("Cyclone V"),
		.invert_output("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_out"),
		.oe_reg("UNREGISTERED"),
		.power_up_high("OFF"),
		.width(1)
	) u_sdram_clock (
		.datain_h(1'b0),
		.datain_l(1'b1),
		.outclock(clk),
		.dataout(SDRAM_CLK),
		.aclr(1'b0),
		.aset(1'b0),
		.oe(1'b1),
		.outclocken(1'b1),
		.sclr(1'b0),
		.sset(1'b0)
	);

	altddio_in #(
		.intended_device_family("Cyclone V"),
		.invert_input_clocks("OFF"),
		.lpm_hint("UNUSED"),
		.lpm_type("altddio_in"),
		.power_up_high("OFF"),
		.width(16)
	) u_sdram_dq_capture (
		.datain(dq_in),
		.inclock(clk),
		.inclocken(1'b1),
		.aclr(1'b0),
		.aset(1'b0),
		.sclr(1'b0),
		.sset(1'b0),
		.dataout_h(),
		.dataout_l(dq_aligned_w)
	);
`else
	// Simulation only: one capture register on the falling state-clock edge,
	// which is the rising SDRAM pin edge. A value captured there is already
	// stable at the state machine's next rising edge, which is what makes the
	// pipeline constant 2.
	reg [15:0] dq_capture_sim_q = 16'd0;

	assign SDRAM_CLK = ~clk;
	assign dq_aligned_w = dq_capture_sim_q;

	always @(posedge SDRAM_CLK) begin
		dq_capture_sim_q <= dq_in;
	end
`endif

	// DQM outputs are exact aliases of A11/A12, matching the board wiring.
	assign SDRAM_DQML = SDRAM_A[11];
	assign SDRAM_DQMH = SDRAM_A[12];

	// Data and OE are both registered so Quartus can place the final stage in
	// the SDRAM IOE.
	assign dq_in = SDRAM_DQ;
	assign SDRAM_DQ = dq_oe_q ? dq_out_q : 16'hZZZZ;

	assign p0_dout = p0_from_cache_q ? {56'd0, p0_cache_byte_q}
	               : format_read_data(PORT0_SIZE[1:0], active_addr_q[0], read_data_q);
	assign p1_dout = p1_from_cache_q ? {56'd0, p1_cache_byte_q}
	               : format_read_data(PORT1_SIZE[1:0], active_addr_q[0], read_data_q);
	assign p2_dout = p2_from_cache_q ? {56'd0, p2_cache_byte_q}
	               : format_read_data(PORT2_SIZE[1:0], active_addr_q[0], read_data_q);

	// Public busy is stricter than the per-port in-flight bit: low means the
	// port can be accepted on the next rising edge, so a one-cycle request
	// pulse is never lost to refresh, close timing or a higher-priority port.
	assign p0_busy = p0_busy_q || !p0_accept_i;
	assign p1_busy = p1_busy_q || !p1_accept_i;
	assign p2_busy = p2_busy_q || !p2_accept_i;

	// An idle port can be accepted into the controller or into the one-entry
	// pending slot after client-visible completion. Urgent refresh debt blocks
	// lower-priority ports first; cache hits stay free because they need no
	// SDRAM command. `pX_ok` is the port's own eligibility without its
	// request, which is what busy reports on an idle bus.
	wire p0_ok = !p0_busy_q && !(promote_p1 && p1_req) && !(promote_p2 && p2_req)
		&& (!refresh_block_p0 || p0_cache_hit);
	wire p1_ok = !p1_busy_q && (!p0_req || promote_p1) && !(promote_p2 && p2_req)
		&& (!refresh_block_p1 || p1_cache_hit);
	wire p2_ok = !p2_busy_q && ((!p0_req && !p1_req) || promote_p2)
		&& (!refresh_block_p2 || p2_cache_hit);
	wire p0_win = p0_req && p0_ok;
	wire p1_win = p1_req && p1_ok;
	wire p2_win = p2_req && p2_ok;
	wire [25:0] win_addr = p0_win ? p0_aligned_addr : p1_win ? p1_aligned_addr : p2_aligned_addr;

	// On an idle bus a request that has been up for a clock is taken from
	// the registered grant, the only accept that writes the pins. A request
	// seen for the first time, and any accept into the pending slot, use the
	// live arbitration and touch no pin.
	always_comb begin
		p0_accept_i = 1'b0;
		p1_accept_i = 1'b0;
		p2_accept_i = 1'b0;

		// A refresh starting in ST_IDLE outranks the grant, so the grant must
		// not be reported taken on that clock.
		if (accept_window) begin
			if (state_q == ST_IDLE && grant_valid_q && !refresh_go_q) begin
				p0_accept_i = grant_q[0] && !p0_busy_q;
				p1_accept_i = grant_q[1] && !p1_busy_q;
				p2_accept_i = grant_q[2] && !p2_busy_q;
			end else if (!(state_q == ST_IDLE && refresh_go_q)) begin
				p0_accept_i = p0_ok;
				p1_accept_i = p1_ok;
				p2_accept_i = p2_ok;
			end
		end
	end

	// States with no command left for the transfer in hand: the stage may
	// turn to the next accept.
	wire stage_idle = (state_q == ST_INIT_MRD) || (state_q == ST_IDLE) || (state_q == ST_RAS_WAIT)
		|| (state_q == ST_READ_LATENCY) || (state_q == ST_READ_BEATS) || (state_q == ST_READ_DRAIN)
		|| (state_q == ST_REFRESH_WAIT);
	wire [25:11] stage_addr = pending_valid_q ? pending_addr_q[25:11] : win_addr[25:11];

	wire grant_take = grant_valid_q && !((grant_q[0] && p0_busy_q) || (grant_q[1] && p1_busy_q)
		|| (grant_q[2] && p2_busy_q));
	wire        grant_we   = grant_q[0] ? p0_we : grant_q[1] ? p1_we : p2_we;
	wire [63:0] grant_din  = grant_q[0] ? p0_din : grant_q[1] ? p1_din : p2_din;
	wire [7:0]  grant_be   = grant_q[0] ? p0_byte_en : grant_q[1] ? p1_byte_en : p2_byte_en;
	wire [1:0]  grant_size = grant_q[0] ? PORT0_SIZE[1:0] : grant_q[1] ? PORT1_SIZE[1:0] : PORT2_SIZE[1:0];
	wire [2:0]  grant_beats = grant_q[0] ? PORT0_BEATS : grant_q[1] ? PORT1_BEATS : PORT2_BEATS;

	// Reset is asynchronous on purpose: Cyclone V SDRAM output and
	// output-enable registers cannot pack into the IOE with a synchronous
	// clear.
	always_ff @(posedge clk or posedge reset) begin
		if (reset) begin
			state_q <= ST_POWER_WAIT;
			init_done_q <= 1'b0;
			wait_q <= {WAIT_W{1'b0}};
			wait_zero_q <= 1'b1;
			wait_zero_q <= 1'b1;
			init_wait_q <= count_after_command(T_POWER_WAIT);
			active_port_q <= 2'd0;
			active_size_q <= 2'd1;
			active_we_q <= 1'b0;
			active_addr_q <= 26'd0;
			active_din_q <= 64'd0;
			active_byte_en_q <= 8'd0;
			active_beats_q <= 3'd1;
			beat_q <= 3'd0;
			read_data_q <= 64'd0;
			client_done_q <= 1'b0;
			pending_valid_q <= 1'b0;
			pending_port_q <= 2'd0;
			pending_size_q <= 2'd1;
			pending_we_q <= 1'b0;
			pending_addr_q <= 26'd0;
			pending_din_q <= 64'd0;
			pending_byte_en_q <= 8'd0;
			pending_beats_q <= 3'd1;
			cache0_valid_q <= 1'b0;
			cache1_valid_q <= 1'b0;
			cache2_valid_q <= 1'b0;
			cache0_tag_q <= 25'd0;
			cache1_tag_q <= 25'd0;
			cache2_tag_q <= 25'd0;
			cache0_data_q <= 16'd0;
			cache1_data_q <= 16'd0;
			cache2_data_q <= 16'd0;
			dq_pipe_q <= DQ_CAPTURE_PIPELINE[2:0];
			cas_wait_q <= wait_after(CAS_READ_CYCLES);
			refresh_ctr_q <= 16'd0;
			auto_refresh_due_q <= 1'b0;
			starve1_q <= 4'd0;
			starve2_q <= 4'd0;
			promote_valid_q <= 1'b0;
			promote_port_q <= 2'd0;
			bank_open_q <= 4'd0;
			for (int b = 0; b < 4; b++) begin
				bank_row_q[b] <= 13'd0;
				bank_pre_wait_q[b] <= {WAIT_W{1'b0}};
			end
			grant_valid_q <= 1'b0;
			grant_q <= 3'd0;
			grant_port_q <= 2'd0;
			grant_addr_q <= 26'd0;
			grant_hit_q <= 1'b0;
			grant_open_q <= 1'b0;
			refresh_go_q <= 1'b0;
			bank_pre_ok_q <= 4'd0;
			pre_done_q <= 1'b0;
			refresh_debt_q <= 5'd0;
			refresh_pending_q <= 1'b0;
			refresh_block_p0_q <= 1'b0;
			refresh_block_p1_q <= 1'b0;
			refresh_block_p2_q <= 1'b0;
			refresh_old_q <= 1'b0;
			refresh_behind_q <= 1'b0;
			refresh_window_q <= 1'b0;
			p0_from_cache_q <= 1'b0;
			p1_from_cache_q <= 1'b0;
			p2_from_cache_q <= 1'b0;
			p0_cache_byte_q <= 8'd0;
			p1_cache_byte_q <= 8'd0;
			p2_cache_byte_q <= 8'd0;
			p0_busy_q <= 1'b1;
			p1_busy_q <= 1'b1;
			p2_busy_q <= 1'b1;
			p0_ready <= 1'b1;
			p1_ready <= 1'b1;
			p2_ready <= 1'b1;
			p0_cache_done_q <= 1'b0;
			p1_cache_done_q <= 1'b0;
			p2_cache_done_q <= 1'b0;
			SDRAM_CKE <= 1'b0;
			SDRAM_A <= 13'd0;
			SDRAM_BA <= 2'd0;
			a_stage_q <= 13'h0400;                   // the init PRECHARGE ALL
			ba_stage_q <= 2'd0;
			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_DESELECT;
			dq_out_q <= 16'd0;
			dq_oe_q <= 1'b0;
		end else begin
			// Only track the request once refreshes can actually be served,
			// so a level raised during the power-up wait still reads as an
			// edge when init finishes instead of being swallowed by history.
			refresh_old_q <= init_done_q && refresh;
			refresh_window_q <= refresh_window;

			// Only between transfers: changing this mid-read would move the
			// capture edge of a burst already in flight.
			if (state_q == ST_IDLE) begin
				dq_pipe_q <= (dq_pipe_sel == 3'd0) ? DQ_CAPTURE_PIPELINE[2:0]
				                                   : dq_pipe_sel;
			end
			cas_wait_q <= wait_after(cas_read_cycles_w);
			if (p0_cache_done_q) begin
				p0_busy_q <= 1'b0;
				p0_ready <= 1'b1;
				p0_cache_done_q <= 1'b0;
			end
			if (p1_cache_done_q) begin
				p1_busy_q <= 1'b0;
				p1_ready <= 1'b1;
				p1_cache_done_q <= 1'b0;
			end
			if (p2_cache_done_q) begin
				p2_busy_q <= 1'b0;
				p2_ready <= 1'b1;
				p2_cache_done_q <= 1'b0;
			end

			{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= init_done_q ? CMD_NOP : CMD_DESELECT;
			dq_oe_q <= 1'b0;

			// The address and bank pins copy the stage every clock; between
			// commands they are don't-care. A[12:11] are also the DQM pins
			// and must hold through a read burst, so they move only with a
			// command.
			SDRAM_A[10:0] <= a_stage_q[10:0];
			SDRAM_BA <= ba_stage_q;
			if (stage_idle) begin
				a_stage_q <= stage_addr[23:11];
				ba_stage_q <= stage_addr[25:24];
			end

			// Patience is spent only on windows the port could have used and
			// lost to someone else. A port that is already mid-transfer still
			// holds req until busy rises, and that is not being passed over.
			if (accept_window) begin
				if (p1_accept_i) starve1_q <= 4'd0;
				else if (p1_req && !p1_busy_q && (starve1_q != PRIORITY_PATIENCE))
					starve1_q <= starve1_q + 4'd1;
				if (p2_accept_i) starve2_q <= 4'd0;
				else if (p2_req && !p2_busy_q && (starve2_q != PRIORITY_PATIENCE))
					starve2_q <= starve2_q + 4'd1;
			end
			// Registered so the override reaches the arbiter as a flop output
			// rather than a compare in front of the command and address pins.
			promote_valid_q <= (starve1_q >= PRIORITY_PATIENCE) || (starve2_q >= PRIORITY_PATIENCE);
			promote_port_q <= (starve1_q >= PRIORITY_PATIENCE) ? 2'd1 : 2'd2;
			refresh_debt_q <= refresh_debt_with_add;
			refresh_pending_q <= (refresh_debt_with_add != 5'd0);
			refresh_behind_q <= (refresh_debt_with_add >= 5'd2);
			refresh_block_p0_q <= (refresh_debt_with_add >= REFRESH_BLOCK_P0);
			refresh_block_p1_q <= (refresh_debt_with_add >= REFRESH_BLOCK_P1);
			refresh_block_p2_q <= (refresh_debt_with_add >= REFRESH_BLOCK_P2);

			if (init_done_q && AUTO_REFRESH) begin
				// Refresh debt: one per JEDEC interval plus one per external
				// `refresh` edge, both counted before a same-cycle service
				// consumes a slot. The due flag is decided from the value the
				// counter is about to take; >= recovers if it ever overshoots.
				if (auto_refresh_due_q) begin
					refresh_ctr_q <= 16'd0;
					auto_refresh_due_q <= (T_REFI_COUNT == 16'd0);
				end else begin
					refresh_ctr_q <= refresh_ctr_q + 16'd1;
					auto_refresh_due_q <= (refresh_ctr_q >= T_REFI_PRECOUNT);
				end
			end

			for (int b = 0; b < 4; b++)
				if (bank_pre_wait_q[b] != {WAIT_W{1'b0}})
					bank_pre_wait_q[b] <= bank_pre_wait_q[b] - {{(WAIT_W-1){1'b0}}, 1'b1};
			grant_valid_q <= p0_win || p1_win || p2_win;
			grant_q <= p0_win ? 3'b001 : p1_win ? 3'b010 : p2_win ? 3'b100 : 3'b000;
			grant_port_q <= p0_win ? 2'd0 : p1_win ? 2'd1 : 2'd2;
			grant_addr_q <= win_addr;
			grant_hit_q <= bank_open_q[win_addr[25:24]] && (bank_row_q[win_addr[25:24]] == win_addr[23:11]);
			grant_open_q <= bank_open_q[win_addr[25:24]];
			refresh_go_q <= refresh_service_now;
			for (int b = 0; b < 4; b++)
				bank_pre_ok_q[b] <= (bank_pre_wait_q[b] <= {{(WAIT_W-1){1'b0}}, 1'b1});

			if (queue_accept) begin
				if (p0_req && p0_accept_i && p0_cache_hit) begin
					p0_cache_byte_q <= cache_read_byte(p0_addr[0], cache0_data_q);
					p0_from_cache_q <= 1'b1;
					p0_busy_q <= 1'b1;
					p0_ready <= 1'b0;
					p0_cache_done_q <= 1'b1;
				end else if (p1_req && p1_accept_i && p1_cache_hit) begin
					p1_cache_byte_q <= cache_read_byte(p1_addr[0], cache1_data_q);
					p1_from_cache_q <= 1'b1;
					p1_busy_q <= 1'b1;
					p1_ready <= 1'b0;
					p1_cache_done_q <= 1'b1;
				end else if (p2_req && p2_accept_i && p2_cache_hit) begin
					p2_cache_byte_q <= cache_read_byte(p2_addr[0], cache2_data_q);
					p2_from_cache_q <= 1'b1;
					p2_busy_q <= 1'b1;
					p2_ready <= 1'b0;
					p2_cache_done_q <= 1'b1;
				end else if (p0_req && p0_accept_i) begin
					p0_busy_q <= 1'b1;
					p0_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd0;
					pending_size_q <= PORT0_SIZE[1:0];
					pending_we_q <= p0_we;
					pending_addr_q <= p0_aligned_addr;
					pending_din_q <= p0_din;
					pending_byte_en_q <= p0_byte_en;
					pending_beats_q <= PORT0_BEATS;
					p0_from_cache_q <= 1'b0;
					cache0_valid_q <= p0_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p0_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p0_we ? 1'b0 : cache2_valid_q;
				end else if (p1_req && p1_accept_i) begin
					p1_busy_q <= 1'b1;
					p1_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd1;
					pending_size_q <= PORT1_SIZE[1:0];
					pending_we_q <= p1_we;
					pending_addr_q <= p1_aligned_addr;
					pending_din_q <= p1_din;
					pending_byte_en_q <= p1_byte_en;
					pending_beats_q <= PORT1_BEATS;
					p1_from_cache_q <= 1'b0;
					cache0_valid_q <= p1_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p1_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p1_we ? 1'b0 : cache2_valid_q;
				end else if (p2_req && p2_accept_i) begin
					p2_busy_q <= 1'b1;
					p2_ready <= 1'b0;
					pending_valid_q <= 1'b1;
					pending_port_q <= 2'd2;
					pending_size_q <= PORT2_SIZE[1:0];
					pending_we_q <= p2_we;
					pending_addr_q <= p2_aligned_addr;
					pending_din_q <= p2_din;
					pending_byte_en_q <= p2_byte_en;
					pending_beats_q <= PORT2_BEATS;
					p2_from_cache_q <= 1'b0;
					cache0_valid_q <= p2_we ? 1'b0 : cache0_valid_q;
					cache1_valid_q <= p2_we ? 1'b0 : cache1_valid_q;
					cache2_valid_q <= p2_we ? 1'b0 : cache2_valid_q;
				end
			end

			unique case (state_q)
				ST_POWER_WAIT: begin
					// JEDEC init: hold CKE low, then CKE high with NOPs,
					// precharge all banks, issue two refreshes, load mode.
					SDRAM_CKE <= 1'b0;
					if (init_wait_q != 16'd0) begin
						init_wait_q <= init_wait_q - 16'd1;
					end else begin
						SDRAM_CKE <= 1'b1;
						state_q <= ST_NOP_WAIT;
						wait_q <= wait_after(T_CKE_NOP);
						wait_zero_q <= (wait_after(T_CKE_NOP) == {WAIT_W{1'b0}});
					end
				end

				ST_NOP_WAIT: begin
					SDRAM_CKE <= 1'b1;
					{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_NOP;
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						SDRAM_A[12:11] <= a_stage_q[12:11];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;
						a_stage_q <= MODE_REG;
						state_q <= ST_INIT_TRP;
						wait_q <= wait_after(T_RP);
						wait_zero_q <= (wait_after(T_RP) == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_TRP: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC1;
						wait_q <= wait_after(T_RFC);
						wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_RFC1: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						state_q <= ST_INIT_RFC2;
						wait_q <= wait_after(T_RFC);
						wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_RFC2: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						SDRAM_A[12:11] <= a_stage_q[12:11];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_LOAD_MODE;
						state_q <= ST_INIT_MRD;
						wait_q <= wait_after(T_MRD);
						wait_zero_q <= (wait_after(T_MRD) == {WAIT_W{1'b0}});
					end
				end

				ST_INIT_MRD: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						init_done_q <= 1'b1;
						p0_busy_q <= 1'b0;
						p1_busy_q <= 1'b0;
						p2_busy_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				ST_IDLE: begin
					if (pending_valid_q) begin
						active_port_q <= pending_port_q;
						active_size_q <= pending_size_q;
						active_we_q <= pending_we_q;
						active_addr_q <= pending_addr_q;
						active_din_q <= pending_din_q;
						active_byte_en_q <= pending_byte_en_q;
						active_beats_q <= pending_beats_q;
						beat_q <= 3'd0;
						client_done_q <= 1'b0;
						pending_valid_q <= 1'b0;
						a_stage_q <= col_word(pending_we_q, pending_size_q, pending_addr_q[10:0], pending_byte_en_q[1:0]);
						ba_stage_q <= pending_addr_q[25:24];
						if (pending_hit_w) begin
							state_q <= ST_RCD;                 // row open: column command next
							wait_q <= {WAIT_W{1'b0}};
							wait_zero_q <= 1'b1;
						end else if (bank_open_q[pending_addr_q[25:24]]) begin
							state_q <= ST_PRE;                 // another row is open here
						end else begin
							SDRAM_A[12:11] <= a_stage_q[12:11];
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
							bank_pre_wait_q[pending_addr_q[25:24]] <= T_RAS[WAIT_W-1:0];
							state_q <= ST_RCD;
							wait_q <= wait_after(T_RCD);
							wait_zero_q <= (wait_after(T_RCD) == {WAIT_W{1'b0}});
						end
					end else if (p0_req && p0_accept_i && p0_cache_hit) begin
						// Cache-hit reads use the same busy-falling
						// completion convention as SDRAM-backed reads.
						// If refresh is pending, use this free SDRAM slot.
						p0_cache_byte_q <= cache_read_byte(p0_addr[0], cache0_data_q);
					p0_from_cache_q <= 1'b1;
						p0_busy_q <= 1'b1;
						p0_ready <= 1'b0;
						p0_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							if (bank_open_q != 4'd0) begin
								a_stage_q <= 13'h0400;
								state_q <= ST_PRE_ALL;             // close every row, then refresh
							end else begin
								{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
								state_q <= ST_REFRESH_WAIT;
								wait_q <= wait_after(T_RFC);
								wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
							end
							refresh_debt_q <= refresh_debt_after_service;
							refresh_pending_q <= (refresh_debt_after_service != 5'd0);
							refresh_behind_q <= (refresh_debt_after_service >= 5'd2);
							refresh_block_p0_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P0);
							refresh_block_p1_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P1);
							refresh_block_p2_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P2);
						end
					end else if (p1_req && p1_accept_i && p1_cache_hit) begin
						p1_cache_byte_q <= cache_read_byte(p1_addr[0], cache1_data_q);
					p1_from_cache_q <= 1'b1;
						p1_busy_q <= 1'b1;
						p1_ready <= 1'b0;
						p1_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							if (bank_open_q != 4'd0) begin
								a_stage_q <= 13'h0400;
								state_q <= ST_PRE_ALL;             // close every row, then refresh
							end else begin
								{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
								state_q <= ST_REFRESH_WAIT;
								wait_q <= wait_after(T_RFC);
								wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
							end
							refresh_debt_q <= refresh_debt_after_service;
							refresh_pending_q <= (refresh_debt_after_service != 5'd0);
							refresh_behind_q <= (refresh_debt_after_service >= 5'd2);
							refresh_block_p0_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P0);
							refresh_block_p1_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P1);
							refresh_block_p2_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P2);
						end
					end else if (p2_req && p2_accept_i && p2_cache_hit) begin
						p2_cache_byte_q <= cache_read_byte(p2_addr[0], cache2_data_q);
					p2_from_cache_q <= 1'b1;
						p2_busy_q <= 1'b1;
						p2_ready <= 1'b0;
						p2_cache_done_q <= 1'b1;
						if (refresh_pending) begin
							if (bank_open_q != 4'd0) begin
								a_stage_q <= 13'h0400;
								state_q <= ST_PRE_ALL;             // close every row, then refresh
							end else begin
								{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
								state_q <= ST_REFRESH_WAIT;
								wait_q <= wait_after(T_RFC);
								wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
							end
							refresh_debt_q <= refresh_debt_after_service;
							refresh_pending_q <= (refresh_debt_after_service != 5'd0);
							refresh_behind_q <= (refresh_debt_after_service >= 5'd2);
							refresh_block_p0_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P0);
							refresh_block_p1_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P1);
							refresh_block_p2_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P2);
						end
					end else if (refresh_go_q) begin
						if (bank_open_q != 4'd0) begin
							a_stage_q <= 13'h0400;
							state_q <= ST_PRE_ALL;             // close every row, then refresh
						end else begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
							state_q <= ST_REFRESH_WAIT;
							wait_q <= wait_after(T_RFC);
							wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
						end
						refresh_debt_q <= refresh_debt_after_service;
						refresh_pending_q <= (refresh_debt_after_service != 5'd0);
						refresh_behind_q <= (refresh_debt_after_service >= 5'd2);
						refresh_block_p0_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P0);
						refresh_block_p1_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P1);
						refresh_block_p2_q <= (refresh_debt_after_service >= REFRESH_BLOCK_P2);
					end else if (grant_valid_q) begin
						// A request that was already up last clock: everything
						// about it is in the grant flops, so the row can be
						// opened on this same edge.
						if (grant_take) begin
							unique case (grant_port_q)
								2'd0: begin p0_busy_q <= 1'b1; p0_ready <= 1'b0; p0_from_cache_q <= 1'b0; end
								2'd1: begin p1_busy_q <= 1'b1; p1_ready <= 1'b0; p1_from_cache_q <= 1'b0; end
								default: begin p2_busy_q <= 1'b1; p2_ready <= 1'b0; p2_from_cache_q <= 1'b0; end
							endcase
							active_port_q <= grant_port_q;
							active_size_q <= grant_size;
							active_we_q <= grant_we;
							active_addr_q <= grant_addr_q;
							active_din_q <= grant_din;
							active_byte_en_q <= grant_be;
							active_beats_q <= grant_beats;
							beat_q <= 3'd0;
							client_done_q <= 1'b0;
							// Writes invalidate all 8-bit caches because any
							// byte lane may have changed.
							cache0_valid_q <= grant_we ? 1'b0 : cache0_valid_q;
							cache1_valid_q <= grant_we ? 1'b0 : cache1_valid_q;
							cache2_valid_q <= grant_we ? 1'b0 : cache2_valid_q;
							a_stage_q <= col_word(grant_we, grant_size, grant_addr_q[10:0], grant_be[1:0]);
							ba_stage_q <= grant_addr_q[25:24];
							if (grant_hit_q) begin
								state_q <= ST_RCD;                 // row open: column command next
								wait_q <= {WAIT_W{1'b0}};
								wait_zero_q <= 1'b1;
							end else if (grant_open_q) begin
								state_q <= ST_PRE;                 // another row is open here
							end else begin
								SDRAM_A[12:11] <= a_stage_q[12:11];
								{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
								bank_pre_wait_q[grant_addr_q[25:24]] <= T_RAS[WAIT_W-1:0];
								state_q <= ST_RCD;
								wait_q <= wait_after(T_RCD);
								wait_zero_q <= (wait_after(T_RCD) == {WAIT_W{1'b0}});
							end
						end
					end else if (p0_req && p0_accept_i) begin
						// First sight of a request: take it now so a one-clock
						// request is never lost, and decide next clock from the
						// grant flops, which will describe it by then.
						p0_busy_q <= 1'b1;
						p0_ready <= 1'b0;
						p0_from_cache_q <= 1'b0;
						active_port_q <= 2'd0;
						active_size_q <= PORT0_SIZE[1:0];
						active_we_q <= p0_we;
						active_addr_q <= p0_aligned_addr;
						active_din_q <= p0_din;
						active_byte_en_q <= p0_byte_en;
						active_beats_q <= PORT0_BEATS;
						beat_q <= 3'd0;
						client_done_q <= 1'b0;
						cache0_valid_q <= p0_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p0_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p0_we ? 1'b0 : cache2_valid_q;
						a_stage_q <= p0_aligned_addr[23:11];
						ba_stage_q <= p0_aligned_addr[25:24];
						state_q <= ST_DECIDE;
					end else if (p1_req && p1_accept_i) begin
						p1_busy_q <= 1'b1;
						p1_ready <= 1'b0;
						p1_from_cache_q <= 1'b0;
						active_port_q <= 2'd1;
						active_size_q <= PORT1_SIZE[1:0];
						active_we_q <= p1_we;
						active_addr_q <= p1_aligned_addr;
						active_din_q <= p1_din;
						active_byte_en_q <= p1_byte_en;
						active_beats_q <= PORT1_BEATS;
						beat_q <= 3'd0;
						client_done_q <= 1'b0;
						cache0_valid_q <= p1_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p1_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p1_we ? 1'b0 : cache2_valid_q;
						a_stage_q <= p1_aligned_addr[23:11];
						ba_stage_q <= p1_aligned_addr[25:24];
						state_q <= ST_DECIDE;
					end else if (p2_req && p2_accept_i) begin
						p2_busy_q <= 1'b1;
						p2_ready <= 1'b0;
						p2_from_cache_q <= 1'b0;
						active_port_q <= 2'd2;
						active_size_q <= PORT2_SIZE[1:0];
						active_we_q <= p2_we;
						active_addr_q <= p2_aligned_addr;
						active_din_q <= p2_din;
						active_byte_en_q <= p2_byte_en;
						active_beats_q <= PORT2_BEATS;
						beat_q <= 3'd0;
						client_done_q <= 1'b0;
						cache0_valid_q <= p2_we ? 1'b0 : cache0_valid_q;
						cache1_valid_q <= p2_we ? 1'b0 : cache1_valid_q;
						cache2_valid_q <= p2_we ? 1'b0 : cache2_valid_q;
						a_stage_q <= p2_aligned_addr[23:11];
						ba_stage_q <= p2_aligned_addr[25:24];
						state_q <= ST_DECIDE;
					end
				end

				ST_RCD: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						// Column command. A10 low: the row stays open for
						// the next access to this bank.
						SDRAM_A[12:11] <= a_stage_q[12:11];
						bank_open_q[active_addr_q[25:24]] <= 1'b1;
						bank_row_q[active_addr_q[25:24]] <= active_addr_q[23:11];
						if (active_we_q) begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_WRITE;
							dq_out_q <= write_beat_data(active_size_q, active_din_q[15:0]);
							dq_oe_q <= 1'b1;
							active_din_q <= {16'd0, active_din_q[63:16]};
							active_byte_en_q <= {2'd0, active_byte_en_q[7:2]};
							active_addr_q[10:1] <= active_addr_q[10:1] + 10'd1;
							a_stage_q <= col_word(1'b1, active_size_q, {active_addr_q[10:1] + 10'd1, active_addr_q[0]}, active_byte_en_q[3:2]);
							beat_q <= 3'd1;
							if (active_beats_q == 3'd1) begin
								client_done_q <= 1'b1;
								a_stage_q <= stage_addr[23:11];
								ba_stage_q <= stage_addr[25:24];
							bank_pre_wait_q[active_addr_q[25:24]] <= T_WR[WAIT_W-1:0];
								unique case (active_port_q)
									2'd0: begin
										p0_busy_q <= 1'b0;
										p0_ready <= 1'b1;
									end
									2'd1: begin
										p1_busy_q <= 1'b0;
										p1_ready <= 1'b1;
									end
									default: begin
										p2_busy_q <= 1'b0;
										p2_ready <= 1'b1;
									end
								endcase
								state_q <= ST_RAS_WAIT;
							end else begin
								state_q <= ST_WRITE_BEATS;
							end
						end else begin
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_READ;
							// CAS_READ_CYCLES includes the registered command path,
							// the DDIO capture, and the state edge that consumes it.
							wait_q <= cas_wait_q;
							wait_zero_q <= (cas_wait_q == {WAIT_W{1'b0}});
							state_q <= ST_READ_LATENCY;
							beat_q <= 3'd0;
						end
					end
				end

				ST_READ_LATENCY: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						read_data_q[15:0] <= dq_aligned_w;
						beat_q <= 3'd1;
						if (active_beats_q == 3'd1) begin
							client_done_q <= 1'b1;
							unique case (active_port_q)
								2'd0: begin
									p0_busy_q <= 1'b0;
									p0_ready <= 1'b1;
									cache0_tag_q <= active_addr_q[25:1];
									cache0_data_q <= dq_aligned_w;
									if (PORT0_SIZE[1:0] == 2'd0) cache0_valid_q <= 1'b1;
								end
								2'd1: begin
									p1_busy_q <= 1'b0;
									p1_ready <= 1'b1;
									cache1_tag_q <= active_addr_q[25:1];
									cache1_data_q <= dq_aligned_w;
									if (PORT1_SIZE[1:0] == 2'd0) cache1_valid_q <= 1'b1;
								end
								default: begin
									p2_busy_q <= 1'b0;
									p2_ready <= 1'b1;
									cache2_tag_q <= active_addr_q[25:1];
									cache2_data_q <= dq_aligned_w;
									if (PORT2_SIZE[1:0] == 2'd0) cache2_valid_q <= 1'b1;
								end
							endcase
							if (active_beats_q < MODE_BEATS) begin
								state_q <= ST_READ_DRAIN;
								wait_q <= {{(WAIT_W-3){1'b0}}, MODE_BEATS - active_beats_q - 3'd1};
								wait_zero_q <= (MODE_BEATS - active_beats_q == 3'd1);
							end else begin
								state_q <= ST_RAS_WAIT;
							end
						end else begin
							state_q <= ST_READ_BEATS;
						end
					end
				end

				ST_READ_BEATS: begin
					// SDR SDRAM returns one 16-bit word per clock in burst
					// mode. Assemble low-to-high words into the 64-bit bus.
					unique case (beat_q[1:0])
						2'd1: read_data_q[31:16] <= dq_aligned_w;
						2'd2: read_data_q[47:32] <= dq_aligned_w;
						default: read_data_q[63:48] <= dq_aligned_w;
					endcase

					if ((beat_q + 3'd1) >= active_beats_q) begin
						client_done_q <= 1'b1;
						unique case (active_port_q)
							2'd0: begin
								p0_busy_q <= 1'b0;
								p0_ready <= 1'b1;
							end
							2'd1: begin
								p1_busy_q <= 1'b0;
								p1_ready <= 1'b1;
							end
							default: begin
								p2_busy_q <= 1'b0;
								p2_ready <= 1'b1;
							end
						endcase
						if (active_beats_q < MODE_BEATS) begin
							state_q <= ST_READ_DRAIN;
							wait_q <= {{(WAIT_W-3){1'b0}}, MODE_BEATS - active_beats_q - 3'd1};
							wait_zero_q <= (MODE_BEATS - active_beats_q == 3'd1);
						end else begin
							state_q <= ST_RAS_WAIT;
						end
					end else begin
						beat_q <= beat_q + 3'd1;
					end
				end

				ST_READ_DRAIN: begin
					// Shorter reads in a wider programmed burst leave
					// harmless tail beats on DQ. Drain them so close timing
					// is counted from the real end of the SDRAM burst.
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						state_q <= ST_RAS_WAIT;
					end
				end

				ST_WRITE_BEATS: begin
					// Writes drive one 16-bit word per clock, each beat shifted
					// into the low word. For 8-bit writes, A12:A11/DQM masks
					// the byte not selected by addr[0]. Only entered with a
					// beat still to send.
					begin
						SDRAM_A[12:11] <= a_stage_q[12:11];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_WRITE;
						dq_out_q <= write_beat_data(active_size_q, active_din_q[15:0]);
						dq_oe_q <= 1'b1;
						active_din_q <= {16'd0, active_din_q[63:16]};
						active_byte_en_q <= {2'd0, active_byte_en_q[7:2]};
						active_addr_q[10:1] <= active_addr_q[10:1] + 10'd1;
						a_stage_q <= col_word(1'b1, active_size_q, {active_addr_q[10:1] + 10'd1, active_addr_q[0]}, active_byte_en_q[3:2]);
						if ((beat_q + 3'd1) >= active_beats_q) begin
							client_done_q <= 1'b1;
							a_stage_q <= stage_addr[23:11];
							ba_stage_q <= stage_addr[25:24];
							bank_pre_wait_q[active_addr_q[25:24]] <= T_WR[WAIT_W-1:0];
							unique case (active_port_q)
								2'd0: begin
									p0_busy_q <= 1'b0;
									p0_ready <= 1'b1;
								end
								2'd1: begin
									p1_busy_q <= 1'b0;
									p1_ready <= 1'b1;
								end
								default: begin
									p2_busy_q <= 1'b0;
									p2_ready <= 1'b1;
								end
							endcase
							state_q <= ST_RAS_WAIT;
						end else begin
							beat_q <= beat_q + 3'd1;
						end
					end
				end

				ST_RAS_WAIT: begin
					if (pending_valid_q) begin
						active_port_q <= pending_port_q;
						active_size_q <= pending_size_q;
						active_we_q <= pending_we_q;
						active_addr_q <= pending_addr_q;
						active_din_q <= pending_din_q;
						active_byte_en_q <= pending_byte_en_q;
						active_beats_q <= pending_beats_q;
						beat_q <= 3'd0;
						client_done_q <= 1'b0;
						pending_valid_q <= 1'b0;
						a_stage_q <= col_word(pending_we_q, pending_size_q, pending_addr_q[10:0], pending_byte_en_q[1:0]);
						ba_stage_q <= pending_addr_q[25:24];
						if (pending_hit_w) begin
							state_q <= ST_RCD;                 // row open: column command next
							wait_q <= {WAIT_W{1'b0}};
							wait_zero_q <= 1'b1;
						end else if (bank_open_q[pending_addr_q[25:24]]) begin
							state_q <= ST_PRE;                 // another row is open here
						end else begin
							SDRAM_A[12:11] <= a_stage_q[12:11];
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
							bank_pre_wait_q[pending_addr_q[25:24]] <= T_RAS[WAIT_W-1:0];
							state_q <= ST_RCD;
							wait_q <= wait_after(T_RCD);
							wait_zero_q <= (wait_after(T_RCD) == {WAIT_W{1'b0}});
						end
					end else begin
						client_done_q <= 1'b0;
						state_q <= ST_IDLE;
					end
				end

				// A request taken on the clock it was first seen. The grant
				// flops were loaded from it on that same edge, so they are
				// about this address now.
				ST_DECIDE: begin
					a_stage_q <= col_word(active_we_q, active_size_q, active_addr_q[10:0], active_byte_en_q[1:0]);
					if (grant_hit_q) begin
						state_q <= ST_RCD;                 // row open: column command next
						wait_q <= {WAIT_W{1'b0}};
						wait_zero_q <= 1'b1;
					end else if (grant_open_q) begin
						state_q <= ST_PRE;                 // another row is open here
					end else begin
						SDRAM_A[12:11] <= a_stage_q[12:11];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						bank_pre_wait_q[active_addr_q[25:24]] <= T_RAS[WAIT_W-1:0];
						state_q <= ST_RCD;
						wait_q <= wait_after(T_RCD);
						wait_zero_q <= (wait_after(T_RCD) == {WAIT_W{1'b0}});
					end
				end

				// Another row is open in the bank this access needs. Close it
				// once tRAS and tWR allow, wait tRP, then open ours.
				ST_PRE: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else if (!pre_done_q) begin
						if (bank_pre_ok_q[active_addr_q[25:24]]) begin
							SDRAM_A[12:11] <= a_stage_q[12:11];      // staged column word: A10 low, this bank only
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;
							a_stage_q <= active_addr_q[23:11];
							bank_open_q[active_addr_q[25:24]] <= 1'b0;
							wait_q <= wait_after(T_RP);
							wait_zero_q <= (wait_after(T_RP) == {WAIT_W{1'b0}});
							pre_done_q <= 1'b1;
						end
					end else begin
						SDRAM_A[12:11] <= a_stage_q[12:11];
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_ACTIVE;
						a_stage_q <= col_word(active_we_q, active_size_q, active_addr_q[10:0], active_byte_en_q[1:0]);
						bank_pre_wait_q[active_addr_q[25:24]] <= T_RAS[WAIT_W-1:0];
						pre_done_q <= 1'b0;
						state_q <= ST_RCD;
						wait_q <= wait_after(T_RCD);
						wait_zero_q <= (wait_after(T_RCD) == {WAIT_W{1'b0}});
					end
				end

				// Refresh with rows open: PRECHARGE ALL once every bank may be
				// closed, wait tRP, then the refresh the caller already booked.
				ST_PRE_ALL: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else if (!pre_done_q) begin
						if (&bank_pre_ok_q) begin
							SDRAM_A[12:11] <= a_stage_q[12:11];      // staged 0x400: A10 high, all banks
							{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_PRECHARGE;
							bank_open_q <= 4'd0;
							wait_q <= wait_after(T_RP);
							wait_zero_q <= (wait_after(T_RP) == {WAIT_W{1'b0}});
							pre_done_q <= 1'b1;
						end
					end else begin
						{SDRAM_nCS, SDRAM_nRAS, SDRAM_nCAS, SDRAM_nWE} <= CMD_AUTO_REFRESH;
						pre_done_q <= 1'b0;
						state_q <= ST_REFRESH_WAIT;
						wait_q <= wait_after(T_RFC);
						wait_zero_q <= (wait_after(T_RFC) == {WAIT_W{1'b0}});
					end
				end

				ST_REFRESH_WAIT: begin
					if (!wait_zero_q) begin
						wait_q <= wait_q - {{(WAIT_W-1){1'b0}}, 1'b1};
						wait_zero_q <= (wait_q == {{(WAIT_W-1){1'b0}}, 1'b1});
					end else begin
						state_q <= ST_IDLE;
					end
				end

				default: begin
					state_q <= ST_POWER_WAIT;
					init_done_q <= 1'b0;
					init_wait_q <= count_after_command(T_POWER_WAIT);
					p0_busy_q <= 1'b1;
					p1_busy_q <= 1'b1;
					p2_busy_q <= 1'b1;
					p0_ready <= 1'b1;
					p1_ready <= 1'b1;
					p2_ready <= 1'b1;
					p0_cache_done_q <= 1'b0;
					p1_cache_done_q <= 1'b0;
					p2_cache_done_q <= 1'b0;
					wait_q <= {WAIT_W{1'b0}};
					wait_zero_q <= 1'b1;
				end
			endcase
		end
	end

endmodule

`default_nettype wire
