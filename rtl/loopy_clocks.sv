// Clock enables for the whole core.
//
//   clk_ram    128.863636 MHz   SDRAM, exactly four times clk_sys
//   clk_video   42.954545 MHz   the RH-7500
//   clk_sys     32.215909 MHz   the SH7021, the sound, the glue
//
//   ce_vdp      clk_video / 2   21.477273 MHz   exact
//   ce_cpu_r    clk_sys * 106/213  16.032329 MHz
//   ce_cpu_f    the other half     16.032329 MHz   half a CPU cycle later
//   ce_midi4m   clk_sys   / 8       4.026989 MHz   +0.675%
//   ce_sample   accumulator        84864 Hz        exact (21.725 MHz / 256)
//
// A CPU state is two clk_sys cycles, and one state in every 106 is three:
// the console's 16 MHz resonator measures about 16.03 MHz against the VDP
// crystal (267,970 CPU cycles a frame), which clk_sys / 2 overshoots. Every
// state still has its two enables, so the bus controller's T1/T2/Tw counts
// are unchanged; the long state only gives memory more time. ce_sample
// alternates 379 and 380 clk_sys cycles.
//
// pause stops every enable. mem_stall stops only the CPU and the MIDI
// receiver, so the two ends of the serial link keep the same time. The
// dividers free-run through reset, like a crystal, so downstream modules can
// still see their enabled reset cycle.

module loopy_clocks
(
	// Video domain.
	input  wire clk_video,
	input  wire pause_video,
	output wire ce_vdp,

	// System domain.
	input  wire clk_sys,
	input  wire pause_sys,
	input  wire mem_stall,
	output wire ce_cpu_r,
	output wire ce_cpu_f,
	output wire ph_cpu_r,      // the cycle ce_cpu_r falls in, stalled or not
	output wire ce_midi4m,
	output wire ce_sample
);

	wire cpu_run = ~pause_sys & ~mem_stall;

	// ---- video domain ----------------------------------------------------

	// Power-up values: these dividers take no reset, so this is what makes
	// them deterministic at configuration and in simulation.
	/* verilator lint_off PROCASSINIT */
	reg vdp_div = 1'b0;
	assign ce_vdp = ~pause_video & vdp_div;
	always @(posedge clk_video) begin
		if (!pause_video) vdp_div <= ~vdp_div;
	end

	// ---- system domain ---------------------------------------------------

	// The two phases are the two halves of a divide by two. After every
	// 106th state one clk_sys cycle is held, lengthening that state's second
	// half.
	localparam [6:0] CPU_LONG_EVERY = 7'd106;
	reg       cpu_div  = 1'b0;
	reg       cpu_hold = 1'b0;
	reg [6:0] cpu_cnt  = 7'd0;
	assign ph_cpu_r = ~cpu_div & ~cpu_hold;
	assign ce_cpu_r = cpu_run & ph_cpu_r;
	assign ce_cpu_f = cpu_run &  cpu_div;
	always @(posedge clk_sys) begin
		if (cpu_run) begin
			if (cpu_hold) begin
				cpu_hold <= 1'b0;
			end else begin
				cpu_div <= ~cpu_div;
				if (cpu_div) begin
					if (cpu_cnt == CPU_LONG_EVERY - 7'd1) begin
						cpu_cnt  <= 7'd0;
						cpu_hold <= 1'b1;
					end else begin
						cpu_cnt <= cpu_cnt + 7'd1;
					end
				end
			end
		end
	end

	// Drives the CDT109 receiver, whose bytes come from the SH7021 SCI running
	// off ce_cpu_r. The two stay within a fraction of a percent of each other
	// (1024 and about 1029 clk_sys per bit) while they stop and start
	// together, hence cpu_run.
	reg [2:0] midi_div = 3'd0;
	assign ce_midi4m = cpu_run & (midi_div == 3'd7);
	always @(posedge clk_sys) begin
		if (cpu_run) midi_div <= midi_div + 3'd1;
	end

	// 84864 Hz exactly: 38896 / 14765625 of clk_sys.
	localparam [23:0] SAMPLE_INC = 24'd38896;
	localparam [23:0] SAMPLE_DEN = 24'd14765625;

	reg [23:0] sample_acc = 24'd0;
	reg        sample_tick = 1'b0;
	/* verilator lint_on PROCASSINIT */
	assign ce_sample = ~pause_sys & sample_tick;
	always @(posedge clk_sys) begin
		if (!pause_sys) begin
			if (sample_acc + SAMPLE_INC >= SAMPLE_DEN) begin
				sample_acc  <= sample_acc + SAMPLE_INC - SAMPLE_DEN;
				sample_tick <= 1'b1;
			end else begin
				sample_acc  <= sample_acc + SAMPLE_INC;
				sample_tick <= 1'b0;
			end
		end
	end

endmodule
