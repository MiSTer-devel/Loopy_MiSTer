// Clock enables for the whole core.
//
//   clk_ram    128.863636 MHz   SDRAM, exactly four times clk_sys
//   clk_video   42.954545 MHz   the RH-7500
//   clk_sys     32.215909 MHz   the SH7021, the sound, the glue
//
//   ce_vdp      clk_video / 2   21.477273 MHz   exact
//   ce_cpu_r    clk_sys   / 2   16.107955 MHz   +0.675%
//   ce_cpu_f    the other half  16.107955 MHz   half a CPU cycle later
//   ce_midi4m   clk_sys   / 8    4.026989 MHz   +0.675%
//   ce_sample   accumulator     84864 Hz        exact (21.725 MHz / 256)
//
// The CPU enables are plain dividers so a memory access always lands a fixed
// number of CPU cycles later; the bus controller needs exact T1/T2/Tw counts.
// One shared VCO cannot give both an exact VDP clock and 32.000 MHz, so the
// CPU runs 0.675% fast. ce_sample alternates 379 and 380 clk_sys cycles.
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

	// CPU clock is exactly half clk_sys, so the two phases are the two halves
	// of the divider.
	reg cpu_div = 1'b0;
	assign ce_cpu_r = cpu_run & ~cpu_div;
	assign ce_cpu_f = cpu_run &  cpu_div;
	always @(posedge clk_sys) begin
		if (cpu_run) cpu_div <= ~cpu_div;
	end

	// Drives the CDT109 receiver, whose bytes come from the SH7021 SCI running
	// off ce_cpu_r. The two only stay in step (1024 clk_sys per bit at each
	// end) while they stop and start together, hence cpu_run.
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
