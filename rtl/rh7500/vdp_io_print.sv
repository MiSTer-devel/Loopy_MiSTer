// Printer registers and the analogue input.
//
// The sensors read whatever the mechanism says, the motor and head registers
// accept writes that pass their key check, and the ADC returns a value per
// channel. With no mechanism attached the sensors read "no seal cartridge".
//
//   PRINT_SENSORS 0x5D030   SCT, OPTO and RJMP, latched when TRIGGER is
//                           written with PSEN. ENP is writable and gates the
//                           motor and head control registers.
//   PRINT_MOTOR 0x5D042     key 0x5A5 in the top twelve bits, four phase bits
//   PRINT_HEAD_CTRL 0x5D044 key 0xA5A, four control bits; the four read back
//   PRINT_HEAD_DATA 0x5D040 the print routine DMAs into this; the word is
//                           kept as written and sent out with a strobe
//   ANALOG_IN 0x5D000       10-bit reading in the top bits, then the channel
//                           mux, an automatic-sample enable and its rate.
//
// A conversion is one of the two IRQ2 sources, so a trigger starts it and
// `adc_ready` pulses a few VDP cycles later.

module vdp_io_print
(
	input  wire clk,
	input  wire reset,

	// Savestate scalar bus.
	input  wire        ss_clk,      // clk_sys
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// Register bus slice.
	input  wire [11:1] addr,
	input  wire        wr,
	input  wire [1:0]  be,
	input  wire [15:0] wdata,
	output reg  [15:0] rdata,
	output reg         hit,

	input  wire        trig_psen,
	input  wire        trig_adc,
	input  wire        line_start,
	input  wire        half_line,   // mid-line tick, for the fastest ADC rate
	input  wire        ce_vdp,

	// Board straps and mechanism sensors.
	input  wire [2:0]  sct,
	input  wire [2:0]  opto,
	input  wire        region_ntsc,

	output reg  [3:0]  motor_phase,
	output reg  [3:0]  head_ctrl,
	output reg  [15:0] head_data,
	output reg         head_data_wr, // one cycle per write, for the capture
	output wire        printer_enable,


	output reg         adc_ready     // one-cycle pulse, an IRQ2 source
);

	// Restored savestate values; the ss_reg instances are at the end of the module.
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] ss_pr;
	/* verilator lint_on UNUSEDSIGNAL */

	localparam [11:0] KEY_MOTOR = 12'h5A5;
	localparam [11:0] KEY_HEAD  = 12'hA5A;

	reg        enp;
	assign printer_enable = enp;
	reg [2:0]  sens_sct;
	reg [2:0]  sens_opto;
	reg        sens_rjmp;

	reg [9:0]  adc_val;
	reg [2:0]  adc_mux;
	reg        adc_auto;
	reg [1:0]  adc_rate;

	// Mux values 5 to 7 are invalid and select channel 4.
	reg [9:0] adc_src;
	always @* begin
		// One console's readings; channels 3 and 4 are a cartridge's.
		case (adc_mux)
		3'd0:    adc_src = 10'h154;   // head thermistor
		3'd1:    adc_src = 10'h236;   // head calibration resistor
		3'd2:    adc_src = 10'h207;   // contrast pot
		3'd3:    adc_src = 10'h01D;
		default: adc_src = 10'h01B;
		endcase
	end

	// Automatic sampling: every fourth line, every other line, every line, or
	// twice a line.
	reg [1:0] line_div;
	wire auto_tick = adc_auto &
		((adc_rate == 2'd0) ? (line_start & (line_div == 2'd3))
		: (adc_rate == 2'd1) ? (line_start & line_div[0])
		: (adc_rate == 2'd2) ? line_start
		:                      (line_start | half_line));

	// A conversion takes about 354 VDP cycles, measured from the trigger to
	// its interrupt.
	reg [8:0] adc_cnt;

	always @(posedge clk) begin
		adc_ready    <= 1'b0;
		head_data_wr <= 1'b0;

		if (reset) begin
			enp        <= ss_pr[0];
			sens_sct   <= ss_pr[3:1];
			sens_opto  <= ss_pr[6:4];
			sens_rjmp  <= ss_pr[7];
			adc_val    <= ss_pr[17:8];
			adc_mux    <= ss_pr[20:18];
			adc_auto   <= ss_pr[21];
			adc_rate   <= ss_pr[23:22];
			motor_phase <= ss_pr[27:24];
			head_ctrl  <= ss_pr[31:28];
			head_data  <= ss_pr[47:32];
			adc_cnt    <= 9'd0;
			line_div   <= 2'd0;
		end else begin
			if (line_start) line_div <= line_div + 2'd1;

			if (trig_psen) begin
				sens_sct  <= sct;
				sens_opto <= opto;
				sens_rjmp <= region_ntsc;
			end

			if (trig_adc | auto_tick) adc_cnt <= 9'd354;
			else if (ce_vdp & (adc_cnt != 9'd0)) begin
				adc_cnt <= adc_cnt - 9'd1;
				if (adc_cnt == 9'd1) begin
					adc_val   <= adc_src;
					adc_ready <= 1'b1;
				end
			end

			if (wr) begin
				case (addr)
				11'h000: begin                                 // ANALOG_IN
					if (be[0]) begin
						adc_rate <= wdata[1:0];
						adc_auto <= wdata[2];
						adc_mux  <= wdata[5:3];
					end
				end
				11'h018: if (be[1]) enp <= wdata[8];           // PRINT_SENSORS
				11'h020: begin                                 // PRINT_HEAD_DATA
					if (be[0]) head_data[7:0]  <= wdata[7:0];
					if (be[1]) head_data[15:8] <= wdata[15:8];
					head_data_wr <= |be;
				end
				// The key spans both bytes, so only a whole-word write can present one.
				11'h021: if (enp & (&be) & (wdata[15:4] == KEY_MOTOR))
					motor_phase <= wdata[3:0];       // PRINT_MOTOR
				11'h022: if (enp & (&be) & (wdata[15:4] == KEY_HEAD))
					head_ctrl <= wdata[3:0];         // PRINT_HEAD_CTRL
				default: ;
				endcase
			end
		end
	end

	always @* begin
		rdata = 16'd0;
		hit   = 1'b0;
		case (addr)
		11'h000: begin hit = 1'b1;
			rdata = {adc_val, adc_mux, adc_auto, adc_rate}; end
		11'h018: begin hit = 1'b1;
			rdata = {7'd0, enp, 1'b0, sens_sct, sens_opto, sens_rjmp}; end
		// The key reads back as zero; the BIOS masks the top of the word off.
		11'h022: begin hit = 1'b1; rdata = {12'd0, head_ctrl}; end
		default: ;
		endcase
	end

	// ---- savestate -----------------------------------------------------------

	`include "ss_map.svh"


	ss_reg #(.ADDR (SSW_VDP_IOPRINT), .DEFAULT (64'd0)) u_ss (
		.clk_i      (ss_clk),
		.bus_din_i  (ss_din),
		.bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren),
		.bus_rst_i  (ss_rst),
		.bus_dout_o (ss_dout),
		.din_i      ({16'd0, head_data, head_ctrl, motor_phase, adc_rate, adc_auto, adc_mux, adc_val, sens_rjmp, sens_opto, sens_sct, enp}),
		.dout_o     (ss_pr)
	);

endmodule
