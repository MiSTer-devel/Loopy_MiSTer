// Quiet audible feedback from the printer's four-phase motor drive.
// One waveform cycle per four phase steps.
module loopy_printer_audio (
	input  wire               clk,
	input  wire               reset,
	input  wire               ce_sample,
	input  wire               enable,
	input  wire [3:0]         motor_phase,
	output reg signed [15:0]  sample
);
	// Motor phase levels remain stable far longer than either clock period.
	reg [3:0] phase_meta, phase_sync;
	reg enable_meta, enable_sync;
	always @(posedge clk) begin
		if (reset) begin
			phase_meta <= 4'd0;
			phase_sync <= 4'd0;
			enable_meta <= 1'b0;
			enable_sync <= 1'b0;
		end else begin
			phase_meta <= motor_phase;
			phase_sync <= phase_meta;
			enable_meta <= enable;
			enable_sync <= enable_meta;
		end
	end

	reg valid;
	reg signed [23:0] target;
	always @* begin
		valid = 1'b1;
		case (phase_sync)
		4'hC, 4'h3: target = 24'sd0;
		4'h6: target = 24'sd65536;  // +256 PCM, with eight fractional bits.
		4'h9: target = -24'sd65536;
		default: begin target = 24'sd0; valid = 1'b0; end
		endcase
	end

	// Remove DC when the motor holds a phase. At 84,864 samples/s the
	// time constant is about 12 ms; low-level residue settles to exact zero.
	reg signed [23:0] dc;
	wire signed [23:0] delta = target - dc;
	always @(posedge clk) begin
		if (reset) begin
			dc <= 24'sd0;
			sample <= 16'sd0;
		end else if (ce_sample) begin
			if (!enable_sync || !valid) begin
				dc <= 24'sd0;
				sample <= 16'sd0;
			end else if ((delta >= -24'sd1024) && (delta <= 24'sd1024)) begin
				dc <= target;
				sample <= 16'sd0;
			end else begin
				dc <= dc + (delta >>> 10);
				sample <= delta[23:8];
			end
		end
	end
endmodule
