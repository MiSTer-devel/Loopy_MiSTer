// Copyright (c) 2026 Jamie Blanks
//
// One SH7021 serial communication interface channel. Channel 0 sits at
// H'5FFFEC0, channel 1 at H'5FFFEC8. Asynchronous mode samples the line at
// sixteen times the bit rate and takes each bit mid-window. Both clock
// paths are internal; SCK is an output only.

module sh7021_sci #(
	parameter [7:0] BASE = 8'h60,     // word address of SMR/BRR
	// First of the two savestate words (SSW_SCI0_BASE or SSW_SCI1_BASE).
	parameter integer SS_BASE = 0
) (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        rst_i,

	input  wire        psel_i,
	input  wire [8:0]  paddr_i,
	input  wire        pwr_i,
	input  wire [1:0]  pbe_i,
	input  wire [15:0] pwdata_i,
	output reg  [15:0] prdata_o,
	output wire        phit_o,

	// A DMAC transfer to TDR or from RDR clears the flag that asked for it;
	// a CPU access leaves it for software.
	input  wire        dma_tx_ack_i,
	input  wire        dma_rx_ack_i,

	input  wire        rxd_i,
	output wire        txd_o,
	output wire        txd_oe_o,
	output wire        sck_o,
	output wire        sck_oe_o,

	output wire        eri_o,
	output wire        rxi_o,
	output wire        txi_o,
	output wire        tei_o,

	// savestate scalar bus
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout
);
	reg [7:0] smr, brr, scr, tdr, rdr;
	reg       tdre, rdrf, orer, fer, per, tend, mpb, mpbt;

	wire       ca   = smr[7];
	wire       chr  = smr[6];
	wire       pe   = smr[5];
	wire       oe   = smr[4];
	wire       stop2= smr[3];
	wire       mp   = smr[2];
	wire [1:0] cks  = smr[1:0];
	wire       tie  = scr[7];
	wire       rie  = scr[6];
	wire       te   = scr[5];
	wire       re   = scr[4];
	wire       mpie = scr[3];
	wire       teie = scr[2];
	wire [1:0] cke  = scr[1:0];

	wire [7:0] ssr = {tdre, rdrf, orer, fer, per, tend, mpb, mpbt};
	wire [7:0] w   = paddr_i[8:1];
	assign phit_o = psel_i && (w >= BASE) && (w <= (BASE + 8'd2));

	always @* begin
		case (w - BASE)
		8'd0:    prdata_o = {smr, brr};
		8'd1:    prdata_o = {scr, tdr};
		default: prdata_o = {ssr, rdr};
		endcase
	end

	// -------------------------------------------------------- rate divider
	// One tick per sixteenth of an async bit (2 * 4^n * (N+1) states) or per
	// quarter of a clocked synchronous bit (4^n * (N+1) states).
	reg  [15:0] div;
	wire [15:0] period = ({8'd0, brr} + 16'd1) << ({1'b0, cks} << 1)
	                     << (ca ? 1'b0 : 1'b1);
	wire        tick = (div == 16'd0);

	// ------------------------------------------------------------ transmit
	reg [3:0]  tx_ph;                 // sixteenth-of-a-bit phase
	reg [3:0]  tx_bit;
	reg [11:0] tx_sr;                 // start, up to 8 data, parity, 2 stop
	reg        tx_active;
	assign txd_o    = tx_active ? tx_sr[0] : 1'b1;
	assign txd_oe_o = te;
	// Clocked synchronous SCK falls where the data changes and idles high;
	// the asynchronous clock output is one pulse per bit.
	assign sck_o    = ca ? (tx_active ? tx_ph[1] : 1'b1) : tx_ph[3];
	assign sck_oe_o = ca ? (cke[1] == 1'b0) : (cke == 2'b01);

	// ------------------------------------------------------------- receive
	reg [3:0]  rx_ph;
	reg [3:0]  rx_bit;                // 0 start, 1..n data, then parity, stop
	reg [15:0] rx_sr;   // the low nine bits carry a frame
	reg        rx_active;
	reg        rx_done;
	reg        rx_stop;
	reg        rxd_d;

	assign eri_o = rie & (orer | fer | per);
	assign rxi_o = rie & rdrf;
	assign txi_o = tie & tdre;
	assign tei_o = teie & tend;

	wire [7:0] rx_data   = chr ? {1'b0, rx_sr[6:0]} : rx_sr[7:0];
	wire [3:0] data_bits = ca ? 4'd8 : (chr ? 4'd7 : 4'd8);
	wire       extra_bit = !ca && (pe | mp);
	// frame length in bit times: start, data, optional parity, stop bits
	wire [3:0] tx_len = ca ? data_bits
	                       : (4'd1 + data_bits + {3'd0, extra_bit}
	                          + (stop2 ? 4'd2 : 4'd1));
	wire tx_last = tx_active && (tx_ph == (ca ? 4'd3 : 4'd15))
	               && (tx_bit == (tx_len - 4'd1));

	function automatic [15:0] mrg(input [15:0] old, input [15:0] nw,
	                              input [1:0] lanes);
		mrg = {lanes[1] ? nw[15:8] : old[15:8],
		       lanes[0] ? nw[7:0]  : old[7:0]};
	endfunction

	wire rx_fer = !ca && !rx_stop;
	wire rx_per = !ca && pe && (parity_of(rx_data, chr, oe) != rx_sr[8]);

	// Qualify with phit_o: the case below decodes only the offset from BASE,
	// so psel_i alone would let any peripheral write clear this SSR.
	wire wr = ce_i && phit_o && pwr_i;

	// --------------------------------------------------------- savestate
	/* verilator lint_off UNUSEDSIGNAL */
	wire [63:0] SS_SCI0, SS_SCI1;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [63:0] ss_dout0, ss_dout1;

	wire [63:0] SS_SCI0_BACK = {div, ssr, rdr, tdr, scr, brr, smr};
	wire [63:0] SS_SCI1_BACK = {15'd0, rxd_d, rx_stop, rx_done, rx_active,
	                            rx_sr, rx_bit, rx_ph,
	                            tx_active, tx_sr, tx_bit, tx_ph};

	ss_reg #(.ADDR (SS_BASE + 0),
	         .DEFAULT ({16'd0, 8'h84, 8'h00, 8'hFF, 8'h00, 8'hFF, 8'h00}))
	       u_ss0 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout0),
		.din_i (SS_SCI0_BACK), .dout_o (SS_SCI0)
	);
	// The idle line and the parked shift register are both all ones.
	ss_reg #(.ADDR (SS_BASE + 1),
	         .DEFAULT ({15'd0, 1'b1, 1'b1, 1'b0, 1'b0, 16'd0, 4'd0, 4'd0,
	                    1'b0, 12'hFFF, 4'd0, 4'd0}))
	       u_ss1 (
		.clk_i (clk_i), .bus_din_i (ss_din), .bus_addr_i (ss_addr),
		.bus_wren_i (ss_wren), .bus_rst_i (ss_rst), .bus_dout_o (ss_dout1),
		.din_i (SS_SCI1_BACK), .dout_o (SS_SCI1)
	);
	assign ss_dout = ss_dout0 | ss_dout1;

	always @(posedge clk_i) begin
		if (rst_i) begin
			smr  <= SS_SCI0[7:0];   brr  <= SS_SCI0[15:8];
			scr  <= SS_SCI0[23:16]; tdr  <= SS_SCI0[31:24];
			rdr  <= SS_SCI0[39:32];
			tdre <= SS_SCI0[47];    rdrf <= SS_SCI0[46];
			orer <= SS_SCI0[45];    fer  <= SS_SCI0[44];
			per  <= SS_SCI0[43];    tend <= SS_SCI0[42];
			mpb  <= SS_SCI0[41];    mpbt <= SS_SCI0[40];
			div  <= SS_SCI0[63:48];
			tx_ph <= SS_SCI1[3:0];   tx_bit <= SS_SCI1[7:4];
			tx_sr <= SS_SCI1[19:8];  tx_active <= SS_SCI1[20];
			rx_ph <= SS_SCI1[24:21]; rx_bit <= SS_SCI1[28:25];
			rx_sr <= SS_SCI1[44:29]; rx_active <= SS_SCI1[45];
			rx_done <= SS_SCI1[46];  rx_stop <= SS_SCI1[47];
			rxd_d <= SS_SCI1[48];
		end else if (ce_i) begin
			div <= tick ? (period - 16'd1) : (div - 16'd1);

			if (tick) begin
				rxd_d <= rxd_i;
				// ---------------------------------------------- transmitter
				// One bit is four ticks in clocked synchronous mode and
				// sixteen in asynchronous mode.
				tx_ph <= ca ? ((tx_ph + 4'd1) & 4'd3) : (tx_ph + 4'd1);
				if (tx_active && (tx_ph == (ca ? 4'd3 : 4'd15))) begin
					tx_sr  <= {1'b1, tx_sr[11:1]};
					tx_bit <= tx_bit + 4'd1;
					if (tx_last) begin
						tx_active <= 1'b0;
						// TEND, and TEI, only when nothing follows this frame
						if (tdre) tend <= 1'b1;
					end
				end

				// A frame whose data is already waiting follows the last one
				// with no gap. In clocked synchronous mode a receive error
				// blocks the start even with TDRE set.
				if (te && (!tx_active || tx_last) && !tdre
				    && !(ca && (orer || fer || per))) begin
					// TDR moves into the shift register and the frame starts
					tx_active <= 1'b1;
					tx_bit    <= 4'd0;
					tx_ph     <= 4'd0;
					tdre      <= 1'b1;
					tend      <= 1'b0;
					tx_sr     <= ca
					             ? {4'b1111, tdr}
					             : {2'b11,
					                mp ? mpbt : (pe ? parity_of(tdr, chr, oe) : 1'b1),
					                chr ? {1'b1, tdr[6:0]} : tdr,
					                1'b0};
				end

				// ------------------------------------------------- receiver
				if (re && !orer && !fer && !per) begin
					if (!rx_active) begin
						// asynchronous reception starts on the falling edge of
						// the start bit, sampled at the sixteen-times rate;
						// the clocked form runs continuously
						if (ca || (!rxd_i && rxd_d)) begin
							rx_active <= 1'b1;
							rx_bit    <= 4'd0;
							rx_ph     <= 4'd0;
						end
					end else begin
						rx_ph <= ca ? ((rx_ph + 4'd1) & 4'd3)
						            : (rx_ph + 4'd1);
						// sample in the middle of the bit
						if (ca ? (rx_ph == 4'd1) : (rx_ph == 4'd7)) begin
							if (!ca && (rx_bit == 4'd0)) begin
								// the edge was noise
								if (rxd_i) rx_active <= 1'b0;
							end else if (rx_bit <= (ca ? data_bits : data_bits))
								rx_sr[ca ? rx_bit : (rx_bit - 4'd1)] <= rxd_i;
							else if (extra_bit && (rx_bit == data_bits + 4'd1))
								rx_sr[8] <= rxd_i;
							else begin
								rx_stop   <= rxd_i;   // the first stop bit
								rx_done   <= 1'b1;
								rx_active <= 1'b0;
							end
							if (ca && (rx_bit == (data_bits - 4'd1))) begin
								rx_done   <= 1'b1;
								rx_stop   <= 1'b1;
								rx_active <= 1'b0;
							end
						end
						if (ca ? (rx_ph == 4'd3) : (rx_ph == 4'd15))
							rx_bit <= rx_bit + 4'd1;
					end
				end
				// The frame is judged where the stop bit was sampled. Each
				// error has its own flag and they can all be set at once;
				// only an overrun withholds the transfer to RDR.
				if (rx_done) begin
					rx_done <= 1'b0;
					// While MPIE is set, a frame whose multiprocessor bit is 0
					// is discarded whole.
					if (!ca && mp && mpie && !rx_sr[8]) begin
						// discarded: no flag, no transfer
					end else begin
						if (!ca && mp && mpie) scr[3] <= 1'b0;
						if (rdrf) orer <= 1'b1;
						if (rx_fer) fer <= 1'b1;
						if (rx_per) per <= 1'b1;
						if (!rdrf) begin
							rdr <= rx_data;
							if (!rx_fer && !rx_per) rdrf <= 1'b1;
							if (!ca && mp) mpb <= rx_sr[8];
						end
					end
				end
			end

			if (dma_tx_ack_i) begin tdre <= 1'b0; tend <= 1'b0; end
			if (dma_rx_ack_i) rdrf <= 1'b0;

			// -------------------------------------------------------- writes
			if (wr) case (w - BASE)
				8'd0: begin
					if (pbe_i[1]) smr <= pwdata_i[15:8];
					if (pbe_i[0]) brr <= pwdata_i[7:0];
				end
				8'd1: begin
					if (pbe_i[1]) begin
						scr <= pwdata_i[15:8];
						// clearing TE parks the transmitter with TDRE set
						if (!pwdata_i[13]) begin
							tdre <= 1'b1; tend <= 1'b1; tx_active <= 1'b0;
						end
					end
					// software clears TDRE in SSR after writing TDR
					if (pbe_i[0]) tdr <= pwdata_i[7:0];
				end
				8'd2: if (pbe_i[1]) begin
					// only a zero clears a flag that has been read as one
					if (!pwdata_i[15]) begin tdre <= 1'b0; tend <= 1'b0; end
					if (!pwdata_i[14]) rdrf <= 1'b0;
					if (!pwdata_i[13]) orer <= 1'b0;
					if (!pwdata_i[12]) fer  <= 1'b0;
					if (!pwdata_i[11]) per  <= 1'b0;
					mpbt <= pwdata_i[8];
				end
				default: ;
			endcase
		end
	end


	function automatic parity_of(input [7:0] d, input seven, input odd);
		begin
			parity_of = (seven ? ^d[6:0] : ^d) ^ odd;
		end
	endfunction

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, paddr_i[0], cke[0], rx_sr[15:9]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
endmodule
