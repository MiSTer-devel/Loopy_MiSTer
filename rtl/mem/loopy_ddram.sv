// One DDR transaction at a time, shared by savestates and sticker storage.
// Accepted reads retain their owner through every response, including reset
// cancellation. Both cold resets must cover the same reset event; DDR must
// still return the outstanding beats so a cancelled burst can drain.
module loopy_ddram (
	input  wire        clk,
	input  wire        reset,
	input  wire        ss_clk,
	input  wire        ss_reset,

	output wire        DDRAM_CLK,
	input  wire        DDRAM_BUSY,
	output wire [7:0]  DDRAM_BURSTCNT,
	output wire [28:0] DDRAM_ADDR,
	input  wire [63:0] DDRAM_DOUT,
	input  wire        DDRAM_DOUT_READY,
	output wire        DDRAM_RD,
	output wire [63:0] DDRAM_DIN,
	output wire [7:0]  DDRAM_BE,
	output wire        DDRAM_WE,

	input  wire [25:0] addr_i,
	input  wire [63:0] din_i,
	output wire [63:0] dout_o,
	input  wire [7:0]  be_i,
	input  wire        rnw_i,
	input  wire        ena_i,
	output wire        done_o,

	input  wire        pr_req,
	output wire        pr_ready,
	input  wire [28:0] pr_addr,
	input  wire [63:0] pr_din,
	input  wire [7:0]  pr_be,
	input  wire        pr_rnw,
	input  wire [7:0]  pr_burstcnt,
	output reg  [63:0] pr_dout,
	output reg         pr_valid,
	output reg         pr_done
);
	localparam [1:0] ST_IDLE = 2'd0, ST_COMMAND = 2'd1, ST_READ = 2'd2;
	reg [1:0] state_q;
	initial state_q = ST_IDLE;
	reg owner_ss_q;
	reg prefer_ss_q;
	reg ss_taken_q;
	reg discard_q;
	reg [28:0] addr_q;
	reg [63:0] din_q;
	reg [7:0] be_q;
	reg rnw_q;
	reg [7:0] burst_q;
	reg [7:0] remaining_q;

	wire ss_valid;
	wire [98:0] ss_data;
	reg ss_ack;
	reg [63:0] ss_response;
	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, ss_data[73]};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on

	cdc_handshake #(.WIDTH(99), .RESP_WIDTH(64)) u_ss (
		.src_clk(ss_clk), .src_reset(ss_reset), .src_req(ena_i),
		.src_data({addr_i, din_i, be_i, rnw_i}), .src_busy(),
		.src_done(done_o), .src_resp(dout_o),
		.dst_clk(clk), .dst_reset(reset), .dst_valid(ss_valid),
		.dst_data(ss_data), .dst_ack(ss_ack), .dst_resp(ss_response)
	);

	wire ss_pending = ss_valid && !ss_taken_q;
	wire grant_ss = ss_pending && (!pr_req || prefer_ss_q);
	assign pr_ready = !reset && (state_q == ST_IDLE) && pr_req && !grant_ss;
	assign DDRAM_CLK = clk;
	assign DDRAM_ADDR = addr_q;
	assign DDRAM_DIN = din_q;
	assign DDRAM_BE = be_q;
	assign DDRAM_BURSTCNT = burst_q;
	assign DDRAM_RD = !reset && (state_q == ST_COMMAND) && rnw_q;
	assign DDRAM_WE = !reset && (state_q == ST_COMMAND) && !rnw_q;
	wire accepted = (DDRAM_RD || DDRAM_WE) && !DDRAM_BUSY;
	wire read_beat = DDRAM_DOUT_READY && ((state_q == ST_READ) ||
		((state_q == ST_COMMAND) && accepted && rnw_q));

	always @(posedge clk) begin
		ss_ack <= 1'b0;
		pr_valid <= 1'b0;
		pr_done <= 1'b0;
		if (reset) begin
			ss_taken_q <= 1'b0;
			prefer_ss_q <= 1'b1;
			discard_q <= 1'b1;
			ss_response <= 64'd0;
			pr_dout <= 64'd0;
			// A core reset cannot cancel an already accepted DDR read.
			if (state_q == ST_READ) begin
				if (DDRAM_DOUT_READY) begin
					remaining_q <= remaining_q - 8'd1;
					if (remaining_q == 8'd1) state_q <= ST_IDLE;
				end
			end else begin
				state_q <= ST_IDLE;
				owner_ss_q <= 1'b0;
				addr_q <= 29'd0;
				din_q <= 64'd0;
				be_q <= 8'd0;
				rnw_q <= 1'b0;
				burst_q <= 8'd1;
				remaining_q <= 8'd0;
			end
		end else begin
			// dst_valid stays high until the registered acknowledgement lands.
			if (!ss_valid) ss_taken_q <= 1'b0;
			case (state_q)
				ST_IDLE: begin
					discard_q <= 1'b0;
					if (grant_ss) begin
						addr_q <= {4'b0011, ss_data[98:74]};
						din_q <= ss_data[72:9];
						be_q <= ss_data[0] ? 8'hff : ss_data[8:1];
						rnw_q <= ss_data[0];
						burst_q <= 8'd1;
						remaining_q <= 8'd1;
						owner_ss_q <= 1'b1;
						prefer_ss_q <= 1'b0;
						ss_taken_q <= 1'b1;
						state_q <= ST_COMMAND;
					end else if (pr_ready) begin
						addr_q <= pr_addr;
						din_q <= pr_din;
						be_q <= pr_be;
						rnw_q <= pr_rnw;
						burst_q <= pr_burstcnt;
						remaining_q <= pr_burstcnt;
						owner_ss_q <= 1'b0;
						prefer_ss_q <= 1'b1;
						state_q <= ST_COMMAND;
					end
				end
				ST_COMMAND: begin
					if (accepted) begin
						if (rnw_q) state_q <= ST_READ;
						else begin
							state_q <= ST_IDLE;
							if (owner_ss_q) ss_ack <= 1'b1;
							else pr_done <= 1'b1;
						end
					end
				end
				default: begin end
			endcase
			// The first read beat may accompany command acceptance.
			if (read_beat) begin
				remaining_q <= remaining_q - 8'd1;
				if (!discard_q) begin
					if (owner_ss_q) ss_response <= DDRAM_DOUT;
					else begin
						pr_dout <= DDRAM_DOUT;
						pr_valid <= 1'b1;
					end
				end
				if (remaining_q == 8'd1) begin
					state_q <= ST_IDLE;
					if (!discard_q) begin
						if (owner_ss_q) ss_ack <= 1'b1;
						else pr_done <= 1'b1;
					end
				end
			end
		end
	end
endmodule
