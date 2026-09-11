// DDR-backed sticker image, with complete-column input and complete-row output.
module loopy_print_store (
	input  wire clk, reset, clear,
	input  wire cap_wr,
	input  wire [6:0] cap_y, cap_x,
	input  wire [1:0] cap_pass,
	input  wire [2:0] cap_ink,
	input  wire active, frame_start,
	input  wire [6:0] display_row,
	input  wire display_active,
	input  wire [12:0] rd_addr,
	output wire [17:0] rd_data,
	output wire rd_valid,
	output reg fault,
	output reg cmd_req,
	output reg [28:0] cmd_addr,
	output reg [63:0] cmd_din,
	output reg [7:0] cmd_be,
	output reg cmd_rnw,
	output reg [7:0] cmd_burstcnt,
	input  wire cmd_ready,
	/* verilator lint_off UNUSEDSIGNAL */
	input  wire [63:0] cmd_dout,
	/* verilator lint_on UNUSEDSIGNAL */
	input  wire cmd_valid, cmd_done
);
	localparam [28:0] DDR_BASE = 29'h07C80000; // Byte 0x3E400000, 64-bit words.
	localparam [2:0] IDLE = 3'd0, W_DATA = 3'd1, W_REQ = 3'd2,
	                 W_WAIT = 3'd3, R_REQ = 3'd4, R_WAIT = 3'd5,
	                 R_PUBLISH = 3'd6;
	reg [2:0] state;
	reg cancelled;

	// One slot holds one ink's 112 pixels. Only a complete column is published.
	reg [3:0] producer, consumer;
	reg [6:0] queue_x [0:7];
	reg [1:0] queue_pass [0:7];
	reg collecting;
	reg [6:0] next_y, collect_x;
	reg [1:0] collect_pass;
	reg [6:0] write_y;
	reg [6:0] write_x;
	reg [1:0] write_pass;
	reg last_write;
	reg [127:0] committed [0:2];
	reg [127:0] visible [0:2];
	wire [3:0] queued = producer - consumer;
	wire queue_full = queued == 4'd8;
	wire write_finishes = ((state == W_REQ && cmd_ready) || state == W_WAIT) && cmd_done;
	wire pop_column = write_finishes && write_y == 7'd111 && !cancelled && !clear && !fault;
	wire column_start = cap_y == 7'd0 && cap_pass < 2'd3;
	wire column_next = collecting && cap_y == next_y && cap_x == collect_x && cap_pass == collect_pass;
	wire overflow = cap_wr && column_start && queue_full && !pop_column && !clear && !fault;
	// Within a job the producer writes each {pass,x} once; a restore clears it.
	wire duplicate = state == IDLE && queued != 4'd0 && write_y == 7'd0
	                 && committed[queue_pass[consumer[2:0]]][queue_x[consumer[2:0]]];
	wire invalidate = clear || fault || overflow || duplicate;
	wire queue_write = cap_wr && !reset && !invalidate && (!queue_full || pop_column)
	                   && (column_start || column_next) && cap_y < 7'd112;
	wire [2:0] queue_data;
	cache_ram_tdp_dc #(.ADDR_WIDTH(10), .DATA_WIDTH(3)) column_queue (
		.clk_a_i(clk), .addr_a_i({producer[2:0], cap_y}),
		.wren_a_i(queue_write), .wdata_a_i(cap_ink),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o(),
		/* verilator lint_on PINCONNECTEMPTY */
		.clk_b_i(clk), .addr_b_i({consumer[2:0], write_y}),
		.wren_b_i(1'b0), .wdata_b_i(3'd0), .q_b_o(queue_data)
	);

	reg maps_dirty;
	wire new_maps = frame_start && maps_dirty;
	reg [7:0] cache_valid;
	reg [6:0] cache_row [0:7];
	reg [6:0] rd_row_q;
	reg [6:0] base_row, fill_row;
	reg [2:0] scan;
	reg [5:0] fill_word;
	reg fill_bad;
	// Validity bits shift two per DDR beat, so the fill needs no wide indexed
	// bitmap mux and every pixel in a row uses one frame's snapshot.
	reg [127:0] mask_y, mask_m, mask_c;
	wire [7:0] candidate_sum = {1'b0, base_row} + {5'd0, scan};
	/* verilator lint_off UNUSEDSIGNAL */
	wire [7:0] candidate_wrap = candidate_sum >= 8'd112 ? candidate_sum - 8'd112 : candidate_sum;
	/* verilator lint_on UNUSEDSIGNAL */
	wire [6:0] candidate = candidate_wrap[6:0];
	wire candidate_protected = display_active && candidate[2:0] == display_row[2:0];
	wire need_row = active && !candidate_protected
	                && (!cache_valid[candidate[2:0]] || cache_row[candidate[2:0]] != candidate);
	wire fill_protected = display_active && fill_row[2:0] == display_row[2:0];
	wire read_owned = state == R_WAIT || (state == R_REQ && cmd_ready);
	wire cache_write = read_owned && cmd_valid && !reset && !invalidate
	                   && !fill_bad && !new_maps && !fill_protected;
	wire [8:0] pixel0 = {cmd_dout[18:16] & {3{mask_c[0]}},
	                      cmd_dout[10:8] & {3{mask_m[0]}},
	                      cmd_dout[2:0] & {3{mask_y[0]}}};
	wire [8:0] pixel1 = {cmd_dout[50:48] & {3{mask_c[1]}},
	                      cmd_dout[42:40] & {3{mask_m[1]}},
	                      cmd_dout[34:32] & {3{mask_y[1]}}};
	cache_ram_tdp_dc #(.ADDR_WIDTH(9), .DATA_WIDTH(18)) row_cache (
		.clk_a_i(clk), .addr_a_i({fill_row[2:0], fill_word}),
		.wren_a_i(cache_write), .wdata_a_i({pixel1, pixel0}),
		/* verilator lint_off PINCONNECTEMPTY */
		.q_a_o(),
		/* verilator lint_on PINCONNECTEMPTY */
		.clk_b_i(clk), .addr_b_i(rd_addr[8:0]),
		.wren_b_i(1'b0), .wdata_b_i(18'd0), .q_b_o(rd_data)
	);
	// The tag follows the same one-clock address latency as the RAM output.
	assign rd_valid = active && !reset && !invalidate && cache_valid[rd_row_q[2:0]]
	                  && cache_row[rd_row_q[2:0]] == rd_row_q;
	wire [2:0] byte_lane = {write_x[0], write_pass};
	integer i;
	always @(posedge clk) begin
		if (reset) begin
			state <= IDLE; cancelled <= 1'b0; fault <= 1'b0;
			cmd_req <= 1'b0; cmd_addr <= DDR_BASE; cmd_din <= 64'd0;
			cmd_be <= 8'd0; cmd_rnw <= 1'b0; cmd_burstcnt <= 8'd1;
			producer <= 4'd0; consumer <= 4'd0;
			collecting <= 1'b0; next_y <= 7'd0; collect_x <= 7'd0; collect_pass <= 2'd0;
			write_y <= 7'd0; write_x <= 7'd0; write_pass <= 2'd0; last_write <= 1'b0;
			maps_dirty <= 1'b0; cache_valid <= 8'd0;
			rd_row_q <= 7'd0; base_row <= 7'd0; fill_row <= 7'd0;
			scan <= 3'd0; fill_word <= 6'd0; fill_bad <= 1'b0;
			mask_y <= 128'd0; mask_m <= 128'd0; mask_c <= 128'd0;
			for (i = 0; i < 3; i = i + 1) begin
				committed[i] <= 128'd0; visible[i] <= 128'd0;
			end
			for (i = 0; i < 8; i = i + 1) begin
				queue_x[i] <= 7'd0; queue_pass[i] <= 2'd0; cache_row[i] <= 7'd0;
			end
		end else begin
			rd_row_q <= rd_addr[12:6];
			if (display_active) base_row <= display_row < 7'd112 ? display_row : 7'd0;
			if (frame_start) begin
				base_row <= 7'd0; scan <= 3'd0;
				if (maps_dirty) begin
					for (i = 0; i < 3; i = i + 1) visible[i] <= committed[i];
					maps_dirty <= 1'b0;
					cache_valid <= 8'd0;
					fill_bad <= 1'b1;
				end
			end

			if (queue_write) begin
				if (column_start) begin
					queue_x[producer[2:0]] <= cap_x;
					queue_pass[producer[2:0]] <= cap_pass;
					collect_x <= cap_x; collect_pass <= cap_pass;
					collecting <= 1'b1; next_y <= 7'd1;
				end else begin
					next_y <= next_y + 7'd1;
					if (cap_y == 7'd111) begin
						producer <= producer + 4'd1;
						collecting <= 1'b0;
					end
				end
			end else if (cap_wr && !column_start) collecting <= 1'b0;

			case (state)
			IDLE: begin
				cancelled <= 1'b0;
				scan <= scan + 3'd1;
				if (need_row && (queued == 4'd0 || last_write) && !new_maps) begin
					fill_row <= candidate; fill_word <= 6'd0; fill_bad <= 1'b0;
					mask_y <= visible[0]; mask_m <= visible[1]; mask_c <= visible[2];
					cache_valid[candidate[2:0]] <= 1'b0;
					cmd_addr <= DDR_BASE + {16'd0, candidate, 6'd0};
					cmd_din <= 64'd0; cmd_be <= 8'hFF;
					cmd_rnw <= 1'b1; cmd_burstcnt <= 8'd64; cmd_req <= 1'b1;
					state <= R_REQ;
				end else if (queued != 4'd0) begin
					write_x <= queue_x[consumer[2:0]];
					write_pass <= queue_pass[consumer[2:0]];
					state <= W_DATA;
				end
			end
			W_DATA: begin
				cmd_addr <= DDR_BASE + {16'd0, write_y, write_x[6:1]};
				cmd_din <= {56'd0, 5'd0, queue_data} << {byte_lane, 3'd0};
				cmd_be <= 8'b00000001 << byte_lane;
				cmd_rnw <= 1'b0; cmd_burstcnt <= 8'd1; cmd_req <= 1'b1;
				state <= W_REQ;
			end
			W_REQ: if (cmd_ready) begin cmd_req <= 1'b0; state <= W_WAIT; end
			R_REQ: if (cmd_ready) begin cmd_req <= 1'b0; state <= R_WAIT; end
			R_PUBLISH: begin
				if (!fill_bad && !new_maps && !fill_protected) begin
					cache_row[fill_row[2:0]] <= fill_row;
					cache_valid[fill_row[2:0]] <= 1'b1;
				end
				state <= IDLE;
			end
			default: ;
			endcase

			if (write_finishes) begin
				state <= IDLE; last_write <= 1'b1;
				if (!cancelled) begin
					if (write_y == 7'd111) begin
						committed[write_pass][write_x] <= 1'b1;
						maps_dirty <= 1'b1;
						consumer <= consumer + 4'd1; write_y <= 7'd0;
					end else write_y <= write_y + 7'd1;
				end
			end
			if (read_owned) begin
				if (fill_protected) fill_bad <= 1'b1;
				if (cmd_valid) begin
					fill_word <= fill_word + 6'd1;
					mask_y <= {2'd0, mask_y[127:2]};
					mask_m <= {2'd0, mask_m[127:2]};
					mask_c <= {2'd0, mask_c[127:2]};
				end
				if (cmd_done) begin
					state <= R_PUBLISH; last_write <= 1'b0;
					if (!cmd_valid || fill_word != 6'd63) begin
						fault <= 1'b1; fill_bad <= 1'b1;
					end
				end
			end

			// Invalidation drops unsent work. Accepted commands still drain so a
			// late response cannot be mistaken for data from the next job.
			if (invalidate) begin
				fault <= clear ? 1'b0 : 1'b1;
				producer <= 4'd0; consumer <= 4'd0; collecting <= 1'b0;
				write_y <= 7'd0; maps_dirty <= 1'b0; cache_valid <= 8'd0;
				base_row <= 7'd0; scan <= 3'd0; fill_bad <= 1'b1; cancelled <= 1'b1;
				for (i = 0; i < 3; i = i + 1) begin
					committed[i] <= 128'd0; visible[i] <= 128'd0;
				end
				cmd_req <= 1'b0;
				case (state)
				W_REQ: state <= cmd_ready && !cmd_done ? W_WAIT : IDLE;
				W_WAIT: if (cmd_done) state <= IDLE;
				R_REQ: state <= cmd_ready && !cmd_done ? R_WAIT : IDLE;
				R_WAIT: if (cmd_done) state <= IDLE;
				default: state <= IDLE;
				endcase
			end
		end
	end
endmodule
