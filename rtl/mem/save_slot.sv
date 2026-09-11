// Copyright (c) 2026 Jamie Blanks

module save_slot #(
	parameter integer MAX_BLOCKS = 4
) (
	input         clk_sys,
	input         invalidate_pulse,
	input         mount_pulse,
	input         mount_readonly,
	input  [63:0] mount_size,
	input  [31:0] block_limit,
	input         load_req,
	input         save_req,
	input         autosave_disable,
	input         osd_status,
	input         dirty_pulse,
	input         transfer_allowed,
	output reg    mounted_writable = 1'b0,
	output reg    pending = 1'b0,
	output reg    busy = 1'b0,
	output wire   load_active,
	output wire [31:0] sd_lba,
	output reg    sd_rd = 1'b0,
	output reg    sd_wr = 1'b0,
	input         sd_ack
);

	localparam integer BLOCK_BITS = $clog2(MAX_BLOCKS + 1);
	localparam [BLOCK_BITS-1:0] MAX_BLOCKS_VALUE = MAX_BLOCKS[BLOCK_BITS-1:0];
	localparam [31:0] MAX_BLOCKS_32 = MAX_BLOCKS[31:0];
	wire [BLOCK_BITS-1:0] block_limit_w =
		(block_limit == 32'd0) || (block_limit > MAX_BLOCKS_32) ?
		MAX_BLOCKS_VALUE : block_limit[BLOCK_BITS-1:0];

	reg old_mount_pulse = 1'b0;
	reg old_load_req = 1'b0;
	reg old_save_req = 1'b0;
	reg old_osd_status = 1'b0;
	reg old_sd_ack = 1'b0;
	reg loading = 1'b0;
	reg transfer_started = 1'b0;
	reg save_dirty_q = 1'b0;
	reg [BLOCK_BITS-1:0] mounted_blocks = '0;
	reg [BLOCK_BITS-1:0] sd_block_q = '0;
	reg mount_readonly_latched = 1'b0;
	reg mount_info_pending = 1'b0;
	wire [BLOCK_BITS-1:0] active_blocks_w =
		(mounted_blocks > block_limit_w) ? block_limit_w : mounted_blocks;

	wire [BLOCK_BITS:0] next_sd_block_w =
		{1'b0, sd_block_q} + {{BLOCK_BITS{1'b0}}, 1'b1};
	assign sd_lba = {{(32-BLOCK_BITS){1'b0}}, sd_block_q};
	assign load_active = busy && loading;

	function automatic [BLOCK_BITS-1:0] blocks_from_size(input [63:0] size_bytes);
		reg [BLOCK_BITS:0] rounded_blocks;
		reg size_over_range;
	begin
		size_over_range = |size_bytes[63:BLOCK_BITS+9];
		rounded_blocks =
			{1'b0, size_bytes[BLOCK_BITS+8:9]} +
			{{BLOCK_BITS{1'b0}}, |size_bytes[8:0]};
		if (size_over_range ||
			(rounded_blocks > {1'b0, MAX_BLOCKS_VALUE})) begin
			blocks_from_size = MAX_BLOCKS_VALUE;
		end else begin
			blocks_from_size = rounded_blocks[BLOCK_BITS-1:0];
		end
	end
	endfunction

	always @(posedge clk_sys) begin
		reg mount_rise;
		reg mount_fall;
		reg load_rise;
		reg save_rise;
		reg osd_rise;
		reg autosave_now;
		reg create_unmounted_save;
		reg finish_mount;
		reg start_load;
		reg [BLOCK_BITS-1:0] next_blocks;

		mount_rise = mount_pulse && !old_mount_pulse;
		mount_fall = old_mount_pulse && !mount_pulse;
		load_rise = load_req && !old_load_req;
		save_rise = save_req && !old_save_req;
		osd_rise = osd_status && !old_osd_status;
		// Opening the OSD starts one pending autosave.
		autosave_now = pending && osd_rise && !autosave_disable;
		next_blocks = blocks_from_size(mount_size);
		create_unmounted_save = !mounted_writable && !mount_readonly_latched && (mounted_blocks == '0);
		finish_mount = mount_info_pending && !mount_pulse && (next_blocks != '0);
		start_load = (mounted_blocks != '0) && load_rise;

		old_mount_pulse <= mount_pulse;
		old_load_req <= load_req;
		old_save_req <= save_req;
		old_osd_status <= osd_status;
		old_sd_ack <= sd_ack;

		if (invalidate_pulse) begin
			mounted_writable <= 1'b0;
			pending <= 1'b0;
			busy <= 1'b0;
			loading <= 1'b0;
			transfer_started <= 1'b0;
			save_dirty_q <= 1'b0;
			mounted_blocks <= '0;
			mount_readonly_latched <= 1'b0;
			mount_info_pending <= 1'b0;
			sd_block_q <= '0;
			sd_rd <= 1'b0;
			sd_wr <= 1'b0;
		end

		if (!invalidate_pulse) begin
			if (dirty_pulse && !osd_status && !busy) begin
				if (mounted_writable) begin
					pending <= 1'b1;
				end else if (create_unmounted_save) begin
					mounted_writable <= 1'b1;
					mounted_blocks <= MAX_BLOCKS_VALUE;
					mount_info_pending <= 1'b0;
					pending <= 1'b1;
				end
			end else if (busy && !loading) begin
				pending <= 1'b0;
			end

			if (busy && !loading && dirty_pulse) begin
				save_dirty_q <= 1'b1;
			end
		end

		if (mount_rise) begin
			// Clear state when mounting starts, then save size and mode when it ends.
			pending <= 1'b0;
			busy <= 1'b0;
			loading <= 1'b0;
			transfer_started <= 1'b0;
			save_dirty_q <= 1'b0;
			mounted_writable <= 1'b0;
			mounted_blocks <= '0;
			mount_readonly_latched <= mount_readonly;
			mount_info_pending <= 1'b1;
			sd_block_q <= '0;
			sd_rd <= 1'b0;
			sd_wr <= 1'b0;
		end else if (!invalidate_pulse) begin
			if (mount_pulse) begin
				mount_readonly_latched <= mount_readonly;
			end

			if (mount_fall) begin
				pending <= 1'b0;
				busy <= 1'b0;
				loading <= 1'b0;
				transfer_started <= 1'b0;
				save_dirty_q <= 1'b0;
				sd_block_q <= '0;
				sd_rd <= 1'b0;
				sd_wr <= 1'b0;
			end

			if (finish_mount) begin
				mounted_blocks <= next_blocks;
				mounted_writable <= !mount_readonly_latched;
				mount_info_pending <= 1'b0;
				pending <= 1'b0;
				busy <= 1'b0;
				loading <= 1'b0;
				transfer_started <= 1'b0;
				save_dirty_q <= 1'b0;
				sd_block_q <= '0;
				sd_rd <= 1'b0;
				sd_wr <= 1'b0;

				// Read-only images are still restored into SRAM;
				// mounted_writable separately blocks dirty/save.
				busy <= 1'b1;
				loading <= 1'b1;
				transfer_started <= 1'b0;
				sd_rd <= 1'b0;
			end else if (!old_sd_ack && sd_ack) begin
				sd_rd <= 1'b0;
				sd_wr <= 1'b0;
			end else if (busy && !transfer_started) begin
				// Saves wait for port B; loads also wait for a full frame stop.
				if (transfer_allowed) begin
					transfer_started <= 1'b1;
					sd_rd <= loading;
					sd_wr <= !loading;
				end
			end else if (busy) begin
				if (old_sd_ack && !sd_ack) begin
					if (next_sd_block_w >= {1'b0, active_blocks_w}) begin
						// If SRAM changes during backup, finish and schedule another pass.
						pending <= !loading && (save_dirty_q || dirty_pulse);
						busy <= 1'b0;
						loading <= 1'b0;
						transfer_started <= 1'b0;
						save_dirty_q <= 1'b0;
						sd_block_q <= '0;
					end else begin
						sd_block_q <= next_sd_block_w[BLOCK_BITS-1:0];
						sd_rd <= loading;
						sd_wr <= !loading;
					end
				end
			end else if (!mount_pulse &&
				(start_load ||
				 (mounted_writable && (save_rise || autosave_now)) ||
				 (save_rise && create_unmounted_save))) begin
				if (save_rise && create_unmounted_save) begin
					mounted_writable <= 1'b1;
					mounted_blocks <= MAX_BLOCKS_VALUE;
					mount_info_pending <= 1'b0;
				end
				busy <= 1'b1;
				loading <= start_load;
				transfer_started <= transfer_allowed && !start_load;
				save_dirty_q <= dirty_pulse && !start_load;
				sd_block_q <= '0;
				sd_rd <= 1'b0;
				sd_wr <= transfer_allowed && !start_load;
			end
		end
	end

endmodule
