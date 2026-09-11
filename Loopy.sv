//============================================================================
//
// Casio Loopy for MiSTer: framework glue.
//
// Nothing of the machine is modelled here. This file makes the three clocks,
// hands content from the HPS to the loader, wires the SDRAM, and takes the
// RH-7500's picture out to the framework's video path. The console itself is
// rtl/mainboard/loopy_mainboard.sv and the chips under rtl/.
//
// Clocks (decisions 0016 and 0025): one PLL, three outputs.
//
//   clk_ram    128.863636 MHz   SDRAM, four times clk_sys
//   clk_video   42.954545 MHz   RH-7500
//   clk_sys     32.215909 MHz   SH7021, sound, loaders, this file
//
// clk_video and clk_sys are what the machine needs. clk_ram is a whole four
// times clk_sys, which is the ratio worth having: it is what lets the memory
// path drop cdc_handshake later, and that handshake is most of what the CPU
// spends stalled. It has no relationship to clk_video and must not be given
// one - nothing in the video pipeline touches SDRAM, whose only clients are
// the cartridge ROM, the work DRAM and the wave ROM, all on the CPU side
// (decisions 0027 and 0028).
//
//============================================================================

module emu
(
	`include "sys/emu_ports.vh"
);

assign ADC_BUS  = 'Z;
assign USER_OUT = '1;
assign {UART_RTS, UART_TXD, UART_DTR} = '0;
assign {SD_SCK, SD_MOSI, SD_CS} = 'Z;

`ifdef MISTER_DUAL_SDRAM
assign {SDRAM2_CLK, SDRAM2_A, SDRAM2_BA, SDRAM2_DQ, SDRAM2_nCS,
	SDRAM2_nCAS, SDRAM2_nRAS, SDRAM2_nWE} = 'Z;
`endif

assign VGA_F1       = 1'b0;
assign VGA_SCALER   = 1'b0;
assign VGA_DISABLE  = 1'b0;
assign HDMI_FREEZE  = 1'b0;
assign HDMI_BLACKOUT = 1'b0;
assign HDMI_BOB_DEINT = 1'b0;

// The synth's output is signed 16-bit and the framework samples it at 48 kHz.
// It needs no anti-alias stage of its own: the board's own 8.2 kHz low pass
// leaves 67 to 76 dB below the total above 24 kHz (LOOPY_SOUND_FACTS section 6).
assign AUDIO_S   = 1'b1;
assign AUDIO_MIX = 2'd0;

wire bk_pending, bk_busy;
assign LED_DISK  = {1'b1, bk_pending | bk_busy};
assign LED_POWER = 2'd0;
assign BUTTONS   = 2'd0;

// Declared here because Icarus wants a symbol before its first use, and the
// aspect-ratio wire below reads status.
wire [1:0] buttons;
wire [127:0] status;
wire loading;

// Backup RAM. The framework moves 512-byte blocks through sd_buff, which is
// 16 bits wide in WIDE mode, and the cartridge SRAM's port B is the same
// width, so the two are wired together with no staging buffer in between.
wire        img_mounted, img_readonly;
wire [63:0] img_size;
wire [31:0] sd_lba;
wire        sd_rd, sd_wr, sd_ack;
wire  [7:0] sd_buff_addr;
wire [15:0] sd_buff_dout;
wire        sd_buff_wr;
wire        bk_writable;
wire [15:0] sram_b_q;

// Savestates. Declared here for the same reason as the block above: hps_io is
// instantiated before the logic that drives these.
wire [10:0] ps2_key;
wire        ss_info_req;
wire [7:0]  ss_info;
wire        ss_status_update;
wire [1:0]  ss_slot;
wire        ss_pause_sys, ss_pause_video;
wire        ss_bus_rst;

wire clk_ram;
wire clk_video;
wire clk_sys;
wire pll_locked;

wire [1:0] ar = status[122:121];

wire        forced_scandoubler;
wire [21:0] gamma_bus;

`include "build_id.v"
localparam CONF_STR = {
	"Loopy;SS3E000000:100000;",
	"FS1,BIN,Load Cartridge;",
	"-;",
	"P3O[19:18],Savestate slot,1,2,3,4;",
	"P3R[20],Save state (Alt-F1);",
	"P3R[21],Restore state (F1);",
	"-;",
	"P1,Audio & Video;",
	"P1-;",
	"P1O[122:121],Aspect ratio,Original,Full Screen,[ARC1],[ARC2];",
	"P1O[24:22],Scandoubler Fx,None,HQ2x,CRT 25%,CRT 50%,CRT 75%;",
	"P1O[27:25],Scale,Normal,V-Integer,Narrower HV-Integer,Wider HV-Integer;",
	"d1P1O[4],Vertical Crop,Disabled,216p(5x);",
	"P1O[5],Border,Crop,Show;",
	"P2,Input;",
	"P2-;",
	"P2O[12],Port 1 device,Gamepad,Mouse;",
	"P2O[14:13],Multitap,Off,2 pads,3 pads,4 pads;",
	"P3,Advanced;",
	"P3-;",
	"P3O[15],Autosave,On,Off;",
	"h0P3T[16],Save Backup RAM;",
	"h0P3T[17],Restore Backup RAM;",
	// Fitting the cassette makes the BIOS home the mechanism at reset - 540
	// steps forward and 530 back at 2 ms each - so the console takes about two
	// seconds longer to boot. That is what the real machine does.
	"P4,Printer;",
	"P4-;",
	"P4O[7],Seal cassette,Absent,Fitted;",
	"P4O[9:8],Print preview,Off,Corner,Full;",
	"P4O[11:10],Ink order,YMC,MCY,CYM,CMY;",
	"-;",
	"R[0],Reset;",
	"-;",
	"J1,A,B,C,D,L,R,Start,Savestates;",
	"jn,A,B,X,Y,L,R,Start;",
	"I,",
	// savestate_ui numbers its messages from 5, so the list starts with four
	// spacers this core has no use for.
	"-,",
	"-,",
	"-,",
	"-,",
	"Slot=L/R Load=U Save=D,",
	"Active Slot 1,",
	"Active Slot 2,",
	"Active Slot 3,",
	"Active Slot 4,",
	"Save to state 1,",
	"Restore state 1,",
	"Save to state 2,",
	"Restore state 2,",
	"Save to state 3,",
	"Restore state 3,",
	"Save to state 4,",
	"Restore state 4;",
	"v,1;",
	"V,v",`BUILD_DATE
};

wire [31:0] joystick_0, joystick_1, joystick_2, joystick_3;
wire [24:0] ps2_mouse;

wire        ioctl_download;
wire [15:0] ioctl_index;
wire        ioctl_wr;
wire [26:0] ioctl_addr;
wire [15:0] ioctl_dout;
wire        ioctl_wait;

// WIDE puts the HPS transfer on a 16-bit word, which is the Loopy's bus width
// and what loopy_loader expects.
hps_io #(.CONF_STR(CONF_STR), .WIDE(1)) hps_io
(
	.clk_sys            (clk_sys),
	.HPS_BUS            (HPS_BUS),
	.EXT_BUS            (),
	.gamma_bus          (gamma_bus),
	.forced_scandoubler (forced_scandoubler),
	.buttons            (buttons),
	.status             (status),
	.status_in          ({status[127:20], ss_slot, status[17:0]}),
	.status_set         (ss_status_update),
	.ps2_key            (ps2_key),
	.info_req           (ss_info_req),
	.info               (ss_info),
	// Bit 0 hides the two backup-RAM entries while nothing writable is
	// mounted, so the OSD never offers a save with nowhere to put it.
	.status_menumask    ({14'd0, en216p, ~bk_writable}),

	.joystick_0         (joystick_0),
	.joystick_1         (joystick_1),
	.joystick_2         (joystick_2),
	.joystick_3         (joystick_3),
	.ps2_mouse          (ps2_mouse),

	.ioctl_download     (ioctl_download),
	.ioctl_index        (ioctl_index),
	.ioctl_wr           (ioctl_wr),
	.ioctl_addr         (ioctl_addr),
	.ioctl_dout         (ioctl_dout),
	.ioctl_wait         (ioctl_wait),
	.ioctl_upload_req   (1'b0),
	.ioctl_upload_index (8'd0),
	.ioctl_din          (16'd0),

	.img_mounted        (img_mounted),
	.img_readonly       (img_readonly),
	.img_size           (img_size),

	.sd_lba             ('{sd_lba}),
	.sd_blk_cnt         ('{6'd0}),
	.sd_rd              (sd_rd),
	.sd_wr              (sd_wr),
	.sd_ack             (sd_ack),
	.sd_buff_addr       (sd_buff_addr),
	.sd_buff_dout       (sd_buff_dout),
	.sd_buff_din        ('{sram_b_q}),
	.sd_buff_wr         (sd_buff_wr)
);

////////////////////////////  CLOCKS  /////////////////////////////////////////

pll pll
(
	.refclk   (CLK_50M),
	.rst      (1'b0),
	.outclk_0 (clk_ram),
	.outclk_1 (clk_video),
	.outclk_2 (clk_sys),
	.locked   (pll_locked)
);

// Two resets. The cold one is the framework's: a new core, a new file, the
// PLL not locked. The other is the console's front panel, and it must not
// reach the loader - resetting a Loopy does not eject its cartridge, and the
// BIOS refuses to boot when DET reads low.
wire cold_reset = RESET | ~pll_locked;
// The machine is held in reset while content arrives. Without that the CPU
// runs on whatever the memories happen to hold and writes nonsense into the
// VDP and the work DRAM before the real boot starts. The clock enables keep
// running throughout - see rtl/loopy_clocks.sv.

// One reset synchroniser per domain: asserted whenever, released in step with
// the clock that will use it.
//
// The assertion must not depend on the domain's own clock. Registers power up
// at zero, so a purely clocked chain reads "not in reset" until two edges have
// arrived - and at configuration, before the PLL locks, those edges do not
// exist. Every domain would come up nominally released at exactly the moment
// nothing is stable. `cold_reset` carries `~pll_locked` and is the one source
// genuinely asynchronous to all three clocks, so it asserts through the reset
// pin; release stays synchronous, two flops deep.
//
// The soft sources - the OSD reset, the front panel, and a load in progress -
// stay on the synchronous path. They only matter while the clocks are already
// running, and putting a combinational decode on an asynchronous reset pin
// would let a glitch reset a domain for no reason.
//
// The memory path - the bridge and the controller - is reset only by a cold
// start, because content arrives while the machine is held in reset and the
// loader writes through it. The controller takes its reset asynchronously and
// everything else takes it synchronously; that is the intent.
// The savestate's restore hold is a machine reset like any other: every block
// reloads from its saved words on the last reset cycle before it lets go.
wire soft_reset = status[0] | buttons[1] | loading | ss_restore_hold;

reg [1:0] rst_sys_q = 2'b11, rst_video_q = 2'b11, rst_ram_q = 2'b11, rst_cold_q = 2'b11;

always @(posedge clk_sys or posedge cold_reset)
	if (cold_reset) rst_cold_q  <= 2'b11;
	else            rst_cold_q  <= {rst_cold_q[0],  1'b0};

always @(posedge clk_sys or posedge cold_reset)
	if (cold_reset) rst_sys_q   <= 2'b11;
	else            rst_sys_q   <= {rst_sys_q[0],   soft_reset};

always @(posedge clk_video or posedge cold_reset)
	if (cold_reset) rst_video_q <= 2'b11;
	else            rst_video_q <= {rst_video_q[0], soft_reset};

always @(posedge clk_ram or posedge cold_reset)
	if (cold_reset) rst_ram_q   <= 2'b11;
	else            rst_ram_q   <= {rst_ram_q[0],   1'b0};

// The savestate path needs the cold reset in the video domain too: a restore
// holds the machine in reset for the whole transfer, and the walk that puts
// the VDP's memories back has to run through it.
reg [1:0] rst_cold_v_q = 2'b11;
always @(posedge clk_video or posedge cold_reset)
	if (cold_reset) rst_cold_v_q <= 2'b11;
	else            rst_cold_v_q <= {rst_cold_v_q[0], 1'b0};
wire reset_cold_video = rst_cold_v_q[1];

wire reset_cold  = rst_cold_q[1];
wire reset_sys   = rst_sys_q[1];
wire reset_video = rst_video_q[1];
wire reset_ram   = rst_ram_q[1];

wire ce_vdp, ce_cpu_r, ce_cpu_f, ce_midi4m, ce_sample;
wire mem_stall;

loopy_clocks clocks
(
	.clk_video   (clk_video),
	.pause_video (ss_pause_video),
	.ce_vdp      (ce_vdp),

	.clk_sys     (clk_sys),
	.pause_sys   (ss_pause_sys),
	.mem_stall   (mem_stall),
	.ce_cpu_r    (ce_cpu_r),
	.ce_cpu_f    (ce_cpu_f),
	.ce_midi4m   (ce_midi4m),
	.ce_sample   (ce_sample)
);

////////////////////////////  CONTENT  ////////////////////////////////////////

wire        ld_req, ld_busy;
wire [25:0] ld_addr;
wire [15:0] ld_din;
wire [21:1] cart_rom_mask;
wire        cart_rom_split;
wire [20:1] cart_rom_hi_mask;
wire [16:0] cart_sram_mask;

wire        bios_wr;
wire [13:0] bios_addr;
wire [15:0] bios_data;

wire        cart_loaded;

loopy_loader loader
(
	.clk             (clk_sys),
	.reset           (reset_cold),

	.ioctl_download  (ioctl_download),
	.ioctl_index     (ioctl_index[7:0]),
	.ioctl_wr        (ioctl_wr),
	.ioctl_addr      (ioctl_addr),
	.ioctl_dout      (ioctl_dout),
	.ioctl_wait      (ioctl_wait),

	.ld_req          (ld_req),
	.ld_addr         (ld_addr),
	.ld_din          (ld_din),
	.ld_busy         (ld_busy),
	.loading         (loading),

	.bios_wr         (bios_wr),
	.bios_addr       (bios_addr),
	.bios_data       (bios_data),

	.cart_loaded     (cart_loaded),
	.cart_mask       (cart_rom_mask),
	.cart_split      (cart_rom_split),
	.cart_hi_mask    (cart_rom_hi_mask),
	.sram_mask       (cart_sram_mask)
);

////////////////////////////  THE MACHINE  ///////////////////////////////////

wire        p0_req, p0_we, p1_req, p2_req;
wire [25:0] p0_addr, p1_addr, p2_addr;
wire [63:0] p0_din, p0_dout, p1_dout, p2_dout;
wire [7:0]  p0_byte_en;
wire        p0_busy, p0_ready, p1_busy, p1_ready, p2_busy, p2_ready;

wire        ce_pix;
wire [14:0] rgb;
wire        hsync, vsync, hvisible, vvisible, hrender, vrender, render;
wire        snd_idle;

// The BIOS arrives as 16-bit words and the on-chip ROM is 32 bits wide, so
// the odd word is held back and the pair is written together. The loader hands
// over words with the file's byte order (low byte first) and the ROM is read
// big-endian - the byte at H'0000000 is the most significant of the longword -
// so each half is swapped on the way in.
reg         bios_wr32;
reg  [12:0] bios_wa32;
reg  [31:0] bios_wd32;
reg  [15:0] bios_hi;
always @(posedge clk_sys) begin
	bios_wr32 <= 1'b0;
	if (bios_wr) begin
		if (!bios_addr[0]) begin
			bios_hi <= bios_data;
		end else begin
			bios_wr32 <= 1'b1;
			bios_wa32 <= bios_addr[13:1];
			bios_wd32 <= {bios_hi[7:0], bios_hi[15:8],
			              bios_data[7:0], bios_data[15:8]};
		end
	end
end

wire [12:0] print_rd_addr;
wire [17:0] print_rd_data;
wire        print_rd_valid, print_fault;
wire        print_clear, print_cap_wr;
wire [6:0]  print_cap_y, print_cap_x;
wire [1:0]  print_cap_pass;
wire [2:0]  print_cap_ink;
wire [6:0]  print_display_row;
wire        print_display_active, print_frame_start;
wire        print_cmd_req, print_cmd_ready, print_cmd_rnw;
wire [28:0] print_cmd_addr;
wire [63:0] print_cmd_din, print_cmd_dout;
wire [7:0]  print_cmd_be, print_cmd_burstcnt;
wire        print_cmd_valid, print_cmd_done;
wire        print_show;

loopy_mainboard mainboard
(
	.clk_sys          (clk_sys),
	.reset_sys        (reset_sys),
	.ce_cpu_r         (ce_cpu_r),
	.ce_cpu_f         (ce_cpu_f),
	.ce_4m            (ce_midi4m),
	.ce_sample        (ce_sample),
	.clk_video        (clk_video),
	.reset_video      (reset_video),
	.ce_vdp           (ce_vdp),
	.clk_ram          (clk_ram),
	.reset_mem_sys    (reset_cold),
	.reset_ss_sys     (reset_cold),
	.reset_ss_video   (reset_cold_video),
	.reset_mem_ram    (reset_ram),

	.rom_wr_en        (bios_wr32),
	.rom_wr_addr      (bios_wa32),
	.rom_wr_data      (bios_wd32),

	.cart_det         (cart_loaded),
	.cart_rom_mask    (cart_rom_mask),
	.cart_rom_split   (cart_rom_split),
	.cart_rom_hi_mask (cart_rom_hi_mask),
	.cart_sram_mask   (cart_sram_mask),

	.loading          (loading),
	.ld_req           (ld_req),
	.ld_addr          (ld_addr),
	.ld_din           (ld_din),
	.ld_busy          (ld_busy),

	.p0_req           (p0_req),
	.p0_we            (p0_we),
	.p0_addr          (p0_addr),
	.p0_din           (p0_din),
	.p0_byte_en       (p0_byte_en),
	.p0_dout          (p0_dout),
	.p0_busy          (p0_busy),
	.p0_ready         (p0_ready),

	.p1_req           (p1_req),
	.p1_addr          (p1_addr),
	.p1_dout          (p1_dout),
	.p1_busy          (p1_busy),
	.p1_ready         (p1_ready),

	.p2_req           (p2_req),
	.p2_addr          (p2_addr),
	.p2_dout          (p2_dout),
	.p2_busy          (p2_busy),
	.p2_ready         (p2_ready),

	.mem_stall        (mem_stall),

	.ce_pix           (ce_pix),
	.rgb              (rgb),
	.hsync            (hsync),
	.vsync            (vsync),
	.hvisible         (hvisible),
	.vvisible         (vvisible),
	.hrender          (hrender),
	.vrender          (vrender),
	.render           (render),

	.print_cassette   (status[7]),
	.print_clear      (print_clear),
	.print_cap_wr     (print_cap_wr),
	.print_cap_y      (print_cap_y),
	.print_cap_x      (print_cap_x),
	.print_cap_pass   (print_cap_pass),
	.print_cap_ink    (print_cap_ink),
	.print_show       (print_show),

	.ctrl_out         (ctrl_out),
	.ctrl_in          (ctrl_in),
	.sound_ctrl       (),

	.aout_l           (AUDIO_L),
	.aout_r           (AUDIO_R),
	.snd_idle         (snd_idle),

	.sram_b_addr      (sram_b_addr),
	.sram_b_wren      (sram_b_wren),
	.sram_b_be        (2'b11),
	.sram_b_wdata     (sd_buff_dout),
	.sram_b_q         (sram_b_q),
	.sram_dirty       (sram_dirty),

	.ss_din           (ss_bus_din),
	.ss_addr          (ss_bus_addr),
	.ss_wren          (ss_bus_wren),
	.ss_rst           (ss_bus_rst),
	.ss_dout          (ss_bus_dout),
	.ss_flush         (ss_flush),
	.ss_idle          (ss_mem_idle),

	.ss_ram_addr      (ss_ram_addr),
	.ss_ram_type      (ss_ram_type),
	.ss_ram_rd        (ss_ram_rd),
	.ss_ram_wr        (ss_ram_wr),
	.ss_ram_wdata     (ss_ram_wdata),
	.ss_ram_rdata     (ss_ram_rdata),
	.ss_ram_ready     (ss_ram_ready),

	.ss_cpu_ready     (ss_cpu_ready),
	.ss_vdp_ready     (ss_vdp_ready),
	.ss_cache_rst     (ss_restore_hold)
);

////////////////////////////  BACKUP RAM  ////////////////////////////////////

// The cartridge SRAM is battery backed on real hardware; here it is a .sav
// file the framework mounts alongside the cartridge. `save_slot` owns the
// block protocol - which block, read or write, and when a write is due - and
// the only thing left here is the address arithmetic and the write strobe.
//
// One block is 512 bytes, so sd_lba counts blocks and sd_buff_addr counts the
// 256 words inside one. Together they are the whole 14-bit word address of a
// 32 KB SRAM.
wire [13:0] sram_b_addr = {sd_lba[5:0], sd_buff_addr};
wire        sram_b_wren = sd_buff_wr & sd_ack;
wire        sram_dirty;

// The header's SRAM range gives the size; below one block there is nothing
// worth writing, so a cartridge without SRAM never mounts a save.
wire [31:0] bk_blocks = {24'd0, cart_sram_mask[16:9]} + 32'd1;

// The OSD entries are momentary toggles, so each press flips the bit either
// way. save_slot wants an edge, and one cycle of level is the shortest thing
// it will see.
reg  bk_save_q, bk_load_q;
always @(posedge clk_sys) begin
	bk_save_q <= status[16];
	bk_load_q <= status[17];
end
wire bk_save_req = status[16] ^ bk_save_q;
wire bk_load_req = status[17] ^ bk_load_q;

// A new cartridge must not inherit the last one's save. The download starts
// before the framework mounts the new .sav, so clearing here cannot throw the
// new mount away.
reg  bk_dl_q;
wire cart_dl = ioctl_download && (ioctl_index[7:0] == 8'd1);
always @(posedge clk_sys) bk_dl_q <= cart_dl;
wire bk_invalidate = cart_dl & ~bk_dl_q;

save_slot #(.MAX_BLOCKS (64)) backup
(
	.clk_sys          (clk_sys),
	.invalidate_pulse (bk_invalidate),
	.mount_pulse      (img_mounted),
	.mount_readonly   (img_readonly),
	.mount_size       (img_size),
	.block_limit      (bk_blocks),
	.load_req         (bk_load_req),
	.save_req         (bk_save_req),
	.autosave_disable (status[15]),
	.osd_status       (OSD_STATUS),
	.dirty_pulse      (sram_dirty),
	// Port B belongs to the save slot alone, so a transfer never has to wait
	// for anything else to let go of it.
	.transfer_allowed (1'b1),
	.mounted_writable (bk_writable),
	.pending          (bk_pending),
	.busy             (bk_busy),
	.load_active      (),
	.sd_lba           (sd_lba),
	.sd_rd            (sd_rd),
	.sd_wr            (sd_wr),
	.sd_ack           (sd_ack)
);


////////////////////////////  SAVESTATES  /////////////////////////////////////

// Four 1 MB slots in DDR3 at 0x3E000000, which is 58,720,256 dwords from the
// framework's DDR base. The layout, the word map and the memory types are
// decision 0005 and rtl/savestate/ss_map.svh; the sizes below have to move
// with them, because STATESIZE is the only thing that stops a state file from
// an older build being loaded into this one.
localparam integer SS_INTERNALS = 160;
localparam integer SS_WORKRAM   = 524288;
localparam integer SS_VRAM      = 131072;
localparam integer SS_TILERAM   = 65536;
localparam integer SS_OAM       = 512;
localparam integer SS_PALETTE   = 512;
localparam integer SS_CARTSRAM  = 32768;
localparam integer SS_ONCHIP    = 3072;    // 1 KB SH7021 RAM, 2 KB voice RAM
localparam integer SS_STATESIZE = SS_INTERNALS * 2
	+ (SS_WORKRAM + SS_VRAM + SS_TILERAM + SS_OAM + SS_PALETTE
	   + SS_CARTSRAM + SS_ONCHIP) / 4;

wire [63:0] ss_bus_din, ss_bus_dout;
wire [9:0]  ss_bus_addr;
wire        ss_bus_wren, ss_bus_rst_pulse;
wire [24:0] ss_ram_addr;
wire [2:0]  ss_ram_type;
wire        ss_ram_rd, ss_ram_wr, ss_ram_ready;
wire [7:0]  ss_ram_wdata, ss_ram_rdata;
wire        ss_cpu_ready, ss_vdp_ready, ss_mem_idle, ss_flush;
wire        ss_sleep, ss_parked;
wire        ss_restore_begin, ss_load_done;
wire        ss_save, ss_load;
wire [25:0] ss_ddr_addr;
wire [63:0] ss_ddr_din, ss_ddr_dout;
wire [7:0]  ss_ddr_be;
wire        ss_ddr_rnw, ss_ddr_ena, ss_ddr_done;
wire        ss_req_save, ss_req_load;
integer     ss_req_addr;
wire        ss_busy;

// The machine is held in reset for the whole restore, so every block reloads
// from its saved words on the last reset cycle before the hold lets go.
reg  ss_restore_hold = 1'b0;
reg  [3:0] ss_after_q = 4'd0;
always @(posedge clk_sys) begin
	if (reset_cold) begin
		ss_restore_hold <= 1'b0;
		ss_after_q      <= 4'd0;
	end else begin
		if (ss_restore_begin)  ss_restore_hold <= 1'b1;
		else if (ss_load_done) ss_restore_hold <= 1'b0;

		// A few cycles after the hold lets go, put every word back to its
		// power-on default. Without that a later front-panel reset would
		// reload the state that was restored instead of resetting the machine.
		if (ss_restore_hold)         ss_after_q <= 4'd1;
		else if (ss_after_q != 4'd0 && ss_after_q != 4'd9)
			ss_after_q <= ss_after_q + 4'd1;
	end
end
wire ss_defaults_now = (ss_after_q == 4'd8);

// The reset synchroniser is two flops behind the hold, so an ordinary reset
// masked on the hold alone would reappear for two cycles while the machine is
// still in reset - long enough to clear every saved word back to its default
// just before the blocks read them.
wire ss_restore_window = (ss_after_q != 4'd0) && (ss_after_q < 4'd8);

// The bus reset carries the engine's own clear, the one above, and an ordinary
// reset - but not the restore hold, which is a machine reset and must leave
// the saved words alone.
assign ss_bus_rst = ss_bus_rst_pulse | ss_defaults_now
                    | (reset_sys & ~ss_restore_window);

// vsync is a video-clock signal and the slot manager is on clk_sys. Only the
// rewind wait counts it, but it still has to arrive as one bit at a time.
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
reg [1:0] vsync_sync_q = 2'b00;
always @(posedge clk_sys) vsync_sync_q <= {vsync_sync_q[0], vsync};
wire vsync_sys = vsync_sync_q[1];

wire ss_chord = joystick_0[11];   // the Savestates button, last in the J1 list

// The Savestates button is the one chord key, so it drives both joySS and
// joySaveState. That collapses the module's "joySaveState or joySS and
// joyStart" test to the button alone, and joyStart is tied low rather than
// borrowed from the game's Start - a chord needing three keys on a pad with
// one spare is worse to reach than Savestates plus a direction. Directions
// come from joystick_0 raw; the gating below is for the game's copy only.
savestate_ui ss_ui
(
	.clk           (clk_sys),
	.ps2_key       (ps2_key),
	.allow_ss      (cart_loaded & ~loading),
	.joySS         (ss_chord),
	.joyRight      (joystick_0[0]),
	.joyLeft       (joystick_0[1]),
	.joyDown       (joystick_0[2]),
	.joyUp         (joystick_0[3]),
	.joyStart      (1'b0),
	.joySaveState  (ss_chord),
	.status_slot   (status[19:18]),
	.OSD_saveload  (status[21:20]),
	.ss_save       (ss_save),
	.ss_load       (ss_load),
	.ss_info_req   (ss_info_req),
	.ss_info       (ss_info),
	.statusUpdate  (ss_status_update),
	.selected_slot (ss_slot)
);

// Rewind is not wired up in this phase: the ring would want a capture every
// few seconds and one capture costs about 190 ms of stopped machine.
statemanager #(
	.Softmap_SaveState_ADDR (58720256),
	.Softmap_Rewind_ADDR    (0),
	.SAVESTATE_SHIFT        (18)
) ss_manager
(
	.clk               (clk_sys),
	.reset             (reset_cold),
	.rewind_on         (1'b0),
	.rewind_active     (1'b0),
	.savestate_number  ({30'd0, ss_slot}),
	.save              (ss_save),
	.load              (ss_load),
	/* verilator lint_off PINCONNECTEMPTY */
	.sleep_rewind      (),
	/* verilator lint_on PINCONNECTEMPTY */
	.vsync             (vsync_sys),
	.request_savestate (ss_req_save),
	.request_loadstate (ss_req_load),
	/* verilator lint_off PINCONNECTEMPTY */
	.request_is_rewind (),
	/* verilator lint_on PINCONNECTEMPTY */
	.request_address   (ss_req_addr),
	.request_busy      (ss_busy)
);

savestates #(
	.STATESIZE_PARAM      (SS_STATESIZE),
	.INTERNALSCOUNT_PARAM (SS_INTERNALS),
	.SAVETYPESCOUNT_PARAM (7),
	.SAVETYPE0_SIZE       (SS_WORKRAM),
	.SAVETYPE1_SIZE       (SS_VRAM),
	.SAVETYPE2_SIZE       (SS_TILERAM),
	.SAVETYPE3_SIZE       (SS_OAM),
	.SAVETYPE4_SIZE       (SS_PALETTE),
	.SAVETYPE5_SIZE       (SS_CARTSRAM),
	.SAVETYPE6_SIZE       (SS_ONCHIP)
) ss_engine
(
	.clk                      (clk_sys),
	.reset_in                 (reset_cold),
	/* verilator lint_off PINCONNECTEMPTY */
	.reset_ss                 (),
	.reset_delay              (),
	/* verilator lint_on PINCONNECTEMPTY */
	.restore_begin            (ss_restore_begin),
	.load_done                (ss_load_done),
	// No cartridge overlay to put back before the machine starts, so the
	// reset-held preparation phase is over as soon as it begins.
	.restore_prepare_ready_i  (1'b1),
	.restore_prepare_failed_i (1'b0),

	.increaseSSHeaderCount    (1'b1),
	.save                     (ss_req_save),
	.load                     (ss_req_load),
	.state_size_i             (SS_STATESIZE[31:0]),
	.savetype3_size_i         (SS_OAM[24:0]),
	.is_rewind_i              (1'b0),
	.savestate_address        (ss_req_addr),
	.savestate_busy           (ss_busy),
	.paused                   (ss_parked),

	.BUS_Din                  (ss_bus_din),
	.BUS_Adr                  (ss_bus_addr),
	.BUS_wren                 (ss_bus_wren),
	.BUS_rst                  (ss_bus_rst_pulse),
	.BUS_Dout                 (ss_bus_dout),

	/* verilator lint_off PINCONNECTEMPTY */
	.loading_savestate        (),
	.saving_savestate         (),
	/* verilator lint_on PINCONNECTEMPTY */
	.sleep_savestate          (ss_sleep),

	.Save_RAMAddr             (ss_ram_addr),
	.Save_RAMRdEn             (ss_ram_rd),
	.Save_RAMWrEn             (ss_ram_wr),
	.Save_RAMWriteData        (ss_ram_wdata),
	.Save_RAMReadData         (ss_ram_rdata),
	.Save_RAMReady            (ss_ram_ready),
	.Save_RAMType             (ss_ram_type),

	.bus_out_Din              (ss_ddr_din),
	.bus_out_Dout             (ss_ddr_dout),
	.bus_out_Adr              (ss_ddr_addr),
	.bus_out_rnw              (ss_ddr_rnw),
	.bus_out_ena              (ss_ddr_ena),
	.bus_out_be               (ss_ddr_be),
	.bus_out_done             (ss_ddr_done)
);

ss_park park
(
	.clk_sys     (clk_sys),
	.reset_sys   (reset_cold),
	.req         (ss_sleep),
	.cpu_ready   (ss_cpu_ready),
	.snd_ready   (snd_idle),
	.mem_idle    (ss_mem_idle),
	.pause_sys   (ss_pause_sys),
	.flush       (ss_flush),
	.parked      (ss_parked),
	.clk_video   (clk_video),
	.reset_video (reset_cold_video),
	.vdp_ready   (ss_vdp_ready),
	.pause_video (ss_pause_video)
);

loopy_ddram ddr
(
	.clk              (clk_video),
	.reset            (reset_cold_video),
	.ss_clk           (clk_sys),
	.ss_reset         (reset_cold),
	.DDRAM_CLK        (DDRAM_CLK),
	.DDRAM_BUSY       (DDRAM_BUSY),
	.DDRAM_BURSTCNT   (DDRAM_BURSTCNT),
	.DDRAM_ADDR       (DDRAM_ADDR),
	.DDRAM_DOUT       (DDRAM_DOUT),
	.DDRAM_DOUT_READY (DDRAM_DOUT_READY),
	.DDRAM_RD         (DDRAM_RD),
	.DDRAM_DIN        (DDRAM_DIN),
	.DDRAM_BE         (DDRAM_BE),
	.DDRAM_WE         (DDRAM_WE),
	.addr_i           (ss_ddr_addr),
	.din_i            (ss_ddr_din),
	.dout_o           (ss_ddr_dout),
	.be_i             (ss_ddr_be),
	.rnw_i            (ss_ddr_rnw),
	.ena_i            (ss_ddr_ena),
	.done_o           (ss_ddr_done),
	.pr_req           (print_cmd_req),
	.pr_ready         (print_cmd_ready),
	.pr_addr          (print_cmd_addr),
	.pr_din           (print_cmd_din),
	.pr_be            (print_cmd_be),
	.pr_rnw           (print_cmd_rnw),
	.pr_burstcnt      (print_cmd_burstcnt),
	.pr_dout          (print_cmd_dout),
	.pr_valid         (print_cmd_valid),
	.pr_done          (print_cmd_done)
);

////////////////////////////  CONTROLLERS  ////////////////////////////////////

// MiSTer pad bits are right, left, down, up, then the buttons in the order the
// "J1" line names them. The Loopy pad wants its own labels, so the two orders
// are spelled out here rather than hidden in the module.
function automatic [10:0] to_pad (input [31:0] j);
	begin
		to_pad = {j[0],   // right
		          j[1],   // left
		          j[2],   // down
		          j[3],   // up
		          j[5],   // B
		          j[6],   // C
		          j[7],   // D
		          j[4],   // A
		          j[9],   // R
		          j[8],   // L
		          j[10]}; // start
	end
endfunction

wire [5:0] ctrl_out;
wire [7:0] ctrl_in;

// The chord picks its slot with the d-pad, so keep the d-pad away from the game
// while the button is held. Only pad 0 carries the chord.
wire [31:0] joystick_0_pad = {joystick_0[31:4], joystick_0[3:0] & {4{~ss_chord}}};

// A pad that is not selected reads as unplugged, which is how software counts
// them: there is no way to ask the HPS how many are really there.
reg [3:0] pads_present;
always @* begin
	case (status[14:13])
	2'd0:    pads_present = 4'b0001;
	2'd1:    pads_present = 4'b0011;
	2'd2:    pads_present = 4'b0111;
	default: pads_present = 4'b1111;
	endcase
end

// Everything from the HPS settles in clk_sys and is read in clk_video. Each of
// these bits stands on its own - one button, one device select, one pad count -
// so two flops apiece is the whole of it; there is no word here whose bits have
// to arrive together.
wire [50:0] pad_async = {ps2_mouse[1], ps2_mouse[0], status[12], pads_present,
                         to_pad(joystick_3), to_pad(joystick_2),
                         to_pad(joystick_1), to_pad(joystick_0_pad)};

(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
reg [50:0] pad_sync_q1 = 51'd0, pad_sync_q2 = 51'd0;
always @(posedge clk_video) begin
	pad_sync_q1 <= pad_async;
	pad_sync_q2 <= pad_sync_q1;
end

wire [10:0] pad0_v         = pad_sync_q2[10:0];
wire [10:0] pad1_v         = pad_sync_q2[21:11];
wire [10:0] pad2_v         = pad_sync_q2[32:22];
wire [10:0] pad3_v         = pad_sync_q2[43:33];
wire [3:0]  pads_present_v = pad_sync_q2[47:44];
wire        use_mouse_v    = pad_sync_q2[48];
wire        mouse_l_v      = pad_sync_q2[49];
wire        mouse_r_v      = pad_sync_q2[50];

// The report bytes are written before ps2_mouse[24] toggles, so they are
// already still by the time the toggle has crossed and can be read straight.
(* altera_attribute = "-name SYNCHRONIZER_IDENTIFICATION FORCED_IF_ASYNCHRONOUS" *)
reg [2:0] ps2_stb_q = 3'b000;
always @(posedge clk_video) ps2_stb_q <= {ps2_stb_q[1:0], ps2_mouse[24]};
wire ps2_stb = ps2_stb_q[2] ^ ps2_stb_q[1];

loopy_input input_i
(
	.clk         (clk_video),
	.reset       (reset_video),
	.ce_pix      (ce_pix),
	.use_mouse   (use_mouse_v),
	.pad0        (pad0_v),
	.pad1        (pad1_v),
	.pad2        (pad2_v),
	.pad3        (pad3_v),
	.pad_present (pads_present_v),
	.mouse_stb   (ps2_stb),
	.mouse_dx    ({ps2_mouse[4], ps2_mouse[15:8]}),
	.mouse_dy    ({ps2_mouse[5], ps2_mouse[23:16]}),
	.mouse_l     (mouse_l_v),
	.mouse_r     (mouse_r_v),
	.ctrl_out    (ctrl_out),
	.ctrl_in     (ctrl_in)
);

// p1 and p2 are read-only in this core, so their write inputs are tied off
// here rather than carried through the bridge as constants. p0 and p1 move
// whole eight-byte cache lines; p2 reads wave ROM bytes.
sdram #(
	.CLK_FREQ_HZ (128_863_636),
	// Measured on this board 2026-09-04. At 85.909 MHz this was 2; at 128.864
	// the period is 7.76 ns instead of 11.64 and the same round trip out
	// through the forwarded clock and back no longer fits inside one of them,
	// so the first beat is sampled before the SDRAM has driven it. The debug
	// page read the cartridge line back as FFFF then beats 0,1,2 - every beat
	// one slot late, the first one an idle bus. Keep sdram.sdc's DQ multicycle
	// in step with this.
	.DQ_CAPTURE_PIPELINE (3),
	.PORT0_SIZE  (3),   // 64-bit: work DRAM lines and the loader
	.PORT1_SIZE  (3),   // 64-bit: cartridge ROM lines
	.PORT2_SIZE  (3)    // 64-bit: wave ROM lines
) sdram_i
(
	.clk        (clk_ram),
	.reset      (reset_ram),
	.refresh    (1'b0),
	.dq_pipe_sel (3'd0),

	.p0_req     (p0_req),
	.p0_we      (p0_we),
	.p0_addr    (p0_addr),
	.p0_din     (p0_din),
	.p0_byte_en (p0_byte_en),
	.p0_dout    (p0_dout),
	.p0_busy    (p0_busy),
	.p0_ready   (p0_ready),
	.p1_req     (p1_req),
	.p1_we      (1'b0),
	.p1_addr    (p1_addr),
	.p1_din     (64'd0),
	.p1_byte_en (8'd0),
	.p1_dout    (p1_dout),
	.p1_busy    (p1_busy),
	.p1_ready   (p1_ready),
	.p2_req     (p2_req),
	.p2_we      (1'b0),
	.p2_addr    (p2_addr),
	.p2_din     (64'd0),
	.p2_byte_en (8'd0),
	.p2_dout    (p2_dout),
	.p2_busy    (p2_busy),
	.p2_ready   (p2_ready),

	.SDRAM_CLK  (SDRAM_CLK),
	.SDRAM_CKE  (SDRAM_CKE),
	.SDRAM_A    (SDRAM_A),
	.SDRAM_BA   (SDRAM_BA),
	.SDRAM_DQ   (SDRAM_DQ),
	.SDRAM_DQML (SDRAM_DQML),
	.SDRAM_DQMH (SDRAM_DQMH),
	.SDRAM_nCS  (SDRAM_nCS),
	.SDRAM_nCAS (SDRAM_nCAS),
	.SDRAM_nRAS (SDRAM_nRAS),
	.SDRAM_nWE  (SDRAM_nWE)
);

////////////////////////////  VIDEO  //////////////////////////////////////////

wire [14:0] color_out;

loopy_print_store print_store
(
	.clk            (clk_video),
	.reset          (reset_cold_video),
	.clear          (print_clear),
	.cap_wr         (print_cap_wr),
	.cap_y          (print_cap_y),
	.cap_x          (print_cap_x),
	.cap_pass       (print_cap_pass),
	.cap_ink        (print_cap_ink),
	.active         (print_show && status[9:8] != 2'd0),
	.frame_start    (print_frame_start),
	.display_row    (print_display_row),
	.display_active (print_display_active),
	.rd_addr        (print_rd_addr),
	.rd_data        (print_rd_data),
	.rd_valid       (print_rd_valid),
	.fault          (print_fault),
	.cmd_req        (print_cmd_req),
	.cmd_ready      (print_cmd_ready),
	.cmd_addr       (print_cmd_addr),
	.cmd_din        (print_cmd_din),
	.cmd_be         (print_cmd_be),
	.cmd_rnw        (print_cmd_rnw),
	.cmd_burstcnt   (print_cmd_burstcnt),
	.cmd_dout       (print_cmd_dout),
	.cmd_valid      (print_cmd_valid),
	.cmd_done       (print_cmd_done)
);

loopy_print_overlay print_overlay
(
	.clk      (clk_video),
	.reset    (reset_video),
	.ce_pix   (ce_pix),

	.hvisible (hvisible),
	.vvisible (vvisible),
	.render   (render),

	.mode     (status[9:8]),
	.ink_order (status[11:10]),
	.show     (print_show && !print_fault),

	.rd_addr  (print_rd_addr),
	.rd_data  (print_rd_data),
	.rd_valid (print_rd_valid),
	.display_row (print_display_row),
	.display_active (print_display_active),
	.frame_start (print_frame_start),

	.color_i  (rgb),
	.color_o  (color_out)
);

// RGB555 to RGB888 by repeating the top bits of each channel, which keeps 0
// black and 31 fully white. A plain left shift would cap white at 248 and tint
// the picture.
wire [4:0] r5 = color_out[14:10];
wire [4:0] g5 = color_out[9:5];
wire [4:0] b5 = color_out[4:0];

// Border shown is what the machine really puts out. Cropped leaves just the
// 256-pixel render window, which suits some displays and scaler presets.
wire crop_border = ~status[5];
wire hblank = crop_border ? ~hrender : ~hvisible;
wire vblank = crop_border ? ~vrender : ~vvisible;

// Scandoubler Fx picks the scanline weight and, when it is not "None", turns
// the doubler on. The framework can also force the doubler for analogue VGA.
wire [2:0] sfx = status[24:22];
wire [2:0] sl  = |sfx ? sfx - 3'd1 : 3'd0;
assign VGA_SL = sl[1:0];

assign CLK_VIDEO = clk_video;

// LINE_LENGTH is the scandoubler's line buffer: the visible line is
// (52 + 1029 + 52) / 4 = 284 pixels.
wire vga_de_mixed;

video_mixer #(.LINE_LENGTH(320), .HALF_DEPTH(0), .GAMMA(1)) video_mixer
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.ce_pix      (ce_pix),

	.scandoubler (|sfx | forced_scandoubler),
	.hq2x        (sfx == 3'd1),

	.gamma_bus   (gamma_bus),

	.R           ({r5, r5[4:2]}),
	.G           ({g5, g5[4:2]}),
	.B           ({b5, b5[4:2]}),

	.HSync       (hsync),
	.VSync       (vsync),
	.HBlank      (hblank),
	.VBlank      (vblank),

	.HDMI_FREEZE (HDMI_FREEZE),
	.freeze_sync (),

	.VGA_R       (VGA_R),
	.VGA_G       (VGA_G),
	.VGA_B       (VGA_B),
	.VGA_VS      (VGA_VS),
	.VGA_HS      (VGA_HS),
	.VGA_DE      (vga_de_mixed)
);

// 216 of the console's lines fit 1080p exactly at 5x. Offer it only when the
// output really is 1080p and nothing else is already resizing the picture.
reg en216p;
always @(posedge CLK_VIDEO) en216p <= (HDMI_WIDTH == 12'd1920) && (HDMI_HEIGHT == 12'd1080)
                                   && !forced_scandoubler && (status[27:25] == 3'd0);
wire vcrop = status[4] & en216p;

// video_freak owns VIDEO_ARX/ARY because the integer scaling modes set their
// [12] flag.
video_freak video_freak
(
	.CLK_VIDEO   (CLK_VIDEO),
	.CE_PIXEL    (CE_PIXEL),
	.VGA_VS      (VGA_VS),
	.HDMI_WIDTH  (HDMI_WIDTH),
	.HDMI_HEIGHT (HDMI_HEIGHT),
	.VGA_DE      (VGA_DE),
	.VIDEO_ARX   (VIDEO_ARX),
	.VIDEO_ARY   (VIDEO_ARY),

	.VGA_DE_IN   (vga_de_mixed),
	.ARX         ((ar == 2'd0) ? 12'd4 : {10'd0, ar - 2'd1}),
	.ARY         ((ar == 2'd0) ? 12'd3 : 12'd0),
	.CROP_SIZE   (vcrop ? 12'd216 : 12'd0),
	.CROP_OFF    (5'd0),
	.SCALE       (status[27:25])
);

assign LED_USER = loading;

// synthesis translate_off
/* verilator lint_off UNUSEDSIGNAL */
wire unused = &{1'b0, buttons[0], status[127:123], status[120:12],
                status[6], status[4:1], ioctl_index[15:8]};
/* verilator lint_on UNUSEDSIGNAL */
// synthesis translate_on

endmodule
