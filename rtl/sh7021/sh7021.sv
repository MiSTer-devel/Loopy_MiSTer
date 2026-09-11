// Copyright (c) 2026 Jamie Blanks
//
// SH7021 pin-level top. Ports carry the chip's own pin names, with
// bidirectional pins split into input, output and output enable. CS4-CS7,
// RAS, WAIT, RD, WRH, WRL, BACK, BREQ and AH live on port A pins and appear
// only when the pin function controller selects them; CS1/CASH and CS3/CASL
// share pins chosen by CASCR. State advances on ce_i, one pulse per SH7021
// state; ceh_i marks the middle of that state for the DRAM strobe edges.

module sh7021 (
	input  wire        clk_i,
	input  wire        ce_i,
	input  wire        ceh_i,          // second half of the state ce_i started
	input  wire        res_n_i,
	input  wire [2:0]  md_i,
	input  wire        nmi_i,

	output wire [21:0] a_o,
	input  wire [15:0] ad_i,
	output wire [15:0] ad_o,
	output wire        ad_oe_o,

	output wire        cs0_n_o,        // pin 46, dedicated
	output wire        cs2_n_o,        // pin 48, dedicated
	output wire        cs1_cash_n_o,   // pin 47, CASCR picks which
	output wire        cs3_casl_n_o,   // pin 49
	output wire        wdtovf_n_o,     // pin 75, dedicated

	input  wire [15:0] pa_i,
	output wire [15:0] pa_o,
	output wire [15:0] pa_oe_o,
	input  wire [15:0] pb_i,
	output wire [15:0] pb_o,
	output wire [15:0] pb_oe_o,

	// BIOS load path into the on-chip ROM
	input  wire [12:0] rom_wr_addr_i,
	input  wire        rom_wr_en_i,
	input  wire [31:0] rom_wr_data_i,

	// Savestate scalar bus. Every block loads its restored state in the same
	// reset branch that serves power-on, so res_n_i must fall after the
	// words are written.
	input  wire [63:0] ss_din,
	input  wire [9:0]  ss_addr,
	input  wire        ss_wren,
	input  wire        ss_rst,
	output wire [63:0] ss_dout,

	// The on-chip RAM's second port, for the savestate bulk walk.
	input  wire [9:0]  ss_ram_addr,
	input  wire        ss_ram_we,
	input  wire [7:0]  ss_ram_din,
	output wire [7:0]  ss_ram_dout,

	// The chip is at a place a savestate may stop it: the core is between
	// instructions with nothing on the bus, and the DMAC between transfer
	// units.
	output wire        ss_ready
);
	`include "sh1_defs.svh"
	`include "ss_map.svh"

	// A watchdog overflow with RSTCSR.RSTE set drives an internal reset for
	// 512 states, of the type RSTS picks. It reaches everything but the
	// watchdog itself, whose RSTCSR survives it.
	wire wdt_reset, wdt_reset_manual;
	wire rst     = ~res_n_i | wdt_reset;
	wire ext_rst = ~res_n_i;

	// One savestate read-back per block; each drives zero unless the address
	// on the bus is one of its own words.
	wire [63:0] ssq_cpu, ssq_ibus, ssq_bsc, ssq_intc, ssq_dmac, ssq_itu;
	wire [63:0] ssq_wdt, ssq_tpc, ssq_ubc, ssq_sci0, ssq_sci1, ssq_pfc;

	assign ss_dout = ssq_cpu | ssq_ibus | ssq_bsc | ssq_intc | ssq_dmac
	               | ssq_itu | ssq_wdt | ssq_tpc | ssq_ubc | ssq_sci0
	               | ssq_sci1 | ssq_pfc;

	// ------------------------------------------------------ internal bus
	wire        cpu_req, cpu_we, cpu_ifetch, cpu_ack;
	wire [31:0] cpu_addr, cpu_wdata, cpu_rdata;
	wire [1:0]  cpu_sz;

	wire        dm_breq, dm_req, dm_we, dm_ack, dm_gnt;
	wire [31:0] dm_addr, dm_wdata, dm_rdata;
	wire [1:0]  dm_sz;

	wire        bus_req, bus_we, bus_ifetch, bus_ack, bus_hold, bus_lock;
	wire        cpu_ss_ready;
	// The DMAC holds the bus for a whole transfer unit, so between units it
	// is neither asking for nor using it.
	assign ss_ready = cpu_ss_ready && !dm_breq && !dm_req;
	wire [31:0] bus_addr, bus_wdata, bus_rdata;
	wire [1:0]  bus_sz;

	sh7021_ibus u_ibus (
		.clk_i        (clk_i),
		.ce_i         (ce_i),
		.rst_i        (rst),
		.hold_i       (bus_hold),
		.lock_i       (bus_lock),
		.cpu_req_i    (cpu_req),
		.cpu_addr_i   (cpu_addr),
		.cpu_we_i     (cpu_we),
		.cpu_sz_i     (cpu_sz),
		.cpu_wdata_i  (cpu_wdata),
		.cpu_ifetch_i (cpu_ifetch),
		.cpu_rdata_o  (cpu_rdata),
		.cpu_ack_o    (cpu_ack),
		.dm_breq_i    (dm_breq),
		.dm_req_i     (dm_req),
		.dm_addr_i    (dm_addr),
		.dm_we_i      (dm_we),
		.dm_sz_i      (dm_sz),
		.dm_wdata_i   (dm_wdata),
		.dm_rdata_o   (dm_rdata),
		.dm_ack_o     (dm_ack),
		.dm_gnt_o     (dm_gnt),
		.req_o        (bus_req),
		.addr_o       (bus_addr),
		.we_o         (bus_we),
		.sz_o         (bus_sz),
		.wdata_o      (bus_wdata),
		.ifetch_o     (bus_ifetch),
		.rdata_i      (bus_rdata),
		.ack_i        (bus_ack),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ssq_ibus)
	);

	// ----------------------------------------------------------- the core
	wire        int_req, int_ack;
	wire [7:0]  int_vec;
	wire [3:0]  int_level;
	wire        sleep;
	wire        dma_addr_err;
	wire [3:0]  sr_mask;

	sh1_core u_cpu (
		.clk_i          (clk_i),
		.ce_i           (ce_i),
		.rst_i          (rst),
		.manual_rst_i   (wdt_reset ? wdt_reset_manual : ~nmi_i),
		.bus_req_o      (cpu_req),
		.bus_addr_o     (cpu_addr),
		.bus_we_o       (cpu_we),
		.bus_sz_o       (cpu_sz),
		.bus_wdata_o    (cpu_wdata),
		.bus_ifetch_o   (cpu_ifetch),
		.bus_rdata_i    (cpu_rdata),
		.bus_ack_i      (cpu_ack),
		.int_req_i      (int_req),
		.int_vec_i      (int_vec),
		.int_level_i    (int_level),
		.int_ack_o      (int_ack),
		.dma_addr_err_i (dma_addr_err),
		.sleep_o        (sleep),
		.bus_lock_o     (bus_lock),
		.sr_mask_o      (sr_mask),
		.ss_ready_o     (cpu_ss_ready),
		.ss_din         (ss_din),
		.ss_addr        (ss_addr),
		.ss_wren        (ss_wren),
		.ss_rst         (ss_rst),
		.ss_dout        (ssq_cpu)
	);

	// ------------------------------------------------------------ the BSC
	wire        rom_sel, ram_sel, ram_we;
	wire [31:0] mem_addr, ram_wd, rom_q, ram_q;
	wire [3:0]  ram_be;
	/* verilator lint_off UNUSEDSIGNAL */
	wire [31:0] ram_q_b;

	// The savestate walks the on-chip RAM a byte at a time. The RAM is
	// big-endian, so byte 0 of a longword is its top byte and the byte
	// enables run the other way round from the address.
	reg  [1:0] ss_ram_sel_q;
	always @(posedge clk_i) ss_ram_sel_q <= ss_ram_addr[1:0];
	wire [3:0] ss_ram_be = 4'b1000 >> ss_ram_addr[1:0];
	assign ss_ram_dout =
		(ss_ram_sel_q == 2'd0) ? ram_q_b[31:24] :
		(ss_ram_sel_q == 2'd1) ? ram_q_b[23:16] :
		(ss_ram_sel_q == 2'd2) ? ram_q_b[15:8]  : ram_q_b[7:0];
	/* verilator lint_on UNUSEDSIGNAL */
	wire        psel, pwr;
	wire [8:0]  paddr;
	wire [1:0]  pbe;
	wire [15:0] pwdata, prdata;
	wire [21:0] bsc_a;
	wire [7:0]  cs_n;
	wire        rd_n, wrh_n, wrl_n, ras_n, cash_n, casl_n, ah;
	wire        cmi;
	wire        wait_n;

	sh7021_bsc u_bsc (
		.clk_i      (clk_i),
		.ce_i       (ce_i),
		.ceh_i      (ceh_i),
		.rst_i      (rst),
		.md_i       (md_i),
		.req_i      (bus_req),
		.addr_i     (bus_addr),
		.we_i       (bus_we),
		.sz_i       (bus_sz),
		.wdata_i    (bus_wdata),
		.rdata_o    (bus_rdata),
		.ack_o      (bus_ack),
		.rom_sel_o  (rom_sel),
		.ram_sel_o  (ram_sel),
		.mem_addr_o (mem_addr),
		.ram_we_o   (ram_we),
		.ram_be_o   (ram_be),
		.ram_wd_o   (ram_wd),
		.rom_q_i    (rom_q),
		.ram_q_i    (ram_q),
		.psel_o     (psel),
		.paddr_o    (paddr),
		.pwr_o      (pwr),
		.pbe_o      (pbe),
		.pwdata_o   (pwdata),
		.prdata_i   (prdata),
		.a_o        (bsc_a),
		.d_o        (ad_o),
		.d_oe_o     (ad_oe_o),
		.d_i        (ad_i),
		.cs_n_o     (cs_n),
		.rd_n_o     (rd_n),
		.wrh_n_o    (wrh_n),
		.wrl_n_o    (wrl_n),
		.ras_n_o    (ras_n),
		.cash_n_o   (cash_n),
		.casl_n_o   (casl_n),
		.ah_o       (ah),
		.wait_n_i   (wait_n),
		.cmi_o      (cmi),
		.hold_o     (bus_hold),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ssq_bsc)
	);
	assign a_o = bsc_a;

	sh7021_rom u_rom (
		.clk_i     (clk_i),
		.addr_i    (mem_addr[14:2]),
		.q_o       (rom_q),
		.wr_addr_i (rom_wr_addr_i),
		.wr_en_i   (rom_wr_en_i),
		.wr_data_i (rom_wr_data_i)
	);

	sh7021_ram u_ram (
		.clk_i     (clk_i),
		.addr_i    (mem_addr[9:2]),
		.we_i      (ram_we),
		.be_i      (ram_be),
		.wdata_i   (ram_wd),
		.q_o       (ram_q),
		.addr_b_i  (ss_ram_addr[9:2]),
		.we_b_i    (ss_ram_we),
		.be_b_i    (ss_ram_be),
		.wdata_b_i ({4{ss_ram_din}}),
		.q_b_o     (ram_q_b)
	);

	// ---------------------------------------------------- register bus fan
	wire [15:0] pr_intc, pr_dmac, pr_itu, pr_wdt, pr_sci0, pr_sci1;
	wire [15:0] pr_pfc, pr_tpc, pr_ubc;
	wire        h_intc, h_dmac, h_itu, h_wdt, h_sci0, h_sci1;
	wire        h_pfc, h_tpc, h_ubc;

	assign prdata = (h_intc ? pr_intc : 16'd0) | (h_dmac ? pr_dmac : 16'd0)
	              | (h_itu  ? pr_itu  : 16'd0) | (h_wdt  ? pr_wdt  : 16'd0)
	              | (h_sci0 ? pr_sci0 : 16'd0) | (h_sci1 ? pr_sci1 : 16'd0)
	              | (h_pfc  ? pr_pfc  : 16'd0) | (h_tpc  ? pr_tpc  : 16'd0)
	              | (h_ubc  ? pr_ubc  : 16'd0);

	// ----------------------------------------------------------- the INTC
	wire [7:0]  irq_n;
	wire [30:0] onchip;
	wire        irqout;
	wire [4:0]  imia, imib, ovi;
	wire [3:0]  dei;
	wire        iti, eri0, rxi0, txi0, tei0, eri1, rxi1, txi1, tei1;
	wire        ubc_brk;

	assign onchip = {cmi, iti, 1'b0,                       // CMI, ITI, PEI
	                 tei1, txi1, rxi1, eri1,
	                 tei0, txi0, rxi0, eri0,
	                 ovi[4], imib[4], imia[4],
	                 ovi[3], imib[3], imia[3],
	                 ovi[2], imib[2], imia[2],
	                 ovi[1], imib[1], imia[1],
	                 ovi[0], imib[0], imia[0],
	                 dei[3], dei[2], dei[1], dei[0],
	                 ubc_brk};

	sh7021_intc u_intc (
		.clk_i     (clk_i),
		.ce_i      (ce_i),
		.rst_i     (rst),
		.psel_i    (psel),
		.paddr_i   (paddr),
		.pwr_i     (pwr),
		.pbe_i     (pbe),
		.pwdata_i  (pwdata),
		.prdata_o  (pr_intc),
		.nmi_i     (nmi_i),
		.irq_i     (irq_n),
		.onchip_i  (onchip),
		.sr_mask_i (sr_mask),
		.req_o     (int_req),
		.vec_o     (int_vec),
		.level_o   (int_level),
		.ack_i     (int_ack),
		.irqout_o  (irqout),
		.ss_din    (ss_din),
		.ss_addr   (ss_addr),
		.ss_wren   (ss_wren),
		.ss_rst    (ss_rst),
		.ss_dout   (ssq_intc)
	);
	assign h_intc = psel && (paddr[8:1] >= 8'hC2) && (paddr[8:1] <= 8'hC7);

	// ----------------------------------------------------------- the DMAC
	wire [1:0] dreq_n, dack;
	wire [3:0] imia_ack;
	wire [1:0] rxi_ack, txi_ack;

	sh7021_dmac u_dmac (
		.clk_i      (clk_i),
		.ce_i       (ce_i),
		.rst_i      (rst),
		.psel_i     (psel),
		.paddr_i    (paddr),
		.pwr_i      (pwr),
		.pbe_i      (pbe),
		.pwdata_i   (pwdata),
		.prdata_o   (pr_dmac),
		.phit_o     (h_dmac),
		.breq_o     (dm_breq),
		.bgnt_i     (dm_gnt),
		.req_o      (dm_req),
		.addr_o     (dm_addr),
		.we_o       (dm_we),
		.sz_o       (dm_sz),
		.wdata_o    (dm_wdata),
		.rdata_i    (dm_rdata),
		.ack_i      (dm_ack),
		.dreq_i     (dreq_n),
		.dack_o     (dack),
		.nmi_i      (int_ack && (int_vec == 8'd11)),
		.addr_err_i (1'b0),
		.rxi_i      ({rxi1, rxi0}),
		.txi_i      ({txi1, txi0}),
		.imia_i     (imia[3:0]),
		.imia_ack_o (imia_ack),
		.rxi_ack_o  (rxi_ack),
		.txi_ack_o  (txi_ack),
		.dei_o      (dei),
		.addr_err_o (dma_addr_err),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ssq_dmac)
	);

	// ------------------------------------------------------------ the ITU
	wire [3:0] tclk;
	wire [4:0] tioca_in, tiocb_in, tioca_out, tiocb_out, tioca_oe, tiocb_oe;
	wire [3:0] itu_cma;

	sh7021_itu u_itu (
		.clk_i      (clk_i),
		.ce_i       (ce_i),
				.rst_i      (rst),
		.psel_i     (psel),
		.paddr_i    (paddr),
		.pwr_i      (pwr),
		.pbe_i      (pbe),
		.pwdata_i   (pwdata),
		.prdata_o   (pr_itu),
		.phit_o     (h_itu),
		.tclk_i     (tclk),
		.tioca_i    (tioca_in),
		.tiocb_i    (tiocb_in),
		.tioca_o    (tioca_out),
		.tiocb_o    (tiocb_out),
		.tioca_oe_o (tioca_oe),
		.tiocb_oe_o (tiocb_oe),
		.imia_o     (imia),
		.imib_o     (imib),
		.ovi_o      (ovi),
		.cma_o      (itu_cma),
		.dmac_ack_i (imia_ack),
		.ss_din     (ss_din),
		.ss_addr    (ss_addr),
		.ss_wren    (ss_wren),
		.ss_rst     (ss_rst),
		.ss_dout    (ssq_itu)
	);

	// ------------------------------------------------- WDT, TPC, UBC, SCI
	sh7021_wdt u_wdt (
		.clk_i              (clk_i),
		.ce_i               (ce_i),
		.rst_i              (ext_rst),
		.psel_i             (psel),
		.paddr_i            (paddr),
		.pwr_i              (pwr),
		.pbe_i              (pbe),
		.pwdata_i           (pwdata),
		.prdata_o           (pr_wdt),
		.phit_o             (h_wdt),
		.iti_o              (iti),
		.wdtovf_n_o         (wdtovf_n_o),
		.int_reset_o        (wdt_reset),
		.int_reset_manual_o (wdt_reset_manual),
		.ss_din             (ss_din),
		.ss_addr            (ss_addr),
		.ss_wren            (ss_wren),
		.ss_rst             (ss_rst),
		.ss_dout            (ssq_wdt)
	);

	wire [15:0] tpc_dr, tpc_en;
	sh7021_tpc u_tpc (
		.clk_i     (clk_i),
		.ce_i      (ce_i),
		.rst_i     (rst),
		.psel_i    (psel),
		.paddr_i   (paddr),
		.pwr_i     (pwr),
		.pbe_i     (pbe),
		.pwdata_i  (pwdata),
		.prdata_o  (pr_tpc),
		.phit_o    (h_tpc),
		.itu_cma_i (itu_cma),
		.tpc_dr_o  (tpc_dr),
		.tpc_en_o  (tpc_en),
		.ss_din    (ss_din),
		.ss_addr   (ss_addr),
		.ss_wren   (ss_wren),
		.ss_rst    (ss_rst),
		.ss_dout   (ssq_tpc)
	);

	sh7021_ubc u_ubc (
		.clk_i        (clk_i),
		.ce_i         (ce_i),
		.rst_i        (rst),
		.psel_i       (psel),
		.paddr_i      (paddr),
		.pwr_i        (pwr),
		.pbe_i        (pbe),
		.pwdata_i     (pwdata),
		.prdata_o     (pr_ubc),
		.phit_o       (h_ubc),
		.cyc_i        (bus_req && bus_ack),
		.cyc_addr_i   (bus_addr),
		.cyc_we_i     (bus_we),
		.cyc_ifetch_i (bus_ifetch),
		.cyc_dma_i    (dm_gnt),
		.cyc_sz_i     (bus_sz),
		.brk_o        (ubc_brk),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ssq_ubc)
	);

	wire rxd0, txd0, txd0_oe, sck0, sck0_oe;
	wire rxd1, txd1, txd1_oe, sck1, sck1_oe;

	sh7021_sci #(.BASE (8'h60), .SS_BASE (SSW_SCI0_BASE)) u_sci0 (
		.clk_i        (clk_i),
		.ce_i         (ce_i),
		.rst_i        (rst),
		.psel_i       (psel),
		.paddr_i      (paddr),
		.pwr_i        (pwr),
		.pbe_i        (pbe),
		.pwdata_i     (pwdata),
		.prdata_o     (pr_sci0),
		.phit_o       (h_sci0),
		.dma_tx_ack_i (txi_ack[0]),
		.dma_rx_ack_i (rxi_ack[0]),
		.rxd_i        (rxd0),
		.txd_o        (txd0),
		.txd_oe_o     (txd0_oe),
		.sck_o        (sck0),
		.sck_oe_o     (sck0_oe),
		.eri_o        (eri0),
		.rxi_o        (rxi0),
		.txi_o        (txi0),
		.tei_o        (tei0),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ssq_sci0)
	);

	sh7021_sci #(.BASE (8'h64), .SS_BASE (SSW_SCI1_BASE)) u_sci1 (
		.clk_i        (clk_i),
		.ce_i         (ce_i),
		.rst_i        (rst),
		.psel_i       (psel),
		.paddr_i      (paddr),
		.pwr_i        (pwr),
		.pbe_i        (pbe),
		.pwdata_i     (pwdata),
		.prdata_o     (pr_sci1),
		.phit_o       (h_sci1),
		.dma_tx_ack_i (txi_ack[1]),
		.dma_rx_ack_i (rxi_ack[1]),
		.rxd_i        (rxd1),
		.txd_o        (txd1),
		.txd_oe_o     (txd1_oe),
		.sck_o        (sck1),
		.sck_oe_o     (sck1_oe),
		.eri_o        (eri1),
		.rxi_o        (rxi1),
		.txi_o        (txi1),
		.tei_o        (tei1),
		.ss_din       (ss_din),
		.ss_addr      (ss_addr),
		.ss_wren      (ss_wren),
		.ss_rst       (ss_rst),
		.ss_dout      (ssq_sci1)
	);

	// ------------------------------------------------------------ the PFC
	wire [15:0] pa_dr, pb_dr, pb_out, pa_dir, pb_dir;
	wire [31:0] pa_fn, pb_fn;
	wire [1:0]  cs1_fn, cs3_fn;

	sh7021_pfc u_pfc (
		.clk_i    (clk_i),
		.ce_i     (ce_i),
		.rst_i    (rst),
		.psel_i   (psel),
		.paddr_i  (paddr),
		.pwr_i    (pwr),
		.pbe_i    (pbe),
		.pwdata_i (pwdata),
		.prdata_o (pr_pfc),
		.phit_o   (h_pfc),
		.pa_i     (pa_i),
		.pb_i     (pb_i),
		.pa_dr_o  (pa_dr),
		.pb_dr_o  (pb_dr),
		.pb_out_o (pb_out),
		.pa_dir_o (pa_dir),
		.pb_dir_o (pb_dir),
		.pa_fn_o  (pa_fn),
		.pb_fn_o  (pb_fn),
		.cs1_fn_o (cs1_fn),
		.cs3_fn_o (cs3_fn),
		.tpc_dr_i (tpc_dr),
		.tpc_en_i (tpc_en),
		.ss_din   (ss_din),
		.ss_addr  (ss_addr),
		.ss_wren  (ss_wren),
		.ss_rst   (ss_rst),
		.ss_dout  (ssq_pfc)
	);

	// --------------------------------------------------------- pin mapping
	// One name per pin's function field, to avoid variable part selects.
	wire [1:0] fa0 = pa_fn[1:0];
	wire [1:0] fa1 = pa_fn[3:2];
	wire [1:0] fa2 = pa_fn[5:4];
	wire [1:0] fa3 = pa_fn[7:6];
	wire [1:0] fa4 = pa_fn[9:8];
	wire [1:0] fa5 = pa_fn[11:10];
	wire [1:0] fa6 = pa_fn[13:12];
	wire [1:0] fa7 = pa_fn[15:14];
	wire [1:0] fa8 = pa_fn[17:16];
	wire [1:0] fa9 = pa_fn[19:18];
	wire [1:0] fa10 = pa_fn[21:20];
	wire [1:0] fa11 = pa_fn[23:22];
	wire [1:0] fa12 = pa_fn[25:24];
	wire [1:0] fa13 = pa_fn[27:26];
	wire [1:0] fa14 = pa_fn[29:28];
	wire [1:0] fa15 = pa_fn[31:30];

	wire [1:0] fb0 = pb_fn[1:0];
	wire [1:0] fb1 = pb_fn[3:2];
	wire [1:0] fb2 = pb_fn[5:4];
	wire [1:0] fb3 = pb_fn[7:6];
	wire [1:0] fb4 = pb_fn[9:8];
	wire [1:0] fb5 = pb_fn[11:10];
	wire [1:0] fb6 = pb_fn[13:12];
	wire [1:0] fb7 = pb_fn[15:14];
	wire [1:0] fb8 = pb_fn[17:16];
	wire [1:0] fb9 = pb_fn[19:18];
	wire [1:0] fb10 = pb_fn[21:20];
	wire [1:0] fb11 = pb_fn[23:22];
	wire [1:0] fb12 = pb_fn[25:24];
	wire [1:0] fb13 = pb_fn[27:26];
	wire [1:0] fb14 = pb_fn[29:28];
	wire [1:0] fb15 = pb_fn[31:30];

	// Dedicated chip selects, and the two that share a pin with a CAS strobe.
	assign cs0_n_o      = cs_n[0];
	assign cs2_n_o      = cs_n[2];
	assign cs1_cash_n_o = (cs1_fn == 2'b10) ? cash_n : cs_n[1];
	assign cs3_casl_n_o = (cs3_fn == 2'b10) ? casl_n : cs_n[3];

	// Port A: bus control, IRQ0-3, DREQ0/DACK0, DREQ1/DACK1, TIOC 0 and 1.
	wire [15:0] pa_alt_o = {
		1'b0,                                    // PA15 DREQ1 input
		dack[1],                                 // PA14
		1'b0,                                    // PA13 DREQ0 input
		dack[0],                                 // PA12
		tiocb_out[1], tioca_out[1],              // PA11, PA10
		(fa9 == 2'b01) ? ah : irqout,     // PA9
		1'b0,                                    // PA8 BREQ input
		1'b0,                                    // PA7 BACK
		rd_n, wrh_n, wrl_n,                      // PA6, PA5, PA4
		cs_n[7],                                 // PA3
		(fa2 == 2'b01) ? cs_n[6] : tiocb_out[0],
		(fa1 == 2'b01) ? cs_n[5] : ras_n,
		(fa0 == 2'b01) ? cs_n[4] : tioca_out[0]
	};
	wire [15:0] pa_alt_oe = {
		1'b0, (fa14 == 2'b11), 1'b0, (fa12 == 2'b11),
		(fa11 == 2'b10) & tiocb_oe[1],
		(fa10 == 2'b10) & tioca_oe[1],
		(fa9 != 2'b00),
		1'b0,
		(fa7 == 2'b01),
		(fa6 == 2'b01), (fa5 == 2'b01), (fa4 == 2'b01),
		(fa3 == 2'b01),
		(fa2 == 2'b01) | ((fa2 == 2'b10) & tiocb_oe[0]),
		(fa1 != 2'b00),
		(fa0 == 2'b01) | ((fa0 == 2'b10) & tioca_oe[0])
	};
	wire [15:0] pa_is_port = {
		fa15 == 2'b00, fa14 == 2'b00,
		fa13 == 2'b00, fa12 == 2'b00,
		fa11 == 2'b00, fa10 == 2'b00,
		fa9  == 2'b00, fa8  == 2'b00,
		fa7  == 2'b00, fa6  == 2'b00,
		fa5  == 2'b00, fa4  == 2'b00,
		fa3  == 2'b00, fa2  == 2'b00,
		fa1  == 2'b00, fa0  == 2'b00
	};

	assign pa_o    = (pa_dr & pa_is_port) | (pa_alt_o & ~pa_is_port);
	assign pa_oe_o = (pa_dir & pa_is_port) | (pa_alt_oe & ~pa_is_port);

	// Port B: SCI, TIOC 2 to 4, TCLKC/D, IRQ4-7 and the TPC pattern outputs.
	wire [15:0] pb_alt_o = {
		1'b0, 1'b0,                              // PB15, PB14: IRQ inputs
		sck1, sck0,                              // PB13, PB12
		txd1, 1'b0, txd0, 1'b0,                  // PB11, PB10, PB9, PB8
		1'b0, 1'b0,                              // PB7, PB6: TOCXB4, TOCXA4
		tiocb_out[4], tioca_out[4],              // PB5, PB4
		tiocb_out[3], tioca_out[3],              // PB3, PB2
		tiocb_out[2], tioca_out[2]               // PB1, PB0
	};
	wire [15:0] pb_alt_oe = {
		1'b0, 1'b0,
		(fb13 == 2'b10) & sck1_oe, (fb12 == 2'b10) & sck0_oe,
		(fb11 == 2'b10) & txd1_oe, 1'b0,
		(fb9  == 2'b10) & txd0_oe, 1'b0,
		1'b0, 1'b0,
		(fb5 == 2'b10) & tiocb_oe[4], (fb4 == 2'b10) & tioca_oe[4],
		(fb3 == 2'b10) & tiocb_oe[3], (fb2 == 2'b10) & tioca_oe[3],
		(fb1 == 2'b10) & tiocb_oe[2], (fb0 == 2'b10) & tioca_oe[2]
	};
	wire [15:0] pb_is_port = {
		fb15 == 2'b00, fb14 == 2'b00,
		fb13 == 2'b00, fb12 == 2'b00,
		fb11 == 2'b00, fb10 == 2'b00,
		fb9  == 2'b00, fb8  == 2'b00,
		fb7  == 2'b00, fb6  == 2'b00,
		fb5  == 2'b00, fb4  == 2'b00,
		fb3  == 2'b00, fb2  == 2'b00,
		fb1  == 2'b00, fb0  == 2'b00
	};
	wire [15:0] pb_alt_sel = {
		1'b0, 1'b0,
		fb13 == 2'b10, fb12 == 2'b10,
		fb11 == 2'b10, 1'b0, fb9 == 2'b10, 1'b0,
		1'b0, 1'b0,
		fb5 == 2'b10, fb4 == 2'b10,
		fb3 == 2'b10, fb2 == 2'b10,
		fb1 == 2'b10, fb0 == 2'b10
	};
	wire [15:0] pb_is_tp = {
		fb15 == 2'b11, fb14 == 2'b11, fb13 == 2'b11, fb12 == 2'b11,
		fb11 == 2'b11, fb10 == 2'b11, fb9  == 2'b11, fb8  == 2'b11,
		fb7  == 2'b11, fb6  == 2'b11, fb5  == 2'b11, fb4  == 2'b11,
		fb3  == 2'b11, fb2  == 2'b11, fb1  == 2'b11, fb0  == 2'b11
	};

	// A timing pattern pin drives the port register, which the TPC writes for
	// the bits NDER enables, and is an output whatever PBIOR says.
	assign pb_o    = (pb_dr & pb_is_port) | (pb_out & pb_is_tp)
	                 | (pb_alt_o & pb_alt_sel);
	assign pb_oe_o = (pb_dir & pb_is_port) | pb_is_tp
	                 | (pb_alt_oe & ~pb_is_port);

	// -------------------------------------------------------- pin inputs
	assign wait_n   = (fa3 == 2'b10) ? pa_i[3] : 1'b1;
	assign irq_n    = {(fb15 == 2'b01) ? pb_i[15] : 1'b1,
	                   (fb14 == 2'b01) ? pb_i[14] : 1'b1,
	                   (fb13 == 2'b01) ? pb_i[13] : 1'b1,
	                   (fb12 == 2'b01) ? pb_i[12] : 1'b1,
	                   (fa15 == 2'b01) ? pa_i[15] : 1'b1,
	                   (fa14 == 2'b01) ? pa_i[14] : 1'b1,
	                   (fa13 == 2'b01) ? pa_i[13] : 1'b1,
	                   (fa12 == 2'b01) ? pa_i[12] : 1'b1};
	assign dreq_n   = {(fa15 == 2'b11) ? pa_i[15] : 1'b1,
	                   (fa13 == 2'b11) ? pa_i[13] : 1'b1};
	assign tclk     = {(fb7 == 2'b01) ? pb_i[7] : 1'b0,
	                   (fb6 == 2'b01) ? pb_i[6] : 1'b0,
	                   (fa13 == 2'b10) ? pa_i[13] : 1'b0,
	                   (fa12 == 2'b10) ? pa_i[12] : 1'b0};
	assign tioca_in = {pb_i[4], pb_i[2], pb_i[0], pa_i[10], pa_i[0]};
	assign tiocb_in = {pb_i[5], pb_i[3], pb_i[1], pa_i[11], pa_i[2]};
	assign rxd0     = (fb8  == 2'b10) ? pb_i[8]  : 1'b1;
	assign rxd1     = (fb10 == 2'b10) ? pb_i[10] : 1'b1;

	// synthesis translate_off
	/* verilator lint_off UNUSEDSIGNAL */
	wire unused = &{1'b0, sleep, ubc_brk,
	                bsc_a, mem_addr, rom_sel, ram_sel};
	/* verilator lint_on UNUSEDSIGNAL */
	// synthesis translate_on
endmodule
