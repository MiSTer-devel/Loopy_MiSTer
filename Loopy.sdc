derive_pll_clocks
derive_clock_uncertainty

# Three core clocks off one PLL (decisions 0016 and 0028):
#
#   general[0]  128.863636 MHz  clk_ram    SDRAM, four times clk_sys. The
#                                          fastest thing here and the first
#                                          place slack runs out.
#   general[1]   42.954545 MHz  clk_video  RH-7500 raster, render, video out
#   general[2]   32.215909 MHz  clk_sys    SH7021, sound, loaders, glue
#
# clk_ram and clk_sys are timed against each other on the strength of the
# exact 4:1 ratio: the memory path crosses them through rtl/mainboard/
# sync_xfer.sv and the direct work-DRAM path in ext_mem_bridge.sv, neither of
# which resynchronises. Everything that touches clk_video goes through
# rtl/cdc_handshake.sv, which does, so clk_video is cut from both. clk_ram has
# no relationship to clk_video and needs none: nothing in the video path
# touches SDRAM (decision 0027).
#
# The rule that goes with this: a crossing that is not covered here does not
# exist, and any change that adds one updates this file in the same commit.

set pll_out_0 {*|pll|pll_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk}
set pll_out_1 {*|pll|pll_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk}
set pll_out_2 {*|pll|pll_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk}

foreach pin [list $pll_out_0 $pll_out_1 $pll_out_2] {
	if {[get_collection_size [get_clocks $pin]] == 0} {
		post_message -type error "Loopy.sdc: no clock matched $pin"
	}
}

# clk_ram and clk_sys are in one group, which means they are NOT cut from each
# other: the memory path crosses between them with no synchroniser, on the
# strength of the 4:1 ratio, so those paths have to be timed. clk_video is on
# its own - the VDP is treated as the separate crystal it is on the real board,
# and everything crossing to it is resynchronised on the far side.
#
# Both patterns go into one get_clocks call. Passing [list [get_clocks a]
# [get_clocks b]] instead hands TimeQuest the two collection handles as text,
# which match no clock name, so the group comes out empty and the whole
# statement silently stops cutting anything - which is how clk_video came to be
# timed against clk_sys at a 7.759 ns relationship.
set grp_ram_sys [get_clocks [list $pll_out_0 $pll_out_2]]
if {[get_collection_size $grp_ram_sys] != 2} {
	post_message -type error "Loopy.sdc: clk_ram and clk_sys did not both resolve"
}

set_clock_groups -asynchronous \
	-group $grp_ram_sys \
	-group [get_clocks $pll_out_1]

# ---- clk_sys <-> clk_ram crossings ------------------------------------------
#
# Every path between the two is timed at the tightest edge pair, 7.76 ns,
# unless the RTL provably consumes it later. The ones below do; each says why.
# The direct work-DRAM path is deliberately absent: the line cache raises its
# request and payload on the same clk_sys edge and the bridge captures both on
# the first clk_ram edge after, and its answer (d_dout, d_done_tgl) is taken
# on the first clk_sys edge after it lands. Those are real 7.76 ns paths.

set clk_ram [get_clocks $pll_out_0]
set clk_sys [get_clocks $pll_out_2]

# sync_xfer request payload. data_reg is loaded on the clk_sys edge that
# raises pend. The bridge registers the request (p*_req_q) on the first
# clk_ram edge after, and the controller accepts and latches the address on
# the second. data_reg then holds until the round trip ends, so the earliest
# edge that reads it is two clk_ram periods out.
set xfer_data [get_registers {*|u_bridge|xfer_p*|data_reg*}]
if {[get_collection_size $xfer_data] == 0} {
	post_message -type error "Loopy.sdc: sync_xfer data_reg did not resolve"
}
set_multicycle_path -setup 2 -from $xfer_data -to $clk_ram
set_multicycle_path -hold  1 -from $xfer_data -to $clk_ram

# sync_xfer response. resp_reg is written on the clk_ram edge that raises
# acked. The clk_sys side sees acked on its next edge and only then registers
# src_done, and every client captures src_resp on src_done, so the first read
# of resp_reg is the second clk_sys edge after it was written.
set xfer_resp [get_registers {*|u_bridge|xfer_p*|resp_reg*}]
if {[get_collection_size $xfer_resp] == 0} {
	post_message -type error "Loopy.sdc: sync_xfer resp_reg did not resolve"
}
set_multicycle_path -setup 2 -from $xfer_resp -to $clk_sys
set_multicycle_path -hold  1 -from $xfer_resp -to $clk_sys

# The bridge's `direct` select samples `loading` and `ss_own`, two clk_sys
# levels that change only with the machine held and no work-DRAM transfer in
# flight, and stay put for far longer than one clk_sys period. Give the sample
# a whole clk_sys period.
set bridge_direct [get_registers {*|u_bridge|direct}]
if {[get_collection_size $bridge_direct] == 0} {
	post_message -type error "Loopy.sdc: ext_mem_bridge direct did not resolve"
}
set_multicycle_path -setup 4 -from $clk_sys -to $bridge_direct
set_multicycle_path -hold  3 -from $clk_sys -to $bridge_direct

# The HPS delivers content on clk_sys through hps_io, and the ioctl payload is
# held stable across the loader's handshake, so nothing extra is needed for it.
