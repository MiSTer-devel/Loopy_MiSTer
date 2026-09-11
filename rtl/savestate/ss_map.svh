// Savestate scalar word allocation.
//
// Windows are fixed here so no two blocks can collide. STATESIZE_PARAM in
// Loopy.sv has to move with any layout change; it is what stops an old state
// file from loading into a new build.
//
// Included inside a module body, so every name is a module-scoped localparam.
// There is no include guard on purpose: one would hand the parameters to the
// first module of a compilation unit and leave the rest without them.
//
//    0- 15  SH-1 core          PC, SR, GBR, VBR, PR, pipeline registers
//   16- 23  SH-1 register file R0-R15
//   24- 27  SH-1 multiplier    MACH, MACL, the operand and accumulate latches
//   28- 31  spare, SH-1
//   32- 35  SH7021 BSC         area and refresh registers, cycle sequencer
//   36- 37  SH7021 INTC
//   38- 46  SH7021 DMAC        four channels, two words each, then the held
//                              transfer; its control word is at 70
//   47- 57  SH7021 ITU         five channels, two words each, then control
//   58- 59  SH7021 SCI0
//   60- 61  SH7021 SCI1
//   62- 64  SH7021 PFC
//   65- 66  SH7021 UBC
//   67      SH7021 WDT
//   68      SH7021 TPC
//   69      SH7021 internal bus arbiter
//   70      SH7021 DMAC        arbiter and transfer state
//   71- 79  spare, SH7021
//   80-111  RH-7500            registers, raster position, pending interrupts
//  112-127  sound and MIDI, including the synth's output queue
//  128-143  top level          cart slot, controller latches, CE phases
//  144-154  analogue filter    the two biquads' state and its last output
//  155-156  printer            mechanism position, capture pointer and hold
//  157-159  spare

/* verilator lint_off UNUSEDPARAM */

localparam int SS_INTERNAL_WORDS = 160;

localparam int    SSW_SH1_BASE    = 0;
localparam int    SSW_SH1_REGS    = 16;
localparam int    SSW_SH1_MAC     = 24;
localparam int    SSW_BSC_BASE    = 32;
localparam int    SSW_INTC_BASE   = 36;
localparam int    SSW_DMAC_BASE   = 38;
localparam int    SSW_DMAC_CTL    = 70;
localparam int    SSW_ITU_BASE    = 47;
localparam int    SSW_SCI0_BASE   = 58;
localparam int    SSW_SCI1_BASE   = 60;
localparam int    SSW_PFC_BASE    = 62;
localparam int    SSW_UBC_BASE    = 65;
localparam int    SSW_WDT_BASE    = 67;
localparam int    SSW_TPC_BASE    = 68;
localparam int    SSW_IBUS_BASE   = 69;
localparam int    SSW_VDP_BASE    = 80;
localparam int    SSW_SOUND_BASE  = 112;
localparam int    SSW_TOP_BASE    = 128;
localparam int    SSW_FILTER_BASE = 144;   // 11 words
localparam int    SSW_PRINTER     = 155;   // stepper index, paper, ribbon
localparam int    SSW_PRINTCAP    = 156;   // head capture pointer and hold

// RH-7500 words. Twelve carry the register file, the rest one block each; the
// window has 32.
localparam int    SSW_VDP_REGS    = SSW_VDP_BASE + 0;   // 12 words, 0-11
localparam int    SSW_VDP_RASTER  = SSW_VDP_BASE + 12;  // hcyc, vline, phase
localparam int    SSW_VDP_BITMAP  = SSW_VDP_BASE + 13;  // the four latched colours
localparam int    SSW_VDP_OBJ     = SSW_VDP_BASE + 14;  // object walk state
localparam int    SSW_VDP_CAPTURE = SSW_VDP_BASE + 15;  // capture arm and progress
localparam int    SSW_VDP_IOCTRL  = SSW_VDP_BASE + 16;  // controller latches, scan phase
localparam int    SSW_VDP_IOMOUSE = SSW_VDP_BASE + 17;  // mouse counters and buttons
localparam int    SSW_VDP_IOPRINT = SSW_VDP_BASE + 18;  // sensors, ADC, motor, head
localparam int    SSW_VDP_IOEXP   = SSW_VDP_BASE + 19;  // expansion timing, sound control

// Sound words. The voice state and wave line buffers are bulk memory and ride
// in SS_MEM_ONCHIP with the SH7021's RAM.
localparam int    SSW_SND_RH7501  = SSW_SOUND_BASE + 0;   // buttons, config state, sliders
localparam int    SSW_SND_MIDI    = SSW_SOUND_BASE + 1;   // receiver and parser
localparam int    SSW_SND_MIDIQ   = SSW_SOUND_BASE + 2;   // event queue
localparam int    SSW_SND_GLOBAL  = SSW_SOUND_BASE + 3;   // envelope counters, active voices
localparam int    SSW_SND_GLOBAL2 = SSW_SOUND_BASE + 4;   // sustained voices
localparam int    SSW_SND_VCH     = SSW_SOUND_BASE + 5;   // voice to channel map
localparam int    SSW_SND_CH      = SSW_SOUND_BASE + 6;   // 4 words, one per channel
localparam int    SSW_SND_PTR0    = SSW_SOUND_BASE + 10;  // wave ROM table pointers
localparam int    SSW_SND_PTR1    = SSW_SOUND_BASE + 11;
localparam int    SSW_SND_OQ      = SSW_SOUND_BASE + 12;  // 4 words, the output queue

// Top-level words in use so far.
localparam int    SSW_TOP_CART   = SSW_TOP_BASE + 0;   // cart slot latches

// Bulk memory types (Save_RAMType).
localparam [2:0] SS_MEM_WORKRAM  = 3'd0;   // 512 KB, streamed from SDRAM
localparam [2:0] SS_MEM_VRAM     = 3'd1;   // bitmap VRAM 128 KB
localparam [2:0] SS_MEM_TILERAM  = 3'd2;   // tile VRAM 64 KB
localparam [2:0] SS_MEM_OAM      = 3'd3;
localparam [2:0] SS_MEM_PALETTE  = 3'd4;
localparam [2:0] SS_MEM_CARTSRAM = 3'd5;   // run-time sized
localparam [2:0] SS_MEM_ONCHIP   = 3'd6;   // SH7021 RAM plus synth work RAM

/* verilator lint_on UNUSEDPARAM */
