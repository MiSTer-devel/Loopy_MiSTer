// Copyright (c) 2026 Jamie Blanks
//
// Control encodings shared by the SH-1 decoder and the core datapath.
// Included inside a module body, so every name below is a module-scoped
// localparam. Each module uses a subset, so the unused-parameter check is
// off for the whole file.

/* verilator lint_off UNUSEDPARAM */
// ---------------------------------------------------------------- operand A
localparam [3:0] A_ZERO = 4'd0;   // 0
localparam [3:0] A_RN   = 4'd1;
localparam [3:0] A_RM   = 4'd2;
localparam [3:0] A_R0   = 4'd3;
localparam [3:0] A_PC   = 4'd4;   // address of this instruction + 4
localparam [3:0] A_PC4  = 4'd5;   // (address + 4) with the low two bits cleared
localparam [3:0] A_GBR  = 4'd6;
localparam [3:0] A_VBR  = 4'd7;
localparam [3:0] A_SR   = 4'd8;
localparam [3:0] A_PR   = 4'd9;

// ---------------------------------------------------------------- operand B
localparam [3:0] B_ZERO = 4'd0;
localparam [3:0] B_RN   = 4'd1;
localparam [3:0] B_RM   = 4'd2;
localparam [3:0] B_IMM  = 4'd4;
localparam [3:0] B_GBR  = 4'd5;

// -------------------------------------------------------------- ALU opcodes
localparam [5:0] AOP_ADD    = 6'd0;
localparam [5:0] AOP_ADDC   = 6'd1;
localparam [5:0] AOP_ADDV   = 6'd2;
localparam [5:0] AOP_SUB    = 6'd3;
localparam [5:0] AOP_SUBC   = 6'd4;
localparam [5:0] AOP_SUBV   = 6'd5;
localparam [5:0] AOP_AND    = 6'd6;
localparam [5:0] AOP_OR     = 6'd7;
localparam [5:0] AOP_XOR    = 6'd8;
localparam [5:0] AOP_NOT    = 6'd9;
localparam [5:0] AOP_CMPEQ  = 6'd10;
localparam [5:0] AOP_CMPHS  = 6'd11;
localparam [5:0] AOP_CMPGE  = 6'd12;
localparam [5:0] AOP_CMPHI  = 6'd13;
localparam [5:0] AOP_CMPGT  = 6'd14;
localparam [5:0] AOP_CMPPZ  = 6'd15;
localparam [5:0] AOP_CMPPL  = 6'd16;
localparam [5:0] AOP_CMPSTR = 6'd17;
localparam [5:0] AOP_TST    = 6'd18;
localparam [5:0] AOP_XTRCT  = 6'd19;
localparam [5:0] AOP_SWAPB  = 6'd20;
localparam [5:0] AOP_SWAPW  = 6'd21;
localparam [5:0] AOP_EXTSB  = 6'd22;
localparam [5:0] AOP_EXTSW  = 6'd23;
localparam [5:0] AOP_EXTUB  = 6'd24;
localparam [5:0] AOP_EXTUW  = 6'd25;
localparam [5:0] AOP_DIV0S  = 6'd26;
localparam [5:0] AOP_DIV0U  = 6'd27;
localparam [5:0] AOP_DIV1   = 6'd28;
localparam [5:0] AOP_PASSB  = 6'd29;
localparam [5:0] AOP_MOVT   = 6'd30;
localparam [5:0] AOP_SHIFT  = 6'd31;
localparam [5:0] AOP_SETT   = 6'd32;
localparam [5:0] AOP_CLRT   = 6'd33;

// ---------------------------------------------------------- shifter opcodes
localparam [3:0] SH_SHLL   = 4'd0;
localparam [3:0] SH_SHLR   = 4'd1;
localparam [3:0] SH_SHAL   = 4'd2;
localparam [3:0] SH_SHAR   = 4'd3;
localparam [3:0] SH_ROTL   = 4'd4;
localparam [3:0] SH_ROTR   = 4'd5;
localparam [3:0] SH_ROTCL  = 4'd6;
localparam [3:0] SH_ROTCR  = 4'd7;
localparam [3:0] SH_SHLL2  = 4'd8;
localparam [3:0] SH_SHLR2  = 4'd9;
localparam [3:0] SH_SHLL8  = 4'd10;
localparam [3:0] SH_SHLR8  = 4'd11;
localparam [3:0] SH_SHLL16 = 4'd12;
localparam [3:0] SH_SHLR16 = 4'd13;

// ------------------------------------- register write targets (EX and MA/WB)
localparam [3:0] W_NONE = 4'd0;
localparam [3:0] W_RN   = 4'd1;
localparam [3:0] W_RM   = 4'd2;
localparam [3:0] W_R0   = 4'd3;
localparam [3:0] W_SR   = 4'd4;
localparam [3:0] W_GBR  = 4'd5;
localparam [3:0] W_VBR  = 4'd6;
localparam [3:0] W_PR   = 4'd7;
localparam [3:0] W_MACH = 4'd8;
localparam [3:0] W_MACL = 4'd9;
localparam [3:0] W_PC   = 4'd10;  // RTE only, from the sequencer

// ------------------------------------------------------ store data selection
localparam [3:0] D_RM   = 4'd0;
localparam [3:0] D_RN   = 4'd1;
localparam [3:0] D_R0   = 4'd2;
localparam [3:0] D_SR   = 4'd3;
localparam [3:0] D_GBR  = 4'd4;
localparam [3:0] D_VBR  = 4'd5;
localparam [3:0] D_PR   = 4'd6;
localparam [3:0] D_MACH = 4'd7;
localparam [3:0] D_MACL = 4'd8;
localparam [3:0] D_SEQ  = 4'd9;   // value produced by the multi-step sequencer

// -------------------------------------------------------- memory-access size
localparam [1:0] SZ_B = 2'd0;
localparam [1:0] SZ_W = 2'd1;
localparam [1:0] SZ_L = 2'd2;

// ----------------------------------------------------------- MA stage action
localparam [2:0] MA_NONE  = 3'd0;
localparam [2:0] MA_LOAD  = 3'd1;  // bus read, result to ld_sel
localparam [2:0] MA_STORE = 3'd2;  // bus write of sd_sel
localparam [2:0] MA_MACWR = 3'd3;  // write the multiplier, no bus cycle
localparam [2:0] MA_MACRD = 3'd4;  // read the multiplier into ld_sel, no bus cycle
localparam [2:0] MA_MUL   = 3'd5;  // start MULS.W/MULU.W, no bus cycle

// ------------------------------------------------------- multiplier commands
localparam [2:0] MACOP_NONE = 3'd0;
localparam [2:0] MACOP_CLR  = 3'd1;
localparam [2:0] MACOP_LDH  = 3'd2;
localparam [2:0] MACOP_LDL  = 3'd3;
localparam [2:0] MACOP_MULS = 3'd4;
localparam [2:0] MACOP_MULU = 3'd5;
localparam [2:0] MACOP_MACW = 3'd6;

// ------------------------------------------------------------ branch classes
localparam [2:0] BR_NONE   = 3'd0;
localparam [2:0] BR_BT     = 3'd1;  // not delayed, taken when T = 1
localparam [2:0] BR_BF     = 3'd2;  // not delayed, taken when T = 0
localparam [2:0] BR_DISP   = 3'd3;  // BRA
localparam [2:0] BR_DISP_L = 3'd4;  // BSR, links PR
localparam [2:0] BR_REG    = 3'd5;  // JMP
localparam [2:0] BR_REG_L  = 3'd6;  // JSR, links PR
localparam [2:0] BR_RTS    = 3'd7;

// ---------------------------------------------------- multi-step instruction
localparam [3:0] SP_NONE   = 4'd0;
localparam [3:0] SP_LOGMEM = 4'd1;  // AND.B/OR.B/XOR.B/TST.B #imm,@(R0,GBR)
localparam [3:0] SP_TAS    = 4'd2;
localparam [3:0] SP_LDCL   = 4'd3;  // LDC.L @Rm+,SR/GBR/VBR
localparam [3:0] SP_MACW   = 4'd4;
localparam [3:0] SP_RTE    = 4'd5;
localparam [3:0] SP_TRAPA  = 4'd6;
localparam [3:0] SP_SLEEP  = 4'd7;
// Sequence classes entered by the core itself.
localparam [3:0] SP_EXC    = 4'd8;   // exception or interrupt entry
localparam [3:0] SP_RESET  = 4'd9;   // reset vector fetch

// Which logic operation SP_LOGMEM performs on the byte it read. The ALU stays
// on ADD for these because it is computing R0 + GBR at the same time.
localparam [1:0] LOP_AND = 2'd0;
localparam [1:0] LOP_OR  = 2'd1;
localparam [1:0] LOP_XOR = 2'd2;
localparam [1:0] LOP_TST = 2'd3;

/* verilator lint_on UNUSEDPARAM */
