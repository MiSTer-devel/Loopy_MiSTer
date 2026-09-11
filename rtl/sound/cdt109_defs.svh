// Copyright (c) 2026 Jamie Blanks

// One voice slot's state in the CDT109's working RAM: 32 slots, one 320-bit
// word (40 bytes) each. Field widths are sized to the wave ROM contents: a
// sample pointer covers the 512 KB ROM, and the pitch envelope's value and
// target stay within +/- 2^18 because no envelope target word exceeds 0x190.
//
// `active`, `sustained` and the voice's channel are kept as flat registers in
// the sequencer and sampler instead, since those read all 32 in one cycle.

// Each includer uses a subset of these.
/* verilator lint_off UNUSEDPARAM */

typedef struct packed {
	logic [17:0]        spare;

	logic signed [11:0] sample_cur;      // interpolation, the newer of the two
	logic signed [11:0] sample_last;
	logic [14:0]        sample_fract;    // 0..0x7FFF, wraps once per new word
	logic [18:0]        sample_ptr;      // 16-bit word index into the ROM
	logic [18:0]        sample_loop;
	logic [18:0]        sample_end;

	logic signed [12:0] pitch_env_rate;
	logic signed [19:0] pitch_env_target;
	logic signed [19:0] pitch_env_value;
	logic [16:0]        pitch_env_delay;
	logic [3:0]         pitch_env_step;
	logic [13:0]        pitch_env;       // index into the pitch envelope table

	logic [1:0]         volume_env_phase;   // divides the 884 Hz pass by four
	logic [9:0]         volume_env_delay;
	logic [4:0]         volume_env_step;    // bit 4 is the release phase
	logic [12:0]        volume_env;         // index into the volume envelope table
	logic               volume_down;
	logic [8:0]         volume_rate_counter;
	logic [8:0]         volume_rate_div;
	logic [15:0]        volume_rate_mul;
	logic [15:0]        volume_target;
	logic [15:0]        volume;
	logic signed [13:0] pitch;              // note offset from the sample's own
	logic [6:0]         note;               // the MIDI note, for note off
} voice_t;

localparam int VOICE_BITS = $bits(voice_t);

// Volume slider positions, scaled to 4096. Positions 2-4 are approximations
// of the hardware; 1 is a guess and nothing selects 0 or 1.
localparam logic [12:0] SLIDER_LEVEL [5] = '{13'd0, 13'd2048, 13'd2580, 13'd3251, 13'd4096};

// Fixed table bases in the wave ROM. The five pointed-at tables come from the
// image's own header instead.
localparam logic [18:0] ROM_RATETABLE = 19'h1000;
localparam logic [18:0] ROM_VOLTABLE  = 19'h1400;
localparam logic [18:0] ROM_PITCHBASE = 19'h1600;
localparam logic [18:0] ROM_INSTDESC  = 19'h2200;
localparam logic [18:0] ROM_KEYMAPS   = 19'h3DA0;

// The volume ramp moves by at most this much per step, which is what keeps a
// steep envelope from crackling.
localparam logic [15:0] VOLUME_RATE_LIMIT = 16'h01FF;

// The bugged drum sample in LSI352 underflows. A jump from one extreme to the
// other is treated as a wrap and clipped instead.
localparam logic signed [11:0] WRAP_MIN = -12'sd1888;   // -0x760
localparam logic signed [11:0] WRAP_MAX =  12'sd2046;   //  0x7FE

/* verilator lint_on UNUSEDPARAM */
