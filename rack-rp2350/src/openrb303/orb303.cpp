// Standalone port of OpenRB-338's TB-303 + distortion to the Pico SDK.
// Source: crates/orb-engine/src/tb303.rs and dist.rs (engine numerics:
// f64, round to f32 exactly where the DLL stores a float, no FMA).
// Tables are the engine's bit-exact DLL dumps, linked via tables.S.
#include "orb303.h"
#include "tables.h"

#include <stddef.h>
#include <stdint.h>

// ---------------------------------------------------------------- constants
union D64 { uint64_t u; double d; };
union F32U { uint32_t u; float f; };

#define DC_(name, bits) static const D64 k_##name = { (bits) };
#define FC_(name, bits) static const F32U k_##name = { (bits) };

DC_(ACC_RESO_SCALE, 0x3F14A4299AE3DC33ULL)
DC_(ACC_RESO_BASE, 0x3FEFAE147AE147AEULL)
DC_(ACC_RESO_MUL, 0x3FEFF7CED916872BULL)
DC_(ACC_AMOUNT_SCALE, 0x3F983060C183060CULL)
DC_(DECAY_SPAN, 0x3F33A92A30553261ULL)
DC_(DECAY_BASE, 0x3FEFFFEB074A771DULL)
FC_(DECAY_SCALE, 0x3C010204U)
FC_(ACC_DECAY, 0x3F7FEBAFU)
FC_(AMP_VINTAGE_IN, 0x3DCCCCCDU)
FC_(AMP_VINTAGE_FB, 0x3F666666U)
FC_(AMP_OFF20_IN, 0x3D810625U)
FC_(AMP_OFF20_FB, 0x3F6FDF3BU)
DC_(CUT_SCALE, 0x3F62B4C5B45E7F8EULL)
DC_(CUT_BASE, 0x3FDD70A3D70A3D71ULL)
DC_(EM_SCALE, 0x3F6882F167EE957DULL)
DC_(EM_BASE, 0x3FB47AE147AE147BULL)
DC_(KA_SLOPE, 0x3FB1111111111111ULL)
DC_(KA_BASE, 0x3FBA06D3A06D3A07ULL)
DC_(ENV_BIAS, 0x3FD999999999999AULL)
FC_(OFFSET, 0x358637BDU)
FC_(RESO_VINTAGE, 0x3C010204U)
DC_(RESO_20, 0x3F80204081020408ULL)
FC_(RESO_ROW, 0x3C00FEB7U)
DC_(RAMP, 0x3FA0000000000000ULL)
FC_(DCFB, 0x3F7C60E3U)
FC_(SQ_THRESH, 0x3D3B8000U)
FC_(SQ_HI, 0x3F0CA000U)
FC_(SQ_LO, 0xBF0CA000U)

#define ACC_RESO_SCALE (k_ACC_RESO_SCALE.d)
#define ACC_RESO_BASE (k_ACC_RESO_BASE.d)
#define ACC_RESO_MUL (k_ACC_RESO_MUL.d)
#define ACC_AMOUNT_SCALE (k_ACC_AMOUNT_SCALE.d)
#define DECAY_SPAN (k_DECAY_SPAN.d)
#define DECAY_BASE (k_DECAY_BASE.d)
#define DECAY_SCALE ((double)k_DECAY_SCALE.f)
#define ACC_DECAY (k_ACC_DECAY.f)
#define AMP_VINTAGE_IN (k_AMP_VINTAGE_IN.f)
#define AMP_VINTAGE_FB (k_AMP_VINTAGE_FB.f)
#define AMP_OFF20_IN (k_AMP_OFF20_IN.f)
#define AMP_OFF20_FB (k_AMP_OFF20_FB.f)
#define CUT_SCALE (k_CUT_SCALE.d)
#define CUT_BASE (k_CUT_BASE.d)
#define EM_SCALE (k_EM_SCALE.d)
#define EM_BASE (k_EM_BASE.d)
#define KA_SLOPE (k_KA_SLOPE.d)
#define KA_BASE (k_KA_BASE.d)
#define ENV_BIAS (k_ENV_BIAS.d)
#define OFFSET ((double)k_OFFSET.f)
#define RESO_VINTAGE ((double)k_RESO_VINTAGE.f)
#define RESO_20 (k_RESO_20.d)
#define RESO_ROW ((double)k_RESO_ROW.f)
#define RAMP (k_RAMP.d)
#define DC ((double)k_DCFB.f)
#define SQ_THRESH ((double)k_SQ_THRESH.f)
#define SQ_HI ((double)k_SQ_HI.f)
#define SQ_LO ((double)k_SQ_LO.f)

#define ONE 1.0
#define ROWS 7.0
#define C_SCALE 1023.0
#define C_MAX 1.0
#define C_MIN 0.0
#define PHASE_MAX 1.0
#define PHASE_WRAP 2.0

// Live parameter byte indices (tb303.rs).
enum { P_ENABLED = 0, P_PATTERN, P_TUNE, P_CUTOFF, P_RESO, P_ENVMOD,
       P_DECAY, P_ACCENT, P_WAVEFORM };

static const int16_t TUNE_TABLE[32] = {
    -12, -12, -11, -10, -9, -8, -7, -6, -5, -4, -3, -2, -1, 0, 0, 0,
     0,   0,   1,   2,  3,  4,  5, 6,  7,  8,  9, 10, 11, 12, 12
};

static const uint8_t DEFAULT_PARAMS[12] =
    { 1, 0, 0x40, 0x14, 0x5A, 0x14, 0x64, 0x40, 0, 0, 0, 0 };

// ------------------------------------------------------------------ engine
struct Coefs { double c0, c1, g, c3; };
struct Voice { double ph, s, y1, y2, o; };

struct Tb303 {
    uint8_t live[12];
    bool vintage;
    bool slide;
    bool retrig;
    float phase, env, env_decay;
    uint32_t pitch_target, pitch;
    float acc_amount, acc_env, acc_coef;
    float cs1, cs2;
    float amp_in, amp_fb, gate, amp;
    float y1f, y2f, sf, of;
    float coefs[4];
};

static Tb303 s_303;
static bool s_gate_on;

static inline float st(double x) { return (float)x; }

static float square_n(float x, int n) {
    double v = (double)x;
    while (n-- > 0) v *= v;
    return (float)v;
}

static double clamp_c(double c) {
    if (c > C_MAX) c = C_MAX;
    if (c < C_MIN) c = C_MIN;
    return c;
}

static void orb_init_303(void) {
    Tb303& s = s_303;
    for (int i = 0; i < 12; ++i) s.live[i] = DEFAULT_PARAMS[i];
    s.vintage = true;                 // the classic 303 engine
    s.slide = false;
    s.retrig = false;
    s.phase = 0; s.env = 0; s.env_decay = 0;
    s.pitch_target = 0; s.pitch = 0;
    s.acc_amount = 0; s.acc_env = 0; s.acc_coef = 0;
    s.cs1 = 0.5f; s.cs2 = 0.5f;
    s.amp_in = 0; s.amp_fb = 0; s.gate = 0; s.amp = 0;
    s.y1f = 0; s.y2f = 0; s.sf = 0; s.of = 0;
    for (int i = 0; i < 4; ++i) s.coefs[i] = 0;
    s_gate_on = false;
}

static void note_on_semis(int semis, bool accent, bool slide) {
    Tb303& s = s_303;
    const int n = s.vintage ? 2 : 6;
    float c = st(((double)s.live[P_RESO] * ACC_RESO_SCALE + ACC_RESO_BASE)
                 * ACC_RESO_MUL);
    s.acc_coef = square_n(c, n);
    if (!accent) {
        s.acc_amount = 0;
        double d = ONE - (double)s.live[P_DECAY] * DECAY_SCALE;
        s.env_decay = st(DECAY_BASE - d * d * DECAY_SPAN);
    } else {
        s.env_decay = ACC_DECAY;
        s.acc_amount = st((double)s.live[P_ACCENT] * ACC_AMOUNT_SCALE);
    }
    s.env_decay = square_n(s.env_decay, n);
    s.gate = 1;
    s.amp_in = s.vintage ? AMP_VINTAGE_IN : 1.0f;
    s.amp_fb = s.vintage ? AMP_VINTAGE_FB : 0.0f;
    if (!slide) { s.env = 1; s.retrig = true; }
    int tune = TUNE_TABLE[s.live[P_TUNE] >> 2];
    s.pitch_target = (uint32_t)(((tune << 6) + (semis << 6) + 0xC00) << 4);
    if (!slide) s.pitch = s.pitch_target;
    s.slide = slide;
    s_gate_on = true;
}

void orb_note_on(int semis, bool accent, bool slide) {
    // Ignore a note the pitch table cannot represent (the original has no bound check).
    int tune = TUNE_TABLE[s_303.live[P_TUNE] >> 2];
    int sum = tune + semis;
    if (sum < -48 || sum > 47) return;
    note_on_semis(semis, accent, slide);
}

void orb_note_off(void) {
    if (s_303.slide) return;
    s_gate_on = false;
    s_303.gate = 0;
    s_303.amp_in = s_303.vintage ? AMP_VINTAGE_IN : AMP_OFF20_IN;
    s_303.amp_fb = s_303.vintage ? AMP_VINTAGE_FB : AMP_OFF20_FB;
}

static double glide(Tb303& s) {
    s.pitch = (uint32_t)((s.pitch * 11u + s.pitch_target) / 12u);
    return (double)g_pitch303[s.pitch >> 4];
}

static double smooth_cutoff(Tb303& s) {
    double cut = (double)s.live[P_CUTOFF] * CUT_SCALE + CUT_BASE;
    double cs = ((double)g_smooth303[1] * (double)s.cs1 + cut * (double)g_smooth303[2])
                - (double)g_smooth303[0] * (double)s.cs2;
    s.cs2 = s.cs1;
    s.cs1 = (float)cs;
    return cs;
}

static Voice voice(const Tb303& s) {
    Voice v;
    v.ph = s.phase; v.s = s.sf; v.y1 = s.y1f; v.y2 = s.y2f; v.o = s.of;
    return v;
}

static void store_voice(Tb303& s, const Voice& v) {
    s.phase = st(v.ph); s.sf = st(v.s); s.y1f = st(v.y1);
    s.y2f = st(v.y2); s.of = st(v.o);
}

static inline float sample(Voice& v, double inc, bool square, const Coefs& k) {
    double ph = v.ph + inc;
    if (ph > PHASE_MAX) ph -= PHASE_WRAP;
    v.ph = ph;
    double x = !square ? ph : (ph <= SQ_THRESH ? SQ_LO : SQ_HI);
    v.s = v.s * k.c3 + x;
    double y = (v.s * k.g + v.y1 * k.c0) - v.y2 * k.c1;
    v.o = (y - v.y1) + v.o * DC;
    v.y2 = v.y1;
    v.y1 = y;
    return st(v.o);
}

static void interpolate(float out[4], double c, double rf) {
    int idx = (int)(c * C_SCALE);                 // trunc toward zero
    double r7 = rf * ROWS;
    int l = (int)(uint16_t)(int64_t)r7;
    double frac = r7 - (double)l;
    const float* lo = &g_filt303[(size_t)(l * 1024 + idx) * 4];
    const float* hi = &g_filt303[(size_t)((l + 1) * 1024 + idx) * 4];
    for (int k = 0; k < 4; ++k)
        out[k] = st((double)lo[k] + ((double)hi[k] - (double)lo[k]) * frac);
}

static void process_vintage(Tb303& s, OrbBlock bus) {
    double r = (double)s.live[P_RESO] * RESO_VINTAGE;
    double cs = smooth_cutoff(s);
    double inc = glide(s);
    double amp = (double)s.amp + OFFSET;
    s.amp = st(amp);
    double e = OFFSET + (double)s.env;
    s.env = st(e);
    double ka = KA_BASE - r * KA_SLOPE;
    double em = (double)s.live[P_ENVMOD] * EM_SCALE + EM_BASE;
    double acc_in = (ONE - (double)s.acc_coef) * (double)s.acc_amount;
    double acc_coef = (double)s.acc_coef;
    double decay = (double)s.env_decay;
    const float* row = &g_filt303[(size_t)((s.live[P_RESO] >> 4) * 1024) * 4];
    bool square = s.live[P_WAVEFORM] != 0;
    Voice v = voice(s);
    for (int q = 0; q < ORB_BLOCK / 4; ++q) {
        double e1 = e * decay;
        double acc = acc_coef * (double)s.acc_env + e * acc_in;
        s.acc_env = st(acc);
        amp = amp * (double)s.amp_fb + (double)s.gate * (double)s.amp_in;
        double c = clamp_c(((e1 - ENV_BIAS) * em + cs) + acc * ka);
        double g = (acc + ONE) * amp;
        e = e1;
        const float* f = &row[(size_t)(int)(c * C_SCALE) * 4];
        Coefs k = { (double)f[0], (double)f[1], g * (double)f[2], (double)f[3] };
        for (int i = 0; i < 4; ++i) bus[q * 4 + i] = sample(v, inc, square, k);
    }
    store_voice(s, v);
    s.env = st(e);
    s.amp = st(amp);
}

static void process_20(Tb303& s, OrbBlock bus) {
    double cs = smooth_cutoff(s);
    double inc = glide(s);
    double amp0 = (double)s.amp + OFFSET;
    s.amp = st(amp0);
    double amp = amp0 * (double)s.amp_fb + (double)s.gate * (double)s.amp_in;
    double e0 = OFFSET + (double)s.env;
    s.env = st(e0);
    double acc_coef = (double)s.acc_coef;
    double acc = acc_coef * (double)s.acc_env
                 + e0 * ((ONE - acc_coef) * (double)s.acc_amount);
    s.acc_env = st(acc);
    double g = amp * (acc + ONE);
    double em = (double)s.live[P_ENVMOD] * EM_SCALE + EM_BASE;
    double r = (double)s.live[P_RESO] * RESO_20;
    double ka = KA_BASE - r * KA_SLOPE;
    double acc32 = (double)s.acc_env;
    double rf = (double)s.live[P_RESO] * RESO_ROW;
    if (s.retrig) {
        double c = clamp_c(ka * acc32 + ((e0 - ENV_BIAS) * em + cs));
        float k[4];
        interpolate(k, st(c), rf);
        s.coefs[0] = k[0]; s.coefs[1] = k[1]; s.coefs[2] = 0; s.coefs[3] = k[3];
        s.retrig = false;
    }
    double e1 = e0 * (double)s.env_decay;
    double c = clamp_c(ka * acc32 + ((e1 - ENV_BIAS) * em + cs));
    float n[4];
    interpolate(n, st(c), rf);
    double k0 = s.coefs[0], k1 = s.coefs[1], k2 = s.coefs[2], k3 = s.coefs[3];
    double d2 = ((double)n[2] * g - k2) * RAMP;
    double d0 = ((double)n[0] - k0) * RAMP;
    double d1 = ((double)n[1] - k1) * RAMP;
    double d3 = ((double)n[3] - k3) * RAMP;
    Coefs kc = { k0, k1, k2, k3 };
    bool square = s.live[P_WAVEFORM] != 0;
    Voice v = voice(s);
    for (int i = 0; i < ORB_BLOCK; i += 2) {
        bus[i] = sample(v, inc, square, kc);
        bus[i + 1] = sample(v, inc, square, kc);
        kc.c0 += d0; kc.c1 += d1; kc.g += d2; kc.c3 += d3;
    }
    s.coefs[0] = st(kc.c0); s.coefs[1] = st(kc.c1);
    s.coefs[2] = st(kc.g); s.coefs[3] = st(kc.c3);
    store_voice(s, v);
    s.env = st(e1);
    s.amp = st(amp);
}

// --------------------------------------------------------------- distortion
DC_(GRID_SCALE, 0x400FFDF3B645A1CBULL)
DC_(KEEP, 0x3FECCCCCCCCCCCCDULL)
DC_(TAKE, 0x3FB999999999999AULL)
FC_(C127, 0x3C010204U)

#define GRID_SCALE (k_GRID_SCALE.d)
#define KEEP (k_KEEP.d)
#define TAKE (k_TAKE.d)
#define C127 ((double)k_C127.f)

static const float GRID[5][5] = {
    { 1.2f, 0.4f, 0.28f, 0.2f, 0.15f },
    { 0.35f, 0.2f, 0.15f, 0.14f, 0.12f },
    { 0.2f, 0.15f, 0.13f, 0.12f, 0.12f },
    { 0.15f, 0.12f, 0.11f, 0.1f, 0.1f },
    { 0.12f, 0.1f, 0.08f, 0.08f, 0.08f },
};

struct Dist {
    uint8_t live[4];       // enabled, amount, shape, meter
    uint16_t counter;
    bool clip;
    float gain_in, gain_out, target_in, target_out;
    uint16_t row;
};

static Dist s_dist;

static inline double coord(int x) { return ((double)x * C127) * GRID_SCALE; }

static double grid(int shape, int amount) {
    double sv = coord(shape), av = coord(amount);
    int i = (int)(int64_t)sv, j = (int)(int64_t)av;   // fistp truncation
    double fs = sv - (double)i, fa = av - (double)j;
    double top = ((double)GRID[i + 1][j + 1] - (double)GRID[i + 1][j]) * fa
                 + (double)GRID[i + 1][j];
    double bot = ((double)GRID[i][j + 1] - (double)GRID[i][j]) * fa
                 + (double)GRID[i][j];
    return fs * (top - bot) + bot;
}

static double shaper(const float* w, double v) {
    if (v < 0.0) return (double)w[0];
    if (v > 254.0) return (double)w[254];
    int i = (int)(int64_t)v;
    return (v - (double)i) * ((double)w[i + 1] - (double)w[i]) + (double)w[i];
}

static float dsmooth(float gain, float target) {
    return st((double)gain * KEEP + (double)target * TAKE);
}

static void orb_init_dist(void) {
    Dist& d = s_dist;
    // Shape 60 selects the waveshaper row (the engine's shape=0 default is a
    // hard clip, which is less useful for an audible bring-up test).
    d.live[0] = 1; d.live[1] = 70; d.live[2] = 60; d.live[3] = 0;
    d.counter = 0; d.clip = false;
    d.gain_in = 0; d.gain_out = 0; d.target_in = 0; d.target_out = 0;
    d.row = 0;
}

static void dist_update(void) {
    Dist& d = s_dist;
    int shape = d.live[2], amount = d.live[1];
    d.clip = (shape == 0);
    d.target_in = st(((double)g_level[amount] + 0.0625) * 2048.0);
    d.row = (uint16_t)((unsigned)shape >> 2);
    d.target_out = st(grid(shape, amount) * 5.0 * 0.00390625);
}

static void dist_process(OrbBlock bus) {
    Dist& d = s_dist;
    if (d.live[0] == 0) return;
    if ((d.counter & 15) == 0) dist_update();
    d.counter++;
    d.gain_in = dsmooth(d.gain_in, d.target_in);
    d.gain_out = dsmooth(d.gain_out, d.target_out);
    if (d.clip) {
        double g = 16.0 * (double)g_level[d.live[1]] + 1.0;
        double h = d.live[1] < 90 ? g : 16.0 * (double)g_level[90] + 1.0;
        double th = 1.0 / h, nth = -th;
        for (int i = 0; i < ORB_BLOCK; ++i) {
            double y = g * (double)bus[i];
            bus[i] = st(y > th ? th : (y < nth ? nth : y));
        }
    } else {
        const float* w = &g_dist_w[(size_t)d.row * 256];
        double gi = (double)d.gain_in, go = (double)d.gain_out;
        for (int i = 0; i < ORB_BLOCK; ++i) {
            double v = gi * (double)bus[i] + 128.0;
            bus[i] = st(shaper(w, v) * go);
        }
    }
}

// ------------------------------------------------------------------ public
static int s_tick = 0;

void orb_init(void) {
    orb_init_303();
    orb_init_dist();
    s_tick = 0;
}

void orb_set_param(int index, uint8_t value) {
    if (index >= 0 && index < 12) s_303.live[index] = value;
}
void orb_set_303_param(int index, uint8_t value) {
    if (index >= 0 && index < 12) s_303.live[index] = value;
}
void orb_set_dist_param(int index, uint8_t value) {
    if (index >= 0 && index < 4) s_dist.live[index] = value;
}

// A small hard-coded acid line: 16 steps, 48 ticks each.
static const int8_t SEQ_NOTE[16] = {
     0,  0, 12,  0,  3,  0,  7, 10,
     0, 12,  0,  3,  0,  7,  0, -2
};
static const uint8_t SEQ_ACC[16] = {
     1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 0
};

void orb_tick_step(int tick) {
    s_tick = tick;
    int step = (tick / 48) & 15;
    if ((tick % 48) == 0) {
        orb_note_on(SEQ_NOTE[step], SEQ_ACC[step] != 0, false);
    } else if ((tick % 48) == 36) {
        orb_note_off();
    }
}

void orb_process(OrbBlock bus) {
    for (int i = 0; i < ORB_BLOCK; ++i) bus[i] = 0.0f;
    if (s_303.vintage) process_vintage(s_303, bus);
    else process_20(s_303, bus);
    dist_process(bus);
}
