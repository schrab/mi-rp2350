// Port of OpenRB-338's PCF (pcf.rs) + compressor (comp.rs) to the standalone
// RP2350 test firmware. Same numerics law: f64, f32 stores at the DLL's points,
// no FMA. Tables are the engine's bit-exact DLL dumps.
#include "pcfcomp.h"
#include "tables.h"

#include <stdint.h>
#include <cstddef>

union D64b { uint64_t u; double d; };
union F32b { uint32_t u; float f; };
#define DC_(name, bits) static const D64b k_##name = { (bits) };
#define FC_(name, bits) static const F32b k_##name = { (bits) };

FC_(FREQ_SCALE, 0)
FC_(FREQ_SCALE2, 0x3C000000U)
FC_(TOP, 0x43FF8000U)
FC_(SLOW_KEEP, 0x3F7EB852U)
FC_(SLOW_TAKE, 0x3BA3D700U)
FC_(C127, 0x3C010204U)
FC_(ATTACK, 0x3C23D70AU)
FC_(RELEASE, 0x3F7F9724U)
FC_(GUARD, 0x358637BDU)
DC_(OFF_SCALE, 0x3FEFAE147AE147AEULL)

#define FREQ_SCALE (512.0)
#define FREQ_SCALE2 ((double)k_FREQ_SCALE2.f)
#define TOP_F ((double)k_TOP.f)
#define SLOW_KEEP (k_SLOW_KEEP.f)
#define SLOW_TAKE (k_SLOW_TAKE.f)
#define C127 ((double)k_C127.f)
#define ATTACK ((double)k_ATTACK.f)
#define RELEASE ((double)k_RELEASE.f)
#define GUARD ((double)k_GUARD.f)
#define OFF_SCALE (k_OFF_SCALE.d)

static inline float st(double x) { return (float)x; }

static const float PCF_DECAY[16] = {
    0.99f, 0.993f, 0.995f, 0.996f, 0.997f, 0.998f, 0.999f, 0.9993f,
    0.9995f, 0.9996f, 0.9997f, 0.9998f, 0.9999f, 0.99993f, 0.99995f, 1.0f
};

// --------------------------------------------------------------------- PCF
enum { PC_EN = 0, PC_FREQ, PC_RESO, PC_AMT, PC_WAVE, PC_DECAY, PC_MODE };

struct Pcf {
    uint8_t live[8];
    uint16_t counter;
    float smooth[2];
    float trigger, base, env;
    int32_t step, quotient;
    float attack, decay;
    float lp[2], bp[2];
};

static Pcf s_pcf;

static void pcf_init() {
    Pcf& p = s_pcf;
    const uint8_t d[8] = {1, 70, 64, 40, 24, 90, 1, 0};
    for (int i = 0; i < 8; ++i) p.live[i] = d[i];
    p.counter = 0; p.smooth[0] = 0; p.smooth[1] = 0;
    p.trigger = 0; p.base = 0; p.env = 0;
    p.step = -1; p.quotient = 0;
    p.attack = 0; p.decay = PCF_DECAY[0];
    p.lp[0] = p.lp[1] = 0; p.bp[0] = p.bp[1] = 0;
}

static uint16_t data_u16(int o) {
    return (uint16_t)g_pcf_data[o] | ((uint16_t)g_pcf_data[o + 1] << 8);
}

static void pattern(int wave, uint16_t& len, uint16_t& res) {
    int w = wave < 0 ? 0 : (wave > 127 ? 127 : wave);
    len = data_u16(w * 2);
    res = data_u16(0x70 + w * 2);
}

static int velocity(int wave, int step) {
    int w = wave < 0 ? 0 : (wave > 127 ? 127 : wave);
    int i = 0xE0 + w * 32 + step;
    return (i >= 0 && i < 4336) ? g_pcf_data[i] : 0;
}

static int32_t quotient(int tick, uint16_t res) {
    if (res == 32) return tick / 24;
    if (res == 16) return tick / 48;
    if (res == 8) return tick / 96;
    return s_pcf.quotient;   // the original leaves the stale stack word
}

static double pcf_smooth(uint8_t freq, float y1, float y2) {
    double f = ((double)freq * FREQ_SCALE) * FREQ_SCALE2 * (double)g_smooth_pcf[2];
    return ((double)g_smooth_pcf[1] * (double)y1 + f)
           - (double)g_smooth_pcf[0] * (double)y2;
}

static double trigger_level(int v, uint8_t amount, float base) {
    double raw = (double)((v * (int)amount) >> 5);
    return (raw + (double)base > TOP_F) ? TOP_F - (double)base : raw;
}

static int env_index(double e) {
    if (e < 0.0 || e != e) e = 0.0;
    if (e > TOP_F) e = TOP_F;
    return (int)e;
}

struct PcfEnv { double e, env, keep, take; };

static double env_step(PcfEnv& x, float dec, float base) {
    x.e *= (double)dec;
    x.env = x.env * x.keep + (x.e + (double)base) * x.take;
    return x.env;
}

static bool pcf_control(Pcf& p, int tick, PcfEnv& out) {
    uint16_t len, res;
    pattern(p.live[PC_WAVE], len, res);
    p.quotient = quotient(tick, res);
    double s = pcf_smooth(p.live[PC_FREQ], p.smooth[0], p.smooth[1]);
    p.smooth[1] = p.smooth[0];
    if (len == 0) return false;              // the DLL divide-by-zero fault
    int step = p.quotient % (int)len;
    p.smooth[0] = st(s);
    p.base = st(s);
    double e = p.trigger;
    double keep = (double)p.attack;
    double take = 1.0 - keep;
    if (step != p.step) {
        p.step = step;
        p.decay = PCF_DECAY[(p.live[PC_DECAY] >> 3) & 15];
        int v = velocity(p.live[PC_WAVE], step);
        if (v != 0) {
            e = trigger_level(v, p.live[PC_AMT], p.base);
            if (v & 1) { p.attack = SLOW_KEEP; keep = (double)SLOW_KEEP; take = (double)SLOW_TAKE; }
            else { p.attack = 0; keep = 0.0; take = 1.0; }
        }
    }
    out = { e, (double)p.env, keep, take };
    return true;
}

static void pcf_process(OrbBlock bus, int tick) {
    Pcf& p = s_pcf;
    if (p.live[PC_EN] == 0) return;
    p.counter++;
    PcfEnv env;
    if (!pcf_control(p, tick, env)) return;  // fault: bus passes unchanged
    int row = ((int)p.live[PC_RESO] * 8) >> 7;
    if (row > 7) row = 7;
    bool lowpass = p.live[PC_MODE] == 1;
    const float* tab = lowpass ? g_pcf_lp : g_pcf_bp;
    const float* coefs = &tab[(size_t)row * 512 * 3];
    double y1 = (double)p.lp[0], y2 = (double)p.lp[1];
    if (!lowpass) { y1 = (double)p.bp[0]; y2 = (double)p.bp[1]; }
    for (int g = 0; g < ORB_BLOCK / 4; ++g) {
        const float* c = &coefs[(size_t)env_index(env_step(env, p.decay, p.base)) * 3];
        for (int i = 4 * g; i < 4 * g + 4; ++i) {
            double prev = y1;
            double y = ((double)bus[i] * (double)c[2] + (double)c[0] * y1)
                       - (double)c[1] * y2;
            y2 = y1; y1 = y;
            bus[i] = st(lowpass ? y : y - prev);
        }
    }
    float s2[2] = { st(y1), st(y2) };
    if (lowpass) { p.lp[0] = s2[0]; p.lp[1] = s2[1]; p.bp[0] = p.bp[1] = 0; }
    else { p.bp[0] = s2[0]; p.bp[1] = s2[1]; p.lp[0] = p.lp[1] = 0; }
    p.trigger = st(env.e);
    p.env = st(env.env);
}

// ------------------------------------------------------------- compressor
enum { C_EN = 0, C_AMT, C_TH, C_GAIN_M, C_PEAK_M };

struct Comp {
    uint8_t live[5];
    uint16_t counter, hold;
    float x, off, env, gain, floor, peak;
};

static Comp s_comp;

static void comp_init() {
    Comp& c = s_comp;
    const uint8_t d[5] = {1, 70, 64, 127, 0};
    for (int i = 0; i < 5; ++i) c.live[i] = d[i];
    c.counter = 0; c.hold = 0;
    c.x = 1; c.off = 1; c.env = 1; c.gain = 0; c.floor = 1000; c.peak = 0;
}

static void comp_params(uint8_t amount, uint8_t threshold, float& x, float& off, float& floor) {
    double af = (double)amount * C127;
    double th = (double)threshold;
    off = st(1.0 - af * OFF_SCALE);
    x = st((af * (th * 0.03125 + 1.0)) * 0.3662109375);
    floor = st(((th * C127) * 80000.0) * (1.0 / 32768.0) + 0.01220703125);
}

static double comp_envelope(double e, double s, uint16_t& hold, float floor) {
    double v;
    if (s > e) { hold = 300; v = e + (s - e) * ATTACK; }
    else if (hold == 0) { v = e * RELEASE + GUARD; }
    else { hold--; v = e; }
    return (v < (double)floor || v != v) ? (double)floor : v;
}

static void comp_process(OrbBlock bus) {
    Comp& c = s_comp;
    if (c.live[C_EN] == 0) return;
    if ((c.counter & 15) == 0) comp_params(c.live[C_AMT], c.live[C_TH], c.x, c.off, c.floor);
    c.counter++;
    double env = c.env, g = c.gain, xd = (double)c.x, offd = (double)c.off;
    for (int i = 0; i < ORB_BLOCK; i += 4) {
        double s = (double)bus[i + 1] + (double)bus[i + 3];
        if (s < 0.0 || s != s) s = -s;
        env = comp_envelope(env, s, c.hold, c.floor);
        double d = ((xd / env + offd) - g) * 0.25;
        for (int k = 0; k < 4; ++k) {
            g += d;
            bus[i + k] = st((double)bus[i + k] * g);
        }
    }
    c.env = st(env);
    c.gain = st(g);
}

// ------------------------------------------------------------------- public
void pcfcomp_init() {
    pcf_init();
    comp_init();
}

void pcfcomp_set_param(int which, int index, uint8_t value) {
    if (which == 0 && index >= 0 && index < 8) s_pcf.live[index] = value;
    if (which == 1 && index >= 0 && index < 5) s_comp.live[index] = value;
}

void pcfcomp_process(OrbBlock bus, int tick) {
    pcf_process(bus, tick);
    comp_process(bus);
}
