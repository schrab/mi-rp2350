// sc_kit_reverb.h — SC 3.14.1 core reverbs (GPL-3.0, ReverbUGens.cpp).
// FreeVerb anchors: struct 28, Ctor 138 (calls next(unit,1) — one priming
// sample), next 208. 8 comb filters + 4 allpass filters, all float math.
// GVerb is added below once its law pass is green.
#ifndef RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
#define RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace sc {

struct FreeVerb {
  void Init(float mix, float room, float damp, float first_in) {
    iota_.fill(0);
    r_.fill(0.f);
    ar_.fill(0.f);
    float unused;
    Step(first_in, mix, room, damp, unused);  // ctor's next(unit, 1)
  }
  void ProcessBlock(const float* in, float* out, int n, float mix, float room,
                    float damp) {
    for (int i = 0; i < n; ++i) Step(in[i], mix, room, damp, out[i]);
  }

 private:
  void Step(float x, float mix, float room, float damp, float& out) {
    if (mix > 1.f) mix = 1.f;
    if (mix < 0.f) mix = 0.f;
    const float dry = 1.f - mix;
    if (room > 1.f) room = 1.f;
    if (room < 0.f) room = 0.f;
    const float fb = 0.700000f + 0.280000f * room;
    if (damp > 1.f) damp = 1.f;
    if (damp < 0.f) damp = 0.f;
    const float d = 0.400000f * damp;
    const float invd = 1.f - d;
    const float exc = 1.500000e-02f * x;

    float t[12];
    // Flat delay memory: the 12 lines have DIFFERENT lengths. The old
    // array-of-1617 allocated 19,404 floats; the exact sum is 12,587 —
    // 27,412 bytes saved per instance with zero numerical change (only the
    // indexing base changes). M33-relevant: 50.3 KB instead of 77.7 KB.
    for (int i = 0; i < 12; ++i) {
      if (++iota_[i] == kSize[i]) iota_[i] = 0;
      t[i] = lines_[static_cast<size_t>(kOffset[i] + iota_[i])];
    }
    // 8 comb stages (indices 4..11), each with its own low-pass state.
    for (int k = 0; k < 8; ++k) {
      const int line = k + 4, rlo = k * 2 + 4, rhi = rlo + 1;
      r_[rhi] = invd * r_[rlo] + d * r_[rhi];
      lines_[static_cast<size_t>(kOffset[line] + iota_[line])] = exc + fb * r_[rhi];
      r_[rlo] = t[line];
    }
    const float ftemp8 = r_[16] + r_[18];
    lines_[static_cast<size_t>(kOffset[3] + iota_[3])] =
        ((0.5f * r_[3] + r_[4]) + (r_[6] + r_[8])) +
        ((r_[10] + r_[12]) + (r_[14] + ftemp8));
    r_[3] = t[3];
    ar_[3] = r_[3] - ((r_[4] + r_[6]) + (r_[8] + r_[10]) +
                      ((r_[12] + r_[14]) + ftemp8));
    lines_[static_cast<size_t>(kOffset[2] + iota_[2])] = 0.5f * r_[2] + ar_[3];
    r_[2] = t[2];
    ar_[2] = r_[2] - ar_[3];
    lines_[static_cast<size_t>(kOffset[1] + iota_[1])] = 0.5f * r_[1] + ar_[2];
    r_[1] = t[1];
    ar_[1] = r_[1] - ar_[2];
    lines_[static_cast<size_t>(kOffset[0] + iota_[0])] = 0.5f * r_[0] + ar_[1];
    r_[0] = t[0];
    ar_[0] = r_[0] - ar_[1];
    out = dry * x + mix * ar_[0];
  }
  static constexpr int kSize[12] = {225, 341, 441, 556, 1617, 1557,
                                    1491, 1422, 1277, 1116, 1188, 1356};
  static constexpr int kOffset[12] = {0, 225, 566, 1007, 1563, 3180,
                                      4737, 6228, 7650, 8927, 10043, 11231};
  static constexpr int kTotal = 12587;
  std::array<int, 12> iota_{};
  std::array<float, 20> r_{};
  std::array<float, 4> ar_{};
  std::array<float, kTotal> lines_{};
};

// GVerb — ReverbUGens.cpp:1269 (ctor), 1387 (next), helpers 1038-1267.
// 4th-order FDN with prime-ish delays, 8 diffusers, dampers and a 44000-sample
// tap line. Ctor hardcodes inputbandwidth/dry/early/tail to 0; the first next()
// sees the real controls and calls the gverb_set_* functions, whose CALCSLOPE
// ramps them across that block. All math is float except alpha (double).
struct GVerb {
  void Init(double sr, float roomsize, float revtime, float damping,
            float inputbw, float spread, float dry, float early, float tail,
            float maxroomsize) {
    sr_ = static_cast<float>(sr);
    maxroomsize_ = maxroomsize > 1.0001f ? maxroomsize : 1.0001f;
    float rs = roomsize < 1.f ? 1.f : roomsize;
    if (rs >= maxroomsize_) rs = maxroomsize_ - 1.f;
    roomsize_ = rs;
    revtime_ = revtime;
    damping_ = damping;
    spread_ = spread;
    inputbw_ = 0.f;            // ctor hardcode; real value arrives in next()
    drylevel_ = 0.f;
    earlylevel_ = 0.f;
    taillevel_ = 0.f;
    const float maxdelay = sr_ * maxroomsize_ / 340.f;
    const float largest = sr_ * rs / 340.f;
    largest_ = largest;
    // Ctor uses pow((double)ga, 1./n) (double exponent) — gverb_set_revtime
    // (called only on a revtime change) uses powf(ga, 1.f/n) instead.
    const double n = static_cast<double>(sr_) * revtime_;
    alpha_ = std::pow(static_cast<double>(0.001f), 1.0 / n);
    const float gbmul[4] = {1.000f, 0.816490f, 0.707100f, 0.632450f};
    for (int i = 0; i < 4; ++i) {
      const float gb = gbmul[i] * largest;
      fdnlens_[i] = i == 0 ? NearestPrime(static_cast<int>(gb), 0.5f)
                           : FRound(gb);
      fdngains_[i] = static_cast<float>(-std::pow(
          alpha_, static_cast<double>(fdnlens_[i])));
    }
    for (int i = 0; i < 4; ++i) {
      fdndels_[i].Init(fdnlens_[i], static_cast<int>(maxdelay) + 1000);
      fdndamps_[i].damping = damping;
    }
    const float diffscale = fdnlens_[3] / (210.f + 159.f + 562.f + 410.f);
    const float spread1 = spread, spread2 = 3.f * spread;
    int a = static_cast<int>(spread1 * 0.125541f);
    int c = 210 + 159 + a, cc = c - 210;
    a = static_cast<int>(spread2 * 0.854046f);
    int d = 210 + 159 + 562 + a, dd = d - c, e = 1341 - d;
    ldifs_[0].Init(FRound(diffscale * 210), 0.75f);
    ldifs_[1].Init(FRound(diffscale * cc), 0.75f);
    ldifs_[2].Init(FRound(diffscale * dd), 0.625f);
    ldifs_[3].Init(FRound(diffscale * e), 0.625f);
    a = static_cast<int>(spread1 * -0.568366f);
    c = 210 + 159 + a; cc = c - 210;
    a = static_cast<int>(spread2 * -0.126815f);
    d = 210 + 159 + 562 + a; dd = d - c; e = 1341 - d;
    rdifs_[0].Init(FRound(diffscale * 210), 0.75f);
    rdifs_[1].Init(FRound(diffscale * cc), 0.75f);
    rdifs_[2].Init(FRound(diffscale * dd), 0.625f);
    rdifs_[3].Init(FRound(diffscale * e), 0.625f);
    taps_[0] = 5 + static_cast<int>(0.410f * largest);
    taps_[1] = 5 + static_cast<int>(0.300f * largest);
    taps_[2] = 5 + static_cast<int>(0.155f * largest);
    taps_[3] = 5;
    for (int i = 0; i < 4; ++i)
      tapgains_[i] = static_cast<float>(
          std::pow(alpha_, static_cast<double>(taps_[i])));
    tapdelay_.Init(44000, 44000);
    inputdamper_.damping = 1.f - inputbw_;
    fdngainslopes_.fill(0.f);
    tapgainslopes_.fill(0.f);
    dryslope_ = earlyslope_ = tailslope_ = 0.f;
    // next() will detect the non-zero controls and call the setters.
    targets_ = {roomsize, revtime, damping, inputbw, dry, early, tail};
  }
  void ProcessBlock(const float* in, float* left, float* right, int n,
                    float roomsize, float revtime, float damping,
                    float inputbw, float dry, float early, float tail) {
    if (roomsize != roomsize_ || revtime != revtime_ ||
        damping != damping_ || inputbw != inputbw_ ||
        dry != drylevel_ || early != earlylevel_ || tail != taillevel_) {
      SetRoomsize(roomsize);
      SetRevtime(revtime);
      SetDamping(damping);
      SetInputBandwidth(inputbw);
      dry = SetLevel(drylevel_, dry, dryslope_);
      early = SetLevel(earlylevel_, early, earlyslope_);
      tail = SetLevel(taillevel_, tail, tailslope_);
    }
    const float ds = dryslope_, es = earlyslope_, ts = tailslope_;
    const float* fds = fdngainslopes_.data();
    const float* tgs = tapgainslopes_.data();
    for (int i = 0; i < n; ++i) {
      const float x = in[i];
      float sum = 0.f, sign = 1.f;
      float z = inputdamper_.Do(x);
      z = ldifs_[0].Do(z);
      for (int j = 0; j < 4; ++j)
        u_[j] = tapgains_[j] * tapdelay_.Read(taps_[j]);
      tapdelay_.Write(z);
      for (int j = 0; j < 4; ++j)
        d_[j] = fdndamps_[j].Do(
            fdngains_[j] * fdndels_[j].Read(fdnlens_[j]));
      for (int j = 0; j < 4; ++j) {
        sum += sign * (tail * d_[j] + early * u_[j]);
        sign = -sign;
      }
      sum += x * early;
      float lsum = sum, rsum = sum;
      const float a0 = d_[0], a1 = d_[1], a2 = d_[2], a3 = d_[3];
      f_[0] = 0.5f * (+a0 + a1 - a2 - a3);
      f_[1] = 0.5f * (+a0 - a1 - a2 + a3);
      f_[2] = 0.5f * (-a0 + a1 - a2 + a3);
      f_[3] = 0.5f * (+a0 + a1 + a2 + a3);
      for (int j = 0; j < 4; ++j)
        fdndels_[j].Write(u_[j] + f_[j]);
      lsum = ldifs_[1].Do(lsum);
      lsum = ldifs_[2].Do(lsum);
      lsum = ldifs_[3].Do(lsum);
      rsum = rdifs_[1].Do(rsum);
      rsum = rdifs_[2].Do(rsum);
      rsum = rdifs_[3].Do(rsum);
      const float dryv = x * dry;
      left[i] = lsum + dryv;
      right[i] = rsum + dryv;
      dry += ds; early += es; tail += ts;
      for (int j = 0; j < 4; ++j) {
        fdngains_[j] += fds[j];
        tapgains_[j] += tgs[j];
      }
    }
    // NB: the source does NOT write the ramped locals back to unit->*level;
    // the setters already stored the TARGET, so a steady param triggers no
    // new ramp on the next block. Keeping the ramped value here would cause
    // a spurious 1-ULP re-ramp every block.
    // Source clears every slope at the end of each block (1511-1512): a
    // control change ramps for ONE block only, then the level holds.
    fdngainslopes_.fill(0.f);
    tapgainslopes_.fill(0.f);
    dryslope_ = earlyslope_ = tailslope_ = 0.f;
  }

 private:
  struct GDelay {
    std::vector<float> buf;
    int size = 0, idx = 0;
    void Init(int sz, int max) {
      size = sz; idx = 0; buf.assign(static_cast<size_t>(max), 0.f);
    }
    float Read(int samples) const {
      const int i = (idx - samples + size) % size;
      return buf[static_cast<size_t>(i)];
    }
    void Write(float x) {
      buf[static_cast<size_t>(idx)] = Zap(x);
      idx = (idx + 1) % size;
    }
  };
  struct GDamper {
    float damping = 0.f, delay = 0.f;
    float Do(float x) {
      const float y = x * (1.f - damping) + delay * damping;
      delay = Zap(y);
      return y;
    }
  };
  struct GDiffuser {
    std::vector<float> buf;
    int size = 0, idx = 0;
    float coef = 0.f;
    void Init(int sz, float c) {
      size = sz < 1 ? 1 : sz;
      buf.assign(static_cast<size_t>(size), 0.f);
      idx = 0; coef = c;
    }
    float Do(float x) {
      float w = x - buf[static_cast<size_t>(idx)] * coef;
      uint32_t bits;
      std::memcpy(&bits, &w, 4);
      if ((bits & 0x7f800000u) < 0x08000000u) w = 0.f;  // flush_to_zero
      const float y = buf[static_cast<size_t>(idx)] + w * coef;
      buf[static_cast<size_t>(idx)] = Zap(w);
      idx = (idx + 1) % size;
      return y;
    }
  };
  static float Zap(float x) {
    // SC_InlineUnaryOp.h:63 zapgremlins(float32)
    const float a = std::fabs(x);
    return (a > 1e-15f && a < 1e15f) ? x : 0.f;
  }
  static int FRound(float f) {
    const float g = f + static_cast<float>(3 << 22);
    int32_t bits;
    std::memcpy(&bits, &g, 4);
    return bits - 0x4b400000;
  }
  static bool IsPrime(int n) {
    if (n == 2) return true;
    if ((n & 1) == 0) return false;
    const int lim = static_cast<int>(std::sqrt(static_cast<float>(n)));
    for (int i = 3; i <= lim; i += 2) if (n % i == 0) return false;
    return true;
  }
  static int NearestPrime(int n, float rerror) {
    if (IsPrime(n)) return n;
    const int bound = static_cast<int>(n * static_cast<int>(rerror));
    for (int k = 1; k <= bound; ++k) {
      if (IsPrime(n + k)) return n + k;
      if (IsPrime(n - k)) return n - k;
    }
    return -1;
  }
  void SetRoomsize(float a) {
    if (a <= 1.f) a = 1.f;
    else if (a >= maxroomsize_) a = maxroomsize_ - 1.f;
    roomsize_ = a;
    largest_ = sr_ * a / 340.f;
    fdnlens_[1] = static_cast<int>(0.816490f * largest_);
    fdnlens_[2] = static_cast<int>(0.707100f * largest_);
    fdnlens_[3] = static_cast<int>(0.632450f * largest_);
    for (int i = 0; i < 4; ++i) {
      const float old = fdngains_[i];
      fdngains_[i] = static_cast<float>(
          -std::pow(alpha_, static_cast<double>(fdnlens_[i])));
      fdngainslopes_[i] = (fdngains_[i] - old) * (1.f / 64.f);
    }
    taps_[0] = 5 + static_cast<int>(0.410f * largest_);
    taps_[1] = 5 + static_cast<int>(0.300f * largest_);
    taps_[2] = 5 + static_cast<int>(0.155f * largest_);
    taps_[3] = 5;
    for (int i = 0; i < 4; ++i) {
      const float old = tapgains_[i];
      tapgains_[i] = static_cast<float>(
          std::pow(alpha_, static_cast<double>(taps_[i])));
      tapgainslopes_[i] = (tapgains_[i] - old) * (1.f / 64.f);
    }
  }
  void SetRevtime(float a) {
    revtime_ = a;
    const double nn = static_cast<double>(sr_) * a;
    alpha_ = static_cast<double>(std::pow(0.001f, 1.f / static_cast<float>(nn)));
    for (int i = 0; i < 4; ++i) {
      const float old = fdngains_[i];
      fdngains_[i] = static_cast<float>(
          -std::pow(alpha_, static_cast<double>(fdnlens_[i])));
      fdngainslopes_[i] = (fdngains_[i] - old) * (1.f / 64.f);
    }
  }
  void SetDamping(float a) {
    damping_ = a;
    for (int i = 0; i < 4; ++i) fdndamps_[i].damping = a;
  }
  void SetInputBandwidth(float a) {
    inputbw_ = a;
    inputdamper_.damping = 1.f - a;
  }
  static float SetLevel(float& cur, float a, float& slope) {
    const float old = cur;
    cur = a;
    slope = (a - old) * (1.f / 64.f);
    return old;
  }
  float sr_ = 48000.f, maxroomsize_ = 300.f, roomsize_ = 10.f;
  float revtime_ = 3.f, damping_ = 0.5f, spread_ = 15.f;
  float inputbw_ = 0.f, drylevel_ = 0.f, earlylevel_ = 0.f, taillevel_ = 0.f;
  float dryslope_ = 0.f, earlyslope_ = 0.f, tailslope_ = 0.f;
  float largest_ = 0.f;
  double alpha_ = 0.0;
  std::array<int, 4> fdnlens_{}, taps_{};
  std::array<float, 4> fdngains_{}, fdngainslopes_{}, tapgains_{},
      tapgainslopes_{}, u_{}, f_{}, d_{};
  std::array<GDelay, 4> fdndels_;
  std::array<GDamper, 4> fdndamps_;
  std::array<GDiffuser, 4> ldifs_, rdifs_;
  GDelay tapdelay_;
  GDamper inputdamper_;
  std::array<float, 7> targets_{};
};

}  // namespace sc
#endif  // RACK_SRC_DRONECASTER_SC_KIT_REVERB_H_
