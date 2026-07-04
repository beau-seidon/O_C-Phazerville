#pragma once
// Platform-independent DSP core for a giant reverb
//
// Topology: Jon Dattorro's figure-eight reverb tank ("Effect Design Part 1",
// JAES 1997), scaled up to 2.5x plate dimensions and tuned to resemble the
// Eventide Blackhole character:
//   - long pre-delay (up to 500ms)
//   - 4 series input diffusers with high diffusion coefficients
//   - two cross-coupled tank branches, each: modulated allpass -> long delay
//     -> damping (hi) + low-cut (lo) -> decay gain -> allpass -> long delay
//   - GRAVITY maps to loop decay; at max it goes past unity ("infinite"),
//     with a soft saturator in the tank guaranteeing stability
//   - negative GRAVITY engages "inverse" mode: the wet output is ducked by
//     an input envelope follower, so the tail blooms in reverse-like swells
//     after each note (an approximation of the Blackhole's inverse gravity)
//   - SIZE rescales every delay/allpass length (slewed, so sweeps warp
//     pitch smoothly rather than clicking)
//
// v2 performance changes (same topology & controls as v1):
//   - ALL delay lines store int16 (arena ~147 KB instead of ~287 KB, so it
//     fits comfortably in Teensy RAM2 instead of spilling to slow PSRAM)
//   - nearest-sample reads for output taps, feedback taps and fixed
//     allpasses; interpolation only where it's audible (the two modulated
//     allpasses and the pre-delay)
//   - polynomial sine LFO instead of two sinf() calls per sample
//   - division-free cubic soft-clip instead of the tanh Pade approx

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cmath>

class AbyssCore {
public:
  static constexpr float kRefRate = 29761.0f; // Dattorro's reference rate
  static constexpr float kMinScale = 0.35f;
  static constexpr float kMaxScale = 2.5f;
  static constexpr float kMaxPredelaySec = 0.5f;

  // ---- memory ----
  // All delay lines are carved out of a single caller-provided arena so the
  // wrapper can place it in fast RAM2 (preferred) or PSRAM in one shot.
  size_t RequiredBytes(float sample_rate) {
    Dims d;
    ComputeDims(sample_rate, d);
    return d.total_bytes;
  }

  bool Init(float sample_rate, void* arena, size_t arena_bytes) {
    sr = sample_rate;
    Dims d;
    ComputeDims(sample_rate, d);
    if (!arena || arena_bytes < d.total_bytes) return false;

    uint8_t* p = static_cast<uint8_t*>(arena);
    size_t used = 0;
    // scale 16384 (+-2.0) for lines fed post-saturator; 8192 (+-4.0) for
    // allpasses, whose internal node can run hotter
    auto carve = [&](Line& l, int32_t cap, float scale) {
      l.buf = reinterpret_cast<int16_t*>(p + used);
      l.cap = cap;
      l.wr = 0;
      l.scale = scale;
      l.inv = 1.0f / scale;
      l.lim = 32767.0f;
      used += Align4(cap * sizeof(int16_t));
    };

    carve(pre, d.cap_pre, 16384.0f);
    carve(dA1, d.cap_dA1, 16384.0f);
    carve(dA2, d.cap_dA2, 16384.0f);
    carve(dB1, d.cap_dB1, 16384.0f);
    carve(dB2, d.cap_dB2, 16384.0f);
    for (int i = 0; i < 4; ++i) carve(inAP[i], d.cap_inAP[i], 8192.0f);
    carve(apA1, d.cap_apA1, 8192.0f);
    carve(apA2, d.cap_apA2, 8192.0f);
    carve(apB1, d.cap_apB1, 8192.0f);
    carve(apB2, d.cap_apB2, 8192.0f);

    // base lengths (samples at current rate, before SIZE scaling)
    const float ratio = sr / kRefRate;
    for (int i = 0; i < 4; ++i) inApBase[i] = kInAP[i] * ratio;
    apA1Base = kApA1 * ratio; dA1Base = kDA1 * ratio;
    apA2Base = kApA2 * ratio; dA2Base = kDA2 * ratio;
    apB1Base = kApB1 * ratio; dB1Base = kDB1 * ratio;
    apB2Base = kApB2 * ratio; dB2Base = kDB2 * ratio;

    // output tap positions (Dattorro's tap table, same units)
    for (int i = 0; i < 7; ++i) {
      tapL[i] = kTapL[i] * ratio;
      tapR[i] = kTapR[i] * ratio;
    }

    // slew coefficients
    fastSlew = OnePoleCoef(0.008f);   // gains, damping, mod depth
    scaleSlew = OnePoleCoef(0.20f);   // SIZE sweeps
    preSlew = OnePoleCoef(0.10f);     // predelay changes
    envAtk = OnePoleCoef(0.004f);     // inverse-mode envelope attack
    loCoef = 1.0f - expf(-6.2831853f * 180.0f / sr); // ~180 Hz low shelf

    // make current == target so we start in a defined state
    scaleCur = scaleTgt; decayCur = decayTgt; dampCur = dampTgt;
    lowCur = lowTgt; excCur = excTgt; invCur = invTgt; preCur = preTgt;
    SetModRate(rateHz);
    SetSize(sizeNorm); // refresh env release for current sr

    Clear();
    ready_ = true;
    return true;
  }

  void Clear() {
    Zero(pre); Zero(dA1); Zero(dA2); Zero(dB1); Zero(dB2);
    for (int i = 0; i < 4; ++i) Zero(inAP[i]);
    Zero(apA1); Zero(apA2); Zero(apB1); Zero(apB2);
    lpA = lpB = loA = loB = 0.0f;
    env = 0.0f;
    phase = 0.0f;
  }

  void Release() { ready_ = false; }
  bool Ready() const { return ready_; }

  // ---- parameters ----
  // Single-word float stores: safe to call from a different context than
  // Process() on ARM (and on any host for testing purposes).

  // g in [-1, 1]. |g| sets decay; negative engages inverse (swell) mode.
  void SetGravity(float g) {
    g = Clampf(g, -1.0f, 1.0f);
    const float a = fabsf(g);
    decayTgt = Clampf(0.55f + 0.47f * a, 0.0f, 1.02f);
    invTgt = (g < 0.0f) ? (0.4f + 0.6f * a) : 0.0f;
  }

  // s in [0, 1] -> tank scale kMinScale..kMaxScale
  void SetSize(float s) {
    s = Clampf(s, 0.0f, 1.0f);
    sizeNorm = s;
    scaleTgt = kMinScale + (kMaxScale - kMinScale) * s;
    if (sr > 0.0f) envRel = OnePoleCoef(0.25f + 1.2f * s);
  }

  void SetPredelay(float seconds) {
    preTgt = Clampf(seconds, 0.0f, kMaxPredelaySec) * sr;
  }

  // d in [0, 1] -> up to ~1.5ms excursion on the tank allpasses
  void SetModDepth(float d) {
    excTgt = Clampf(d, 0.0f, 1.0f) * 0.0015f * sr;
  }

  void SetModRate(float hz) {
    rateHz = Clampf(hz, 0.01f, 5.0f);
    if (sr > 0.0f) phaseInc = 2.0f * rateHz / sr; // phase in [-1,1) turns*2
  }

  // v in [0, 1]: amount of low frequencies removed from the loop
  void SetLoCut(float v) { lowTgt = Clampf(v, 0.0f, 1.0f) * 0.7f; }

  // v in [0, 1]: high-frequency damping in the loop (0 = bright)
  void SetHiDamp(float v) { dampTgt = Clampf(v, 0.0f, 1.0f) * 0.9f; }

  // MONO applets get the full tank folded to one channel
  void SetMonoSum(bool b) { monoSum = b; }

  // ---- audio ----
  inline void Process(float inL, float inR, float& outL, float& outR) {
    if (!ready_) { outL = 0.0f; outR = 0.0f; return; }

    // parameter smoothing
    scaleCur += scaleSlew * (scaleTgt - scaleCur);
    decayCur += fastSlew * (decayTgt - decayCur);
    dampCur  += fastSlew * (dampTgt - dampCur);
    lowCur   += fastSlew * (lowTgt - lowCur);
    excCur   += fastSlew * (excTgt - excCur);
    invCur   += fastSlew * (invTgt - invCur);
    preCur   += preSlew  * (preTgt - preCur);

    // tank modulation LFOs: cheap polynomial sine on a [-1,1) phasor,
    // second tap ~3/8 turn later (quadrature-ish, avoids unison wobble)
    phase += phaseInc;
    if (phase >= 1.0f) phase -= 2.0f;
    const float m1 = PolySin(phase);
    float p2 = phase + 0.7639f;
    if (p2 >= 1.0f) p2 -= 2.0f;
    const float m2 = PolySin(p2);

    // input envelope follower (drives inverse-gravity ducking)
    const float mag = 0.5f * (fabsf(inL) + fabsf(inR));
    env += (mag > env ? envAtk : envRel) * (mag - env);

    // ---- input path ----
    float x = 0.5f * (inL + inR);
    pre.Write(x);
    if (preCur > 1.0f) x = pre.Read(preCur); // interp: predelay slews

    const float sc = scaleCur;
    x = RunAPN(inAP[0], x, 0.750f, Len(inApBase[0], sc));
    x = RunAPN(inAP[1], x, 0.750f, Len(inApBase[1], sc));
    x = RunAPN(inAP[2], x, 0.625f, Len(inApBase[2], sc));
    x = RunAPN(inAP[3], x, 0.625f, Len(inApBase[3], sc));

    // ---- figure-eight tank ----
    const float fbA = dB2.ReadN(Len(dB2Base, sc)) * decayCur; // B feeds A
    const float fbB = dA2.ReadN(Len(dA2Base, sc)) * decayCur; // A feeds B
    const float dCoef = 1.0f - dampCur;

    // branch A (modulated allpass keeps interpolated read)
    float a = x + fbA;
    a = RunAP(apA1, a, -0.70f, apA1Base * sc + excCur * m1);
    dA1.Write(Sat(a));
    a = dA1.ReadN(Len(dA1Base, sc));
    lpA += dCoef * (a - lpA); a = lpA;            // hi damping
    loA += loCoef * (a - loA); a -= lowCur * loA; // lo cut
    a *= decayCur;
    a = RunAPN(apA2, a, 0.50f, Len(apA2Base, sc));
    dA2.Write(Sat(a));

    // branch B
    float b = x + fbB;
    b = RunAP(apB1, b, -0.70f, apB1Base * sc + excCur * m2);
    dB1.Write(Sat(b));
    b = dB1.ReadN(Len(dB1Base, sc));
    lpB += dCoef * (b - lpB); b = lpB;
    loB += loCoef * (b - loB); b -= lowCur * loB;
    b *= decayCur;
    b = RunAPN(apB2, b, 0.50f, Len(apB2Base, sc));
    dB2.Write(Sat(b));

    // ---- output taps (Dattorro tap table, scaled with SIZE) ----
    // nearest-sample reads: a tap stepping by one sample during a SIZE
    // sweep is inaudible inside this much diffusion
    float yL = dB1.ReadN(Len(tapL[0], sc)) + dB1.ReadN(Len(tapL[1], sc))
             - apB2.ReadN(Len(tapL[2], sc)) + dB2.ReadN(Len(tapL[3], sc))
             - dA1.ReadN(Len(tapL[4], sc)) - apA2.ReadN(Len(tapL[5], sc))
             - dA2.ReadN(Len(tapL[6], sc));
    float yR = dA1.ReadN(Len(tapR[0], sc)) + dA1.ReadN(Len(tapR[1], sc))
             - apA2.ReadN(Len(tapR[2], sc)) + dA2.ReadN(Len(tapR[3], sc))
             - dB1.ReadN(Len(tapR[4], sc)) - apB2.ReadN(Len(tapR[5], sc))
             - dB2.ReadN(Len(tapR[6], sc));
    yL *= 0.6f;
    yR *= 0.6f;

    // inverse gravity: duck the wet with the input envelope so the tail
    // swells up after the note instead of speaking immediately
    float wetg = 1.0f - invCur * Minf(1.0f, env * 4.0f);
    wetg *= wetg;
    yL *= wetg;
    yR *= wetg;

    if (monoSum) {
      const float m = 0.7071f * (yL + yR);
      outL = m; outR = m;
    } else {
      outL = yL; outR = yR;
    }
  }

private:
  // ---- Dattorro reference lengths (samples at 29761 Hz) ----
  static constexpr float kInAP[4] = {142, 107, 379, 277};
  static constexpr float kApA1 = 672, kDA1 = 4453, kApA2 = 1800, kDA2 = 3720;
  static constexpr float kApB1 = 908, kDB1 = 4217, kApB2 = 2656, kDB2 = 3163;
  // output taps: {dB1, dB1, apB2, dB2, dA1, apA2, dA2}
  static constexpr float kTapL[7] = {266, 2974, 1913, 1996, 1990, 187, 1066};
  // output taps: {dA1, dA1, apA2, dA2, dB1, apB2, dB2}
  static constexpr float kTapR[7] = {353, 3627, 1228, 2673, 2111, 335, 121};

  struct Dims {
    int32_t cap_pre, cap_dA1, cap_dA2, cap_dB1, cap_dB2;
    int32_t cap_inAP[4], cap_apA1, cap_apA2, cap_apB1, cap_apB2;
    size_t total_bytes;
  };

  static size_t Align4(size_t n) { return (n + 3u) & ~size_t(3); }
  static float Clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
  }
  static float Minf(float a, float b) { return a < b ? a : b; }
  float OnePoleCoef(float tau_sec) const {
    return 1.0f - expf(-1.0f / (tau_sec * sr));
  }
  static inline int32_t Len(float base, float sc) {
    return static_cast<int32_t>(base * sc + 0.5f);
  }

  // Bhaskara-style parabolic sine: phase in [-1,1) is a full turn.
  // Max error ~0.1% after the refinement pass; plenty for a chorus LFO.
  static inline float PolySin(float p) {
    const float y = 4.0f * p - 4.0f * p * fabsf(p); // sin(pi*p) approx
    return 0.775f * y + 0.225f * y * fabsf(y);
  }

  static void ComputeDims(float sample_rate, Dims& d) {
    const float ratio = sample_rate / kRefRate;
    const int32_t guard = static_cast<int32_t>(0.002f * sample_rate) + 8;
    auto cap = [&](float ref, int32_t extra) {
      return static_cast<int32_t>(ref * ratio * kMaxScale) + extra + 8;
    };
    d.cap_pre = static_cast<int32_t>(kMaxPredelaySec * sample_rate) + 8;
    d.cap_dA1 = cap(kDA1, 0); d.cap_dA2 = cap(kDA2, 0);
    d.cap_dB1 = cap(kDB1, 0); d.cap_dB2 = cap(kDB2, 0);
    for (int i = 0; i < 4; ++i) d.cap_inAP[i] = cap(kInAP[i], 0);
    d.cap_apA1 = cap(kApA1, guard); d.cap_apA2 = cap(kApA2, 0);
    d.cap_apB1 = cap(kApB1, guard); d.cap_apB2 = cap(kApB2, 0);

    size_t bytes = 0;
    bytes += Align4(d.cap_pre * sizeof(int16_t));
    bytes += Align4(d.cap_dA1 * sizeof(int16_t));
    bytes += Align4(d.cap_dA2 * sizeof(int16_t));
    bytes += Align4(d.cap_dB1 * sizeof(int16_t));
    bytes += Align4(d.cap_dB2 * sizeof(int16_t));
    for (int i = 0; i < 4; ++i) bytes += Align4(d.cap_inAP[i] * sizeof(int16_t));
    bytes += Align4(d.cap_apA1 * sizeof(int16_t));
    bytes += Align4(d.cap_apA2 * sizeof(int16_t));
    bytes += Align4(d.cap_apB1 * sizeof(int16_t));
    bytes += Align4(d.cap_apB2 * sizeof(int16_t));
    d.total_bytes = bytes;
  }

  // ---- delay line: int16 storage, per-line fixed-point scale ----
  struct Line {
    int16_t* buf = nullptr;
    int32_t cap = 0, wr = 0;
    float scale = 16384.0f, inv = 1.0f / 16384.0f, lim = 32767.0f;

    inline void Write(float x) {
      x *= scale;
      if (x > lim) x = lim;
      else if (x < -lim) x = -lim;
      buf[wr] = static_cast<int16_t>(x);
      if (++wr >= cap) wr = 0;
    }
    // nearest-sample read (cheap path)
    inline float ReadN(int32_t d) const {
      if (d < 1) d = 1;
      else if (d > cap - 1) d = cap - 1;
      int32_t rp = wr - d;
      if (rp < 0) rp += cap;
      return buf[rp] * inv;
    }
    // linear-interpolated read (modulated allpasses, predelay)
    inline float Read(float d) const {
      if (d < 1.0f) d = 1.0f;
      const float dmax = static_cast<float>(cap - 2);
      if (d > dmax) d = dmax;
      float rp = static_cast<float>(wr) - d;
      if (rp < 0.0f) rp += static_cast<float>(cap);
      int32_t i0 = static_cast<int32_t>(rp);
      const float fr = rp - static_cast<float>(i0);
      int32_t i1 = i0 + 1;
      if (i1 >= cap) i1 = 0;
      const float s0 = buf[i0], s1 = buf[i1];
      return (s0 + fr * (s1 - s0)) * inv;
    }
  };

  static void Zero(Line& l) {
    if (l.buf) memset(l.buf, 0, l.cap * sizeof(int16_t));
    l.wr = 0;
  }

  // allpass, nearest-sample read
  static inline float RunAPN(Line& l, float x, float g, int32_t d) {
    const float v = l.ReadN(d);
    const float w = x - g * v;
    l.Write(w);
    return v + g * w;
  }
  // allpass, interpolated read (for modulated delays)
  static inline float RunAP(Line& l, float x, float g, float d) {
    const float v = l.Read(d);
    const float w = x - g * v;
    l.Write(w);
    return v + g * w;
  }

  // division-free cubic soft clip: monotonic on [-2,2], bounded at 4/3
  static inline float Sat(float x) {
    if (x > 2.0f) x = 2.0f;
    else if (x < -2.0f) x = -2.0f;
    return x - x * x * x * (1.0f / 12.0f);
  }

  // ---- state ----
  bool ready_ = false;
  bool monoSum = false;
  float sr = 0.0f;

  Line pre, dA1, dA2, dB1, dB2;
  Line inAP[4], apA1, apA2, apB1, apB2;

  float inApBase[4] = {0};
  float apA1Base = 0, dA1Base = 0, apA2Base = 0, dA2Base = 0;
  float apB1Base = 0, dB1Base = 0, apB2Base = 0, dB2Base = 0;
  float tapL[7] = {0}, tapR[7] = {0};

  // smoothed parameters (targets are written by setters)
  float scaleTgt = 1.5f, scaleCur = 1.5f;
  float decayTgt = 0.80f, decayCur = 0.80f;
  float dampTgt = 0.25f, dampCur = 0.25f;
  float lowTgt = 0.10f, lowCur = 0.10f;
  float excTgt = 0.0f, excCur = 0.0f;
  float invTgt = 0.0f, invCur = 0.0f;
  float preTgt = 0.0f, preCur = 0.0f;
  float sizeNorm = 0.5f;
  float rateHz = 0.5f;

  float fastSlew = 0.01f, scaleSlew = 0.0001f, preSlew = 0.0005f;
  float envAtk = 0.01f, envRel = 0.0001f, loCoef = 0.02f;
  float phase = 0.0f, phaseInc = 0.0f;
  float env = 0.0f;
  float lpA = 0.0f, lpB = 0.0f, loA = 0.0f, loB = 0.0f;
};
