#pragma once
// Platform-independent DSP core for the "Animorf" audio applet — an emulation
// of the Moog MF-105M MIDI MuRF (Multiple Resonance Filter Array):
//
//   - 8 parallel resonant bandpass filters (Chamberlin state-variable),
//     switchable between the Mids and Bass frequency banks
//   - a pattern step-sequencer animates the bands: each step gates a set of
//     bands, and a per-band envelope (attack/release morphed by the ENV
//     control, negative values reverse into swells) shapes the transitions
//   - each band has its own level slider, like the MuRF's front panel
//   - RATE runs the animation free (0.1..20 steps/s) or steps advance from
//     an external clock (StepNow); internal stepping can be suppressed
//   - DRIVE adds input gain into a soft clipper for that transistor grit
//   - SWEEP shifts the whole filter array up/down (+-2 octaves), standing in
//     for the MuRF's expression-pedal frequency sweep
//   - stereo: odd bands lean left, even bands lean right, like the
//     hardware's stereo outputs; MONO applets sum the array
//
// State is tiny (no delay lines), so there is no arena to allocate.

#include <cstdint>
#include <cstddef>
#include <cmath>

class AnimorfCore {
public:
  static constexpr int kBands = 8;
  static constexpr int kPatterns = 16;

  bool Init(float sample_rate) {
    sr = sample_rate;
    for (int i = 0; i < kBands; ++i) {
      lo[i] = bp[i] = 0.0f;
      env[i] = 0.0f;
      lvl[i] = 1.0f;
    }
    // stereo placement: odd bands left, even bands right (hardware-style)
    for (int i = 0; i < kBands; ++i) {
      const bool left = (i & 1) == 0;
      panL[i] = left ? 0.80f : 0.20f;
      panR[i] = left ? 0.20f : 0.80f;
    }
    sweepCur = sweepTgt = 0.0f;
    driveCur = driveTgt = 0.0f;
    stepPhase = 0.0f;
    stepIdx = 0;
    lfsr = 0xACE1u;
    fastSlew = 1.0f - expf(-1.0f / (0.008f * sr));
    SetRate(rateHz);
    SetEnv(envKnob);
    UpdateCoeffs(true);
    ApplyStepMask();
    ready_ = true;
    return true;
  }

  void Release() { ready_ = false; }
  bool Ready() const { return ready_; }

  // ---- parameters (single-word stores; ISR-safe) ----

  void SetPattern(int p) {
    if (p < 0) p = 0;
    if (p >= kPatterns) p = kPatterns - 1;
    if (p != pattern) {
      pattern = p;
      stepIdx = 0;
      ApplyStepMask();
    }
  }

  // steps per second for the internal animation clock
  void SetRate(float hz) {
    rateHz = Clampf(hz, 0.05f, 30.0f);
    if (sr > 0.0f) stepInc = rateHz / sr;
  }

  // e in [-1, 1]: magnitude sets envelope speed (0 = staccato, 1 = long
  // release); negative swaps attack/release for MuRF-style reverse swells
  void SetEnv(float e) {
    envKnob = Clampf(e, -1.0f, 1.0f);
    const float m = fabsf(envKnob);
    float attSec = 0.001f + 0.20f * m * m;
    float relSec = 0.020f + 0.60f * m;
    if (envKnob < 0.0f) {
      const float t = attSec; // reverse: slow swell in, quick out
      attSec = relSec;
      relSec = t * 0.5f + 0.01f;
    }
    if (sr > 0.0f) {
      attCoef = 1.0f - expf(-1.0f / (attSec * sr));
      relCoef = 1.0f - expf(-1.0f / (relSec * sr));
    }
  }

  // d in [0, 1]: input gain 1..8x into a soft clipper, level-compensated
  void SetDrive(float d) { driveTgt = Clampf(d, 0.0f, 1.0f); }

  // s in [-1, 1] -> +-2 octaves of array frequency shift
  void SetSweep(float s) { sweepTgt = Clampf(s, -1.0f, 1.0f); }

  // true = Bass bank (MF-105B voicing), false = Mids bank (MF-105)
  void SetBassMode(bool b) {
    if (b != bassMode) {
      bassMode = b;
      UpdateCoeffs(true);
    }
  }

  // band i level in [0, 1] (the 8 front-panel sliders)
  void SetBandLevel(int i, float v) {
    if (i < 0 || i >= kBands) return;
    lvl[i] = Clampf(v, 0.0f, 1.0f);
  }

  // false freezes the internal clock (use with external StepNow clocking)
  void SetInternalStepping(bool on) { internalStep = on; }

  void SetMonoSum(bool b) { monoSum = b; }

  // advance one pattern step right now (external clock / tap)
  void StepNow() { stepFlag = true; }

  // ---- UI feedback ----
  float GetBandEnv(int i) const {
    return (i >= 0 && i < kBands) ? env[i] * lvl[i] : 0.0f;
  }
  int GetStep() const { return stepIdx; }

  // ---- audio ----
  inline void Process(float inL, float inR, float& outL, float& outR) {
    if (!ready_) { outL = 0.0f; outR = 0.0f; return; }

    // animation clock
    if (stepFlag) {
      stepFlag = false;
      stepPhase = 0.0f;
      Advance();
    } else if (internalStep) {
      stepPhase += stepInc;
      if (stepPhase >= 1.0f) {
        stepPhase -= 1.0f;
        Advance();
      }
    }

    // smoothed globals
    driveCur += fastSlew * (driveTgt - driveCur);
    if (fabsf(sweepTgt - sweepCur) > 1e-5f) {
      sweepCur += fastSlew * (sweepTgt - sweepCur);
      UpdateCoeffs(false);
    }

    // drive -> soft clip (keeps the array fed with harmonics, MuRF-style)
    float x = 0.5f * (inL + inR);
    x *= 1.0f + 7.0f * driveCur;
    x = SoftClip(x);
    x *= 1.0f / (1.0f + 2.2f * driveCur); // level compensation

    // 8 resonant bandpass filters with animated per-band envelopes
    float yL = 0.0f, yR = 0.0f;
    for (int i = 0; i < kBands; ++i) {
      // Chamberlin SVF (bandpass output)
      lo[i] += f[i] * bp[i];
      const float hi = x - lo[i] - kQ * bp[i];
      bp[i] += f[i] * hi;

      // envelope toward gated slider level
      const float target = (mask >> i) & 1 ? lvl[i] : 0.0f;
      env[i] += (target > env[i] ? attCoef : relCoef) * (target - env[i]);

      const float y = bp[i] * env[i];
      yL += y * panL[i];
      yR += y * panR[i];
    }
    yL *= kMakeup;
    yR *= kMakeup;

    if (monoSum) {
      const float m = 0.7071f * (yL + yR);
      outL = m; outR = m;
    } else {
      outL = yL; outR = yR;
    }
  }

private:
  // ---- filter banks (Hz, approximations of the MuRF voicings) ----
  static constexpr float kMids[kBands] = {200, 300, 450, 675, 1000, 1500,
                                          2200, 3400};
  static constexpr float kBass[kBands] = {110, 160, 240, 350, 520, 760,
                                          1100, 1600};
  static constexpr float kQ = 0.12f;      // damping = 1/Q, Q ~ 8
  static constexpr float kMakeup = 0.30f; // resonant gain compensation

  // ---- animation patterns (8-bit band masks per step) ----
  struct Pat {
    uint8_t len; // 0 = random (LFSR mask per step)
    uint8_t step[16];
  };
  static constexpr Pat kPat[kPatterns] = {
    {1, {0xFF}},                                                 // 0 static
    {8, {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80}},       // 1 up
    {8, {0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x02, 0x01}},       // 2 down
    {14, {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80,
          0x40, 0x20, 0x10, 0x08, 0x04, 0x02}},                  // 3 up-down
    {4, {0x03, 0x0C, 0x30, 0xC0}},                               // 4 pairs
    {2, {0x55, 0xAA}},                                           // 5 odd/even
    {4, {0x18, 0x24, 0x42, 0x81}},                               // 6 in->out
    {4, {0x81, 0x42, 0x24, 0x18}},                               // 7 out->in
    {8, {0x81, 0x42, 0x24, 0x18, 0x18, 0x24, 0x42, 0x81}},       // 8 bounce
    {4, {0x11, 0x22, 0x44, 0x88}},                               // 9 dual chase
    {16, {0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF,
          0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00}},      // 10 stairs
    {8, {0x01, 0x04, 0x10, 0x40, 0x02, 0x08, 0x20, 0x80}},       // 11 skip
    {6, {0x07, 0x38, 0xC0, 0x38, 0x07, 0xFF}},                   // 12 chords
    {2, {0xFF, 0x00}},                                           // 13 pulse
    {8, {0x09, 0x12, 0x24, 0x48, 0x90, 0x21, 0x42, 0x84}},       // 14 weave
    {0, {0}},                                                    // 15 random
  };

  static float Clampf(float x, float a, float b) {
    return x < a ? a : (x > b ? b : x);
  }

  // cubic soft clip, bounded, division-free
  static inline float SoftClip(float x) {
    if (x > 1.5f) x = 1.5f;
    else if (x < -1.5f) x = -1.5f;
    return x - x * x * x * (1.0f / 6.75f);
  }

  void Advance() {
    const Pat& p = kPat[pattern];
    if (p.len == 0) {
      // 16-bit Galois LFSR, one clock per step
      lfsr = (lfsr >> 1) ^ (uint16_t)(-(int16_t)(lfsr & 1u) & 0xB400u);
      stepIdx = 0;
      mask = (uint8_t)(lfsr & 0xFF);
      if (mask == 0) mask = 0x10;
      return;
    }
    if (++stepIdx >= p.len) stepIdx = 0;
    ApplyStepMask();
  }

  void ApplyStepMask() {
    const Pat& p = kPat[pattern];
    if (p.len > 0) mask = p.step[stepIdx < p.len ? stepIdx : 0];
  }

  // recompute SVF f coefficients; force also refreshes after mode change.
  // Linear approximation f = 2*pi*fc/sr (error <1% below sr/8).
  void UpdateCoeffs(bool force) {
    (void)force;
    const float* bank = bassMode ? kBass : kMids;
    const float factor = exp2f(2.0f * sweepCur); // +-2 octaves
    for (int i = 0; i < kBands; ++i) {
      float fc = bank[i] * factor;
      const float fmax = sr * 0.145f; // keep Chamberlin SVF well-behaved
      if (fc > fmax) fc = fmax;
      if (fc < 20.0f) fc = 20.0f;
      f[i] = 6.2831853f * fc / sr;
    }
  }

  // ---- state ----
  bool ready_ = false;
  bool monoSum = false;
  bool bassMode = false;
  bool internalStep = true;
  volatile bool stepFlag = false;
  float sr = 0.0f;

  float f[kBands] = {0};
  float lo[kBands] = {0}, bp[kBands] = {0};
  float env[kBands] = {0};
  float lvl[kBands] = {0};
  float panL[kBands] = {0}, panR[kBands] = {0};

  int pattern = 1;
  uint8_t mask = 0xFF;
  int stepIdx = 0;
  uint16_t lfsr = 0xACE1u;
  float stepPhase = 0.0f, stepInc = 0.0f;
  float rateHz = 2.0f;
  float envKnob = 0.3f;
  float attCoef = 0.01f, relCoef = 0.001f;
  float driveTgt = 0.0f, driveCur = 0.0f;
  float sweepTgt = 0.0f, sweepCur = 0.0f;
  float fastSlew = 0.01f;
};
