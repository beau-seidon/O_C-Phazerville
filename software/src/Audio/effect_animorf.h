#pragma once
// Teensy Audio Library wrapper for AnimorfCore (see animorf_core.h).
// Stereo in, stereo out, wet-only (the applet handles dry/wet mixing).
// No delay memory: state is a few hundred bytes, no allocation needed.

#include <Arduino.h>
#include <AudioStream.h>
#include "animorf_core.h"

class AudioEffectAnimorf : public AudioStream {
public:
  AudioEffectAnimorf() : AudioStream(2, inputQueueArray) {}

  bool begin() {
    return core.Init(AUDIO_SAMPLE_RATE_EXACT);
  }
  void end() { core.Release(); }
  bool ready() const { return core.Ready(); }

  // parameter passthroughs (single-word stores; ISR-safe)
  void setPattern(int p) { core.SetPattern(p); }
  void setRate(float hz) { core.SetRate(hz); }
  void setEnv(float e) { core.SetEnv(e); }         // -1..1
  void setDrive(float d) { core.SetDrive(d); }     // 0..1
  void setSweep(float s) { core.SetSweep(s); }     // -1..1
  void setBassMode(bool b) { core.SetBassMode(b); }
  void setBandLevel(int i, float v) { core.SetBandLevel(i, v); }
  void setInternalStepping(bool on) { core.SetInternalStepping(on); }
  void setMonoSum(bool b) { core.SetMonoSum(b); }
  void stepNow() { core.StepNow(); }

  float bandEnv(int i) const { return core.GetBandEnv(i); }
  int step() const { return core.GetStep(); }

  virtual void update(void) override {
    audio_block_t* inL = receiveReadOnly(0);
    audio_block_t* inR = receiveReadOnly(1);
    if (!core.Ready()) {
      if (inL) release(inL);
      if (inR) release(inR);
      return;
    }
    audio_block_t* outL = allocate();
    audio_block_t* outR = allocate();
    if (!outL || !outR) {
      if (outL) release(outL);
      if (outR) release(outR);
      if (inL) release(inL);
      if (inR) release(inR);
      return;
    }

    const int16_t* pl = inL ? inL->data : nullptr;
    const int16_t* pr = inR ? inR->data : pl; // mono sources mirror ch0

    for (int n = 0; n < AUDIO_BLOCK_SAMPLES; ++n) {
      const float l = pl ? pl[n] * (1.0f / 32768.0f) : 0.0f;
      const float r = pr ? pr[n] * (1.0f / 32768.0f) : 0.0f;
      float wl, wr;
      core.Process(l, r, wl, wr);
      int32_t sl = static_cast<int32_t>(wl * 32767.0f);
      int32_t sr_ = static_cast<int32_t>(wr * 32767.0f);
      if (sl > 32767) sl = 32767; else if (sl < -32768) sl = -32768;
      if (sr_ > 32767) sr_ = 32767; else if (sr_ < -32768) sr_ = -32768;
      outL->data[n] = static_cast<int16_t>(sl);
      outR->data[n] = static_cast<int16_t>(sr_);
    }

    transmit(outL, 0);
    transmit(outR, 1);
    release(outL);
    release(outR);
    if (inL) release(inL);
    if (inR) release(inR);
  }

private:
  audio_block_t* inputQueueArray[2];
  AnimorfCore core;
};
