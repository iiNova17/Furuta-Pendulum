#pragma once
#include <stdint.h>
#include <math.h>

// Inputs are 12-bit angle values. Exact half-turn maps to -180 degrees.
inline int wrappedDelta(int value, int reference) {
  int delta = value - reference;
  if (delta > 2047) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return delta;
}

struct AnalogWindow {
  uint32_t n = 0;
  double mean = 0;
  double m2 = 0;
  uint32_t minMv = UINT32_MAX;
  uint32_t maxMv = 0;
  uint16_t minRaw = 4095;
  uint16_t maxRaw = 0;

  void add(uint32_t mv, uint16_t raw) {
    ++n;
    const double d = mv - mean;
    mean += d / n;
    m2 += d * (mv - mean);
    if (mv < minMv) minMv = mv;
    if (mv > maxMv) maxMv = mv;
    if (raw < minRaw) minRaw = raw;
    if (raw > maxRaw) maxRaw = raw;
  }

  double sigma() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0; }
  bool plausibleZero() const {
    // Bench heuristics, not a sensor specification or proof of connection.
    return n >= 100 && mean >= 2200 && mean <= 2800 &&
           minRaw > 16 && maxRaw < 4079 &&
           sigma() <= 30 && maxMv - minMv <= 200;
  }
};
