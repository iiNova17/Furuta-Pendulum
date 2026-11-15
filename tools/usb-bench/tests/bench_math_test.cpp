#include "../include/bench_math.h"
#include <assert.h>
#include <stdio.h>

int main() {
  // Wrap crossings and the signed half-turn boundary.
  assert(wrappedDelta(0, 4095) == 1);
  assert(wrappedDelta(4095, 0) == -1);
  assert(wrappedDelta(5, 4090) == 11);
  assert(wrappedDelta(4090, 5) == -11);
  assert(wrappedDelta(2048, 0) == -2048);
  assert(wrappedDelta(0, 2048) == -2048);
  for (int ref = 0; ref < 4096; ++ref) {
    assert(wrappedDelta(ref, ref) == 0);
    assert(wrappedDelta((ref + 1) % 4096, ref) == 1);
    assert(wrappedDelta((ref + 4095) % 4096, ref) == -1);
  }

  AnalogWindow good;
  assert(!good.plausibleZero());
  for (int i = 0; i < 99; ++i) good.add(2500, 3200);
  assert(!good.plausibleZero());  // Not enough samples.
  good.add(2500, 3200);
  assert(good.plausibleZero());
  assert(good.mean == 2500 && good.sigma() == 0);

  AnalogWindow stats;
  stats.add(2490, 3190);
  stats.add(2510, 3210);
  assert(stats.mean == 2500);
  assert(fabs(stats.sigma() - sqrt(200.0)) < 1e-9);
  assert(stats.minMv == 2490 && stats.maxMv == 2510);

  auto clippedLow = good;
  clippedLow.add(2500, 0);
  assert(!clippedLow.plausibleZero());
  auto clippedHigh = good;
  clippedHigh.add(2500, 4095);
  assert(!clippedHigh.plausibleZero());
  auto spike = good;
  spike.add(2800, 3500);
  assert(!spike.plausibleZero());  // Mean still plausible, peak-to-peak too large.
  AnalogWindow noisy, unpowered, highOffset;
  for (int i = 0; i < 100; ++i) {
    noisy.add(i % 2 ? 2450 : 2550, 3200);
    unpowered.add(150, 180);
    highOffset.add(2900, 3700);
  }
  assert(!noisy.plausibleZero());
  assert(!unpowered.plausibleZero());
  assert(!highOffset.plausibleZero());
  puts("PASS: angle wrapping, sample statistics, and calibration rejection cases.");
}
