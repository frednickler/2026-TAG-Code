#pragma once
#include <stdint.h>
#include <time.h>

// WMM2020 magnetic field model (valid 2020-2025)
// Returns magnetic declination in degrees (positive = East of True North)
// Accuracy: ~0.1° (much better than simple dipole model)
float computeDeclination(double latDeg, double lonDeg, double altMeters,
                        time_t timestamp = 0);

// Get last computed declination (avoids recomputing if position hasn't changed)
float getLastDeclination();

// Invalidate the declination cache (force recompute on next call)
void invalidateDeclinationCache();
