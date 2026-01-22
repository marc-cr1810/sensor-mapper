#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>


namespace sensor_mapper {

struct antenna_pattern_t {
  std::string name;
  // Normalized gain in dBi at every degree (0-359).
  // Array index = azimuth offset (0 = Boresight/Front).
  // Value = Gain relative to MAX gain (usually max is 0.0 or positive dBi).
  // Actually, usually patterns are normalized to 0dB max, and we add
  // antenna_gain_dbi on top. Let's store raw relative dB. 0.0 = peak. -20.0 =
  // 20dB down.
  std::vector<float> gain_db;

  antenna_pattern_t() {
    // Default to Omni
    gain_db.resize(360, 0.0f);
    name = "Omni";
  }
};

class antenna_library_t {
public:
  static auto generate_omni() -> std::shared_ptr<antenna_pattern_t> {
    return std::make_shared<antenna_pattern_t>();
  }

  static auto generate_sector(float beamwidth_deg, float fxb_ratio_db)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Sector " + std::to_string((int)beamwidth_deg) + "deg";
    pattern->gain_db.resize(360);

    for (int i = 0; i < 360; ++i) {
      // Angle from boresight (-180 to 180)
      float angle = (float)i;
      if (angle > 180)
        angle -= 360;

      // Simple Cosine power model approximation
      // G(theta) = -12 * (theta / BW)^2
      // Clamped to Back-to-Front ratio

      float norm_angle =
          std::abs(angle) / (beamwidth_deg / 2.0f); // 1.0 at -3dB edge?
      // Standard approx: -3dB at beamwidth/2
      // -3 = coeff * (1.0)^2  => coeff = -3.
      // Wait, standard formulas use 12dB for 2xBW?
      // Let's use 3dB beamwidth def: at angle = beamwidth/2, gain is -3dB.
      // dB = -3 * (angle / (bw/2))^2

      float gain = -3.0f * std::pow(angle / (beamwidth_deg / 2.0f), 2.0f);

      // Side lobe / Back lobe floor
      if (gain < -fxb_ratio_db)
        gain = -fxb_ratio_db;

      pattern->gain_db[i] = gain;
    }
    return pattern;
  }
};

} // namespace sensor_mapper
