#pragma once

#include "antenna_pattern.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace sensor_mapper
{

// Advanced antenna pattern library with real-world models
class antenna_pattern_library_advanced_t
{
public:
  // ========== Cellular Base Station Antennas ==========

  // Typical 3-sector cellular antenna (120° beamwidth)
  static auto generate_cellular_sector_antenna(float frequency_mhz = 2100.0f, float electrical_tilt_deg = 0.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Cellular Sector 120°";
    pattern->manufacturer = "Generic";
    pattern->frequency_mhz = frequency_mhz;
    pattern->horizontal_beamwidth_deg = 65.0f; // Typical 3dB beamwidth
    pattern->vertical_beamwidth_deg = 7.0f;    // Narrow vertical beam
    pattern->max_gain_dbi = 17.5f;             // Typical sector gain
    pattern->electrical_tilt_deg = electrical_tilt_deg;
    pattern->polarization = polarization_t::DUAL_SLANT;

    // Create 3D pattern
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 2)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->azimuth_angles.clear();
    for (int az = 0; az < 360; ++az)
    {
      pattern->azimuth_angles.push_back(static_cast<float>(az));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx] - electrical_tilt_deg;
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;

        // Horizontal pattern (65° 3dB beamwidth)
        float h_gain = -12.0f * std::pow(azi_angle / 65.0f, 2.0f);
        h_gain = std::max(h_gain, -30.0f);

        // Vertical pattern (7° 3dB beamwidth)
        float v_gain = -12.0f * std::pow(elev / 7.0f, 2.0f);
        v_gain = std::max(v_gain, -25.0f);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + h_gain + v_gain;
      }
    }

    return pattern;
  }

  // Small cell / Femtocell omnidirectional antenna
  static auto generate_small_cell_omni(float frequency_mhz = 2400.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Small Cell Omni";
    pattern->frequency_mhz = frequency_mhz;
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->vertical_beamwidth_deg = 60.0f;
    pattern->max_gain_dbi = 5.0f;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 5)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      // Omnidirectional in azimuth, shaped in elevation
      float v_pattern = std::cos(elev * M_PI / 180.0f);
      float gain = pattern->max_gain_dbi + 10.0f * std::log10(v_pattern * v_pattern + 0.001f);

      for (int azi = 0; azi < 360; ++azi)
      {
        pattern->gain_db[elev_idx][azi] = gain;
      }
    }

    return pattern;
  }

  // Massive MIMO antenna array (5G)
  static auto generate_massive_mimo_array(int num_elements_h = 8, int num_elements_v = 4, float frequency_mhz = 3500.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Massive MIMO " + std::to_string(num_elements_h) + "x" + std::to_string(num_elements_v);
    pattern->manufacturer = "Generic";
    pattern->frequency_mhz = frequency_mhz;

    // Array gain calculation
    pattern->max_gain_dbi = 10.0f * std::log10(static_cast<float>(num_elements_h * num_elements_v)) + 5.0f;

    // Beamwidth inversely proportional to array size
    pattern->horizontal_beamwidth_deg = 65.0f / std::sqrt(static_cast<float>(num_elements_h));
    pattern->vertical_beamwidth_deg = 15.0f / std::sqrt(static_cast<float>(num_elements_v));

    pattern->polarization = polarization_t::DUAL_SLANT;

    // Create 3D pattern
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 2)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;

        // Array factor pattern with narrow beams and side lobes
        float h_pattern = calculate_array_factor(azi_angle, num_elements_h, pattern->horizontal_beamwidth_deg);
        float v_pattern = calculate_array_factor(elev, num_elements_v, pattern->vertical_beamwidth_deg);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + h_pattern + v_pattern;
      }
    }

    return pattern;
  }

  // ========== WiFi / WLAN Antennas ==========

  // WiFi 2.4 GHz omnidirectional dipole
  static auto generate_wifi_dipole() -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "WiFi Dipole 2.4GHz";
    pattern->frequency_mhz = 2400.0f;
    pattern->max_gain_dbi = 2.15f;
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->vertical_beamwidth_deg = 78.0f;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 5)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];

      // Classic dipole pattern (donut shape)
      float elev_factor = std::abs(std::cos(elev * M_PI / 180.0f));
      float gain = pattern->max_gain_dbi + 10.0f * std::log10(elev_factor + 0.001f);

      pattern->gain_db[elev_idx].resize(360, gain);
    }

    return pattern;
  }

  // WiFi patch antenna (directional)
  static auto generate_wifi_patch_panel(float beamwidth_deg = 80.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "WiFi Patch Panel " + std::to_string(static_cast<int>(beamwidth_deg)) + "°";
    pattern->frequency_mhz = 5800.0f; // 5 GHz band
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = 8.0f;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 3)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;

        float h_gain = -12.0f * std::pow(azi_angle / beamwidth_deg, 2.0f);
        float v_gain = -12.0f * std::pow(elev / beamwidth_deg, 2.0f);

        h_gain = std::max(h_gain, -35.0f);
        v_gain = std::max(v_gain, -35.0f);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + h_gain + v_gain;
      }
    }

    return pattern;
  }

  // ========== ISM Band / LoRa / IoT Antennas ==========

  // LoRa gateway antenna (omnidirectional collinear)
  static auto generate_lora_gateway_collinear(int num_sections = 4) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "LoRa Gateway Collinear " + std::to_string(num_sections) + "-section";
    pattern->frequency_mhz = 915.0f; // US ISM band
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->max_gain_dbi = 2.15f + 2.0f * num_sections; // ~2dB per section
    pattern->vertical_beamwidth_deg = 60.0f / num_sections;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 3)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];

      // Compressed elevation pattern due to collinear sections
      float v_beamwidth = pattern->vertical_beamwidth_deg;
      float v_gain = -12.0f * std::pow(elev / v_beamwidth, 2.0f);
      v_gain = std::max(v_gain, -40.0f);

      float gain = pattern->max_gain_dbi + v_gain;
      pattern->gain_db[elev_idx].resize(360, gain);
    }

    return pattern;
  }

  // IoT device antenna (small omnidirectional)
  static auto generate_iot_chip_antenna() -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "IoT Chip Antenna";
    pattern->frequency_mhz = 2400.0f;
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->vertical_beamwidth_deg = 180.0f;
    pattern->max_gain_dbi = -2.0f; // Negative gain typical for chip antennas
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    // Somewhat omnidirectional but with nulls
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 10)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      // Irregular pattern typical of chip antennas
      float base_gain = pattern->max_gain_dbi;

      for (int azi = 0; azi < 360; ++azi)
      {
        // Add some variation
        float variation = 3.0f * std::sin(azi * M_PI / 180.0f) * std::cos(elev * M_PI / 180.0f);
        pattern->gain_db[elev_idx][azi] = base_gain + variation;
      }
    }

    return pattern;
  }

  // ========== Satellite Communication Antennas ==========

  // GNSS/GPS antenna (RHCP)
  static auto generate_gnss_antenna() -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "GNSS/GPS Antenna";
    pattern->frequency_mhz = 1575.42f; // L1 frequency
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->vertical_beamwidth_deg = 140.0f;
    pattern->max_gain_dbi = 3.0f;
    pattern->polarization = polarization_t::CIRCULAR_RIGHT;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 5)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];

      // Upper hemisphere focused (sky-facing)
      float gain;
      if (elev > 0.0f)
      {
        // Gain increases toward zenith
        gain = pattern->max_gain_dbi * std::sin(elev * M_PI / 180.0f);
      }
      else
      {
        // Reduced gain below horizon
        gain = pattern->max_gain_dbi * std::sin(elev * M_PI / 180.0f) - 10.0f;
      }
      gain = std::max(gain, -30.0f);

      pattern->gain_db[elev_idx].resize(360, gain);
    }

    return pattern;
  }

  // Satellite dish (parabolic reflector with feed)
  static auto generate_satellite_dish(float diameter_m, float frequency_ghz = 12.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    // Calculate parameters
    float wavelength_m = 0.3f / frequency_ghz;
    float efficiency = 0.65f; // Typical efficiency
    float gain_dbi = 10.0f * std::log10(efficiency * std::pow(M_PI * diameter_m / wavelength_m, 2.0f));
    float beamwidth_deg = 70.0f * wavelength_m / diameter_m;

    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Satellite Dish " + std::to_string(diameter_m) + "m";
    pattern->frequency_mhz = frequency_ghz * 1000.0f;
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = gain_dbi;
    pattern->front_to_back_ratio_db = 60.0f;
    pattern->polarization = polarization_t::CIRCULAR_RIGHT;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 1)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;

        // Parabolic dish pattern
        float angle_from_boresight = std::sqrt(azi_angle * azi_angle + elev * elev);
        float normalized_angle = angle_from_boresight / beamwidth_deg;

        float gain = -12.0f * normalized_angle * normalized_angle;
        gain = std::max(gain, -60.0f);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + gain;
      }
    }

    return pattern;
  }

  // ========== Point-to-Point Link Antennas ==========

  // Microwave backhaul antenna
  static auto generate_microwave_backhaul(float diameter_m = 0.6f, float frequency_ghz = 23.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    float wavelength_m = 0.3f / frequency_ghz;
    float efficiency = 0.7f;
    float gain_dbi = 10.0f * std::log10(efficiency * std::pow(M_PI * diameter_m / wavelength_m, 2.0f));
    float beamwidth_deg = 65.0f * wavelength_m / diameter_m;

    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Microwave Backhaul " + std::to_string(static_cast<int>(frequency_ghz)) + " GHz";
    pattern->frequency_mhz = frequency_ghz * 1000.0f;
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = gain_dbi;
    pattern->front_to_back_ratio_db = 70.0f;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;

    pattern->elevation_angles.clear();
    for (int el = -45; el <= 45; el += 1)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;

        // Very tight beam with low side lobes
        float h_gain = -15.0f * std::pow(azi_angle / beamwidth_deg, 2.0f);
        float v_gain = -15.0f * std::pow(elev / beamwidth_deg, 2.0f);

        h_gain = std::max(h_gain, -70.0f);
        v_gain = std::max(v_gain, -70.0f);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + h_gain + v_gain;
      }
    }

    return pattern;
  }

  // ========== Radar Antennas ==========

  // Phased array radar
  static auto generate_phased_array_radar(int num_elements = 64, float frequency_ghz = 10.0f, float scan_angle_deg = 0.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Phased Array Radar " + std::to_string(num_elements) + " elements";
    pattern->frequency_mhz = frequency_ghz * 1000.0f;
    pattern->max_gain_dbi = 10.0f * std::log10(static_cast<float>(num_elements)) + 8.0f;
    pattern->horizontal_beamwidth_deg = 90.0f / std::sqrt(static_cast<float>(num_elements));
    pattern->vertical_beamwidth_deg = pattern->horizontal_beamwidth_deg;
    pattern->polarization = polarization_t::LINEAR_HORIZONTAL;

    pattern->elevation_angles.clear();
    for (int el = -60; el <= 60; el += 2)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi) - scan_angle_deg;
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;
        if (azi_angle < -180.0f)
          azi_angle += 360.0f;

        // Phased array pattern with grating lobes
        float h_pattern = calculate_phased_array_pattern(azi_angle, num_elements, pattern->horizontal_beamwidth_deg);
        float v_pattern = calculate_phased_array_pattern(elev, num_elements, pattern->vertical_beamwidth_deg);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + h_pattern + v_pattern;
      }
    }

    return pattern;
  }

  // ========== Beamforming and MIMO Patterns ==========

  // Adaptive beamformed pattern (steered toward specific direction)
  static auto generate_beamformed_pattern(float beam_azimuth_deg, float beam_elevation_deg, float beamwidth_deg = 15.0f, int num_elements = 16) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Beamformed @ " + std::to_string(static_cast<int>(beam_azimuth_deg)) + "°";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = 10.0f * std::log10(static_cast<float>(num_elements)) + 5.0f;

    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 3)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float azi_angle = static_cast<float>(azi) - beam_azimuth_deg;
        if (azi_angle > 180.0f)
          azi_angle -= 360.0f;
        if (azi_angle < -180.0f)
          azi_angle += 360.0f;

        float elev_angle = elev - beam_elevation_deg;

        // Gaussian beam pattern
        float angle_from_beam = std::sqrt(azi_angle * azi_angle + elev_angle * elev_angle);
        float gain = -12.0f * std::pow(angle_from_beam / beamwidth_deg, 2.0f);
        gain = std::max(gain, -40.0f);

        pattern->gain_db[elev_idx][azi] = pattern->max_gain_dbi + gain;
      }
    }

    return pattern;
  }

  // Multi-beam MIMO pattern (multiple simultaneous beams)
  static auto generate_multibeam_mimo(const std::vector<float> &beam_azimuths, float beamwidth_deg = 20.0f) -> std::shared_ptr<antenna_pattern_t>
  {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Multi-beam MIMO " + std::to_string(beam_azimuths.size()) + " beams";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = 18.0f;

    pattern->elevation_angles.clear();
    for (int el = -60; el <= 60; el += 3)
    {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }

    pattern->gain_db.resize(pattern->elevation_angles.size());

    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx)
    {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);

      for (int azi = 0; azi < 360; ++azi)
      {
        float max_gain_at_angle = -100.0f;

        // Find maximum gain from any beam at this angle
        for (float beam_az : beam_azimuths)
        {
          float angle_diff = static_cast<float>(azi) - beam_az;
          if (angle_diff > 180.0f)
            angle_diff -= 360.0f;
          if (angle_diff < -180.0f)
            angle_diff += 360.0f;

          float h_gain = -12.0f * std::pow(angle_diff / beamwidth_deg, 2.0f);
          float v_gain = -12.0f * std::pow(elev / beamwidth_deg, 2.0f);

          float gain = pattern->max_gain_dbi + h_gain + v_gain;
          max_gain_at_angle = std::max(max_gain_at_angle, gain);
        }

        pattern->gain_db[elev_idx][azi] = std::max(max_gain_at_angle, -40.0f);
      }
    }

    return pattern;
  }

private:
  // Helper: Calculate array factor for linear array
  static auto calculate_array_factor(float angle_deg, int num_elements, float /* beamwidth_deg */) -> float
  {
    if (num_elements <= 1)
      return 0.0f;

    // Array factor calculation
    float angle_rad = angle_deg * M_PI / 180.0f;
    float d_lambda = 0.5f; // Element spacing in wavelengths
    float psi = 2.0f * M_PI * d_lambda * std::sin(angle_rad);

    // Array factor
    float af;
    if (std::abs(psi) < 0.001f)
    {
      af = static_cast<float>(num_elements);
    }
    else
    {
      af = std::sin(num_elements * psi / 2.0f) / std::sin(psi / 2.0f);
    }

    float af_db = 20.0f * std::log10(std::abs(af) / num_elements + 0.0001f);
    return std::max(af_db, -40.0f);
  }

  // Helper: Calculate phased array pattern with grating lobes
  static auto calculate_phased_array_pattern(float angle_deg, int num_elements, float beamwidth_deg) -> float
  {
    // Main beam
    float main_beam = -12.0f * std::pow(angle_deg / beamwidth_deg, 2.0f);

    // Grating lobes (appear at large scan angles)
    float grating_lobe_level = -20.0f;
    float grating_lobe_spacing = 60.0f / std::sqrt(static_cast<float>(num_elements));

    float pattern = main_beam;

    // Add grating lobes
    for (int i = 1; i <= 3; ++i)
    {
      float lobe_angle = i * grating_lobe_spacing;
      float dist_from_lobe_1 = std::abs(std::abs(angle_deg) - lobe_angle);
      float dist_from_lobe_2 = std::abs(std::abs(angle_deg) + lobe_angle);

      float lobe_contrib_1 = grating_lobe_level - 3.0f * std::pow(dist_from_lobe_1 / 10.0f, 2.0f);
      float lobe_contrib_2 = grating_lobe_level - 3.0f * std::pow(dist_from_lobe_2 / 10.0f, 2.0f);

      // Combine in linear domain then convert back
      float pattern_linear = std::pow(10.0f, pattern / 10.0f);
      float lobe_1_linear = std::pow(10.0f, lobe_contrib_1 / 10.0f);
      float lobe_2_linear = std::pow(10.0f, lobe_contrib_2 / 10.0f);

      pattern = 10.0f * std::log10(pattern_linear + lobe_1_linear + lobe_2_linear + 0.0001f);
    }

    return std::max(pattern, -50.0f);
  }
};

} // namespace sensor_mapper