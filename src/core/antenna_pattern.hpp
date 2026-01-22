#pragma once

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <map>

namespace sensor_mapper {

// Polarization types
enum class polarization_t {
  LINEAR_VERTICAL,
  LINEAR_HORIZONTAL,
  CIRCULAR_LEFT,
  CIRCULAR_RIGHT,
  DUAL_SLANT,
  UNKNOWN
};

// Antenna pattern resolution
struct pattern_resolution_t {
  float azimuth_step_deg = 1.0f;   // Degrees between azimuth samples
  float elevation_step_deg = 1.0f; // Degrees between elevation samples
};

// 3D Antenna Pattern with full azimuth and elevation support
struct antenna_pattern_t {
  std::string name;
  std::string manufacturer;
  std::string model;
  float frequency_mhz = 0.0f;
  polarization_t polarization = polarization_t::LINEAR_VERTICAL;
  
  // Pattern resolution
  pattern_resolution_t resolution;
  
  // Gain pattern data
  // For 2D (azimuth-only): elevation_angles contains only one value (typically 0)
  // For 3D: full spherical pattern
  // gain_db[elevation_index][azimuth_index]
  std::vector<std::vector<float>> gain_db;
  
  // Azimuth angles (0-360 degrees, where 0 = boresight/north)
  std::vector<float> azimuth_angles;
  
  // Elevation angles (-90 to +90 degrees, where 0 = horizontal, +90 = zenith)
  std::vector<float> elevation_angles;
  
  // Antenna parameters
  float electrical_tilt_deg = 0.0f;  // Electrical downtilt
  float mechanical_tilt_deg = 0.0f;  // Mechanical downtilt
  float horizontal_beamwidth_deg = 360.0f; // 3dB beamwidth horizontal
  float vertical_beamwidth_deg = 90.0f;    // 3dB beamwidth vertical
  float front_to_back_ratio_db = 30.0f;    // Front-to-back isolation
  float max_gain_dbi = 0.0f;               // Maximum gain at boresight
  
  // Cross-polarization pattern (optional)
  bool has_cross_pol = false;
  std::vector<std::vector<float>> cross_pol_gain_db;
  
  // Metadata
  std::map<std::string, std::string> metadata;
  
  antenna_pattern_t() {
    // Default to simple omni pattern
    azimuth_angles.resize(360);
    for (int i = 0; i < 360; ++i) {
      azimuth_angles[i] = static_cast<float>(i);
    }
    
    elevation_angles = {0.0f};
    gain_db.resize(1);
    gain_db[0].resize(360, 0.0f);
    
    name = "Omni";
    horizontal_beamwidth_deg = 360.0f;
    vertical_beamwidth_deg = 90.0f;
  }
  
  // Get gain at specific azimuth and elevation with interpolation
  auto get_gain(float azimuth_deg, float elevation_deg = 0.0f, 
                bool interpolate = true) const -> float {
    if (gain_db.empty() || gain_db[0].empty()) {
      return 0.0f;
    }
    
    // Normalize azimuth to 0-360
    while (azimuth_deg < 0.0f) azimuth_deg += 360.0f;
    while (azimuth_deg >= 360.0f) azimuth_deg -= 360.0f;
    
    // Clamp elevation to -90 to +90
    elevation_deg = std::clamp(elevation_deg, -90.0f, 90.0f);
    
    if (!interpolate) {
      // Nearest neighbor lookup
      int azi_idx = find_nearest_azimuth_index(azimuth_deg);
      int elev_idx = find_nearest_elevation_index(elevation_deg);
      return gain_db[elev_idx][azi_idx];
    }
    
    // Bilinear interpolation
    return interpolate_gain(azimuth_deg, elevation_deg);
  }
  
  // Check if this is a 2D (azimuth-only) or 3D pattern
  auto is_3d() const -> bool {
    return elevation_angles.size() > 1;
  }
  
  // Apply electrical tilt (modifies the pattern in-place)
  auto apply_electrical_tilt(float tilt_deg) -> void {
    electrical_tilt_deg += tilt_deg;
    // Tilt is applied during gain lookup, not stored in the pattern data
  }
  
  // Apply mechanical tilt (modifies the pattern in-place)
  auto apply_mechanical_tilt(float tilt_deg) -> void {
    mechanical_tilt_deg += tilt_deg;
    // Tilt is applied during gain lookup, not stored in the pattern data
  }
  
  // Get the effective elevation angle accounting for tilts
  auto get_effective_elevation(float elevation_deg) const -> float {
    return elevation_deg - electrical_tilt_deg - mechanical_tilt_deg;
  }
  
  // Calculate 3dB beamwidth from pattern data
  auto calculate_beamwidths() -> void {
    if (gain_db.empty() || gain_db[0].empty()) return;
    
    // Find horizontal beamwidth at elevation = 0
    int mid_elev_idx = find_nearest_elevation_index(0.0f);
    if (mid_elev_idx >= 0 && mid_elev_idx < static_cast<int>(gain_db.size())) {
      horizontal_beamwidth_deg = calculate_3db_beamwidth(gain_db[mid_elev_idx]);
    }
    
    // Find vertical beamwidth at azimuth = 0 (boresight)
    if (is_3d()) {
      std::vector<float> vertical_cut;
      for (size_t i = 0; i < elevation_angles.size(); ++i) {
        int boresight_idx = find_nearest_azimuth_index(0.0f);
        vertical_cut.push_back(gain_db[i][boresight_idx]);
      }
      vertical_beamwidth_deg = calculate_3db_beamwidth(vertical_cut);
    }
  }
  
  // Get cross-polarization gain
  auto get_cross_pol_gain(float azimuth_deg, float elevation_deg = 0.0f) const -> float {
    if (!has_cross_pol || cross_pol_gain_db.empty()) {
      return -100.0f; // Very low cross-pol if not specified
    }
    
    while (azimuth_deg < 0.0f) azimuth_deg += 360.0f;
    while (azimuth_deg >= 360.0f) azimuth_deg -= 360.0f;
    elevation_deg = std::clamp(elevation_deg, -90.0f, 90.0f);
    
    int azi_idx = find_nearest_azimuth_index(azimuth_deg);
    int elev_idx = find_nearest_elevation_index(elevation_deg);
    
    return cross_pol_gain_db[elev_idx][azi_idx];
  }

private:
  auto find_nearest_azimuth_index(float azimuth_deg) const -> int {
    if (azimuth_angles.empty()) return 0;
    
    auto it = std::lower_bound(azimuth_angles.begin(), azimuth_angles.end(), azimuth_deg);
    if (it == azimuth_angles.end()) {
      return static_cast<int>(azimuth_angles.size() - 1);
    }
    if (it == azimuth_angles.begin()) {
      return 0;
    }
    
    // Find closest
    int idx = static_cast<int>(std::distance(azimuth_angles.begin(), it));
    float diff_high = azimuth_angles[idx] - azimuth_deg;
    float diff_low = azimuth_deg - azimuth_angles[idx - 1];
    
    return (diff_low < diff_high) ? idx - 1 : idx;
  }
  
  auto find_nearest_elevation_index(float elevation_deg) const -> int {
    if (elevation_angles.empty()) return 0;
    
    auto it = std::lower_bound(elevation_angles.begin(), elevation_angles.end(), elevation_deg);
    if (it == elevation_angles.end()) {
      return static_cast<int>(elevation_angles.size() - 1);
    }
    if (it == elevation_angles.begin()) {
      return 0;
    }
    
    int idx = static_cast<int>(std::distance(elevation_angles.begin(), it));
    float diff_high = elevation_angles[idx] - elevation_deg;
    float diff_low = elevation_deg - elevation_angles[idx - 1];
    
    return (diff_low < diff_high) ? idx - 1 : idx;
  }
  
  auto interpolate_gain(float azimuth_deg, float elevation_deg) const -> float {
    // Apply tilts
    elevation_deg = get_effective_elevation(elevation_deg);
    
    // Find surrounding indices
    int azi_idx = find_nearest_azimuth_index(azimuth_deg);
    int elev_idx = find_nearest_elevation_index(elevation_deg);
    
    // For now, use nearest neighbor (full bilinear interpolation is more complex)
    // TODO: Implement proper bilinear interpolation
    if (elev_idx >= 0 && elev_idx < static_cast<int>(gain_db.size()) &&
        azi_idx >= 0 && azi_idx < static_cast<int>(gain_db[elev_idx].size())) {
      return gain_db[elev_idx][azi_idx];
    }
    
    return -100.0f; // Very low gain if out of bounds
  }
  
  auto calculate_3db_beamwidth(const std::vector<float>& pattern) const -> float {
    if (pattern.empty()) return 360.0f;
    
    // Find maximum gain
    float max_gain = *std::max_element(pattern.begin(), pattern.end());
    float threshold = max_gain - 3.0f; // 3dB down from peak
    
    // Find first and last points above threshold
    int first = -1, last = -1;
    for (size_t i = 0; i < pattern.size(); ++i) {
      if (pattern[i] >= threshold) {
        if (first == -1) first = static_cast<int>(i);
        last = static_cast<int>(i);
      }
    }
    
    if (first == -1 || last == -1) return 360.0f;
    
    float beamwidth = static_cast<float>(last - first);
    return beamwidth * resolution.azimuth_step_deg;
  }
};

// Basic antenna pattern generators
class antenna_library_t {
public:
  // Generate omnidirectional pattern
  static auto generate_omni() -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Omnidirectional";
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->max_gain_dbi = 0.0f;
    return pattern;
  }
  
  // Generate isotropic pattern (perfect sphere - theoretical reference)
  static auto generate_isotropic() -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Isotropic";
    pattern->horizontal_beamwidth_deg = 360.0f;
    pattern->vertical_beamwidth_deg = 180.0f;
    pattern->max_gain_dbi = 0.0f;
    
    // Create full 3D pattern
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 10) {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }
    
    pattern->gain_db.resize(pattern->elevation_angles.size());
    for (auto& row : pattern->gain_db) {
      row.resize(360, 0.0f); // 0 dBi everywhere
    }
    
    return pattern;
  }
  
  // Generate sector antenna pattern
  static auto generate_sector(float beamwidth_deg, float fxb_ratio_db = 25.0f)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Sector " + std::to_string(static_cast<int>(beamwidth_deg)) + "°";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->front_to_back_ratio_db = fxb_ratio_db;
    pattern->gain_db[0].resize(360);
    
    for (int i = 0; i < 360; ++i) {
      float angle = static_cast<float>(i);
      if (angle > 180.0f) angle -= 360.0f;
      
      // Cosine-squared pattern approximation
      float gain = -3.0f * std::pow(angle / (beamwidth_deg / 2.0f), 2.0f);
      
      // Clamp to front-to-back ratio
      gain = std::max(gain, -fxb_ratio_db);
      
      pattern->gain_db[0][i] = gain;
    }
    
    pattern->max_gain_dbi = 0.0f;
    return pattern;
  }
  
  // Generate patch/panel antenna (narrower vertical beamwidth)
  static auto generate_patch(float h_beamwidth_deg, float v_beamwidth_deg, 
                             float gain_dbi = 6.0f)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Patch " + std::to_string(static_cast<int>(h_beamwidth_deg)) + "°x" +
                    std::to_string(static_cast<int>(v_beamwidth_deg)) + "°";
    pattern->horizontal_beamwidth_deg = h_beamwidth_deg;
    pattern->vertical_beamwidth_deg = v_beamwidth_deg;
    pattern->max_gain_dbi = gain_dbi;
    pattern->polarization = polarization_t::LINEAR_VERTICAL;
    
    // Create 3D pattern
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 5) {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }
    
    pattern->gain_db.resize(pattern->elevation_angles.size());
    
    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx) {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);
      
      for (int azi = 0; azi < 360; ++azi) {
        float azi_angle = static_cast<float>(azi);
        if (azi_angle > 180.0f) azi_angle -= 360.0f;
        
        // Horizontal pattern
        float h_gain = -3.0f * std::pow(azi_angle / (h_beamwidth_deg / 2.0f), 2.0f);
        h_gain = std::max(h_gain, -30.0f);
        
        // Vertical pattern
        float v_gain = -3.0f * std::pow(elev / (v_beamwidth_deg / 2.0f), 2.0f);
        v_gain = std::max(v_gain, -30.0f);
        
        // Combined gain (multiplicative in linear, additive in dB)
        pattern->gain_db[elev_idx][azi] = gain_dbi + h_gain + v_gain;
      }
    }
    
    return pattern;
  }
  
  // Generate Yagi antenna (directional with side lobes)
  static auto generate_yagi(float beamwidth_deg, int num_elements = 5)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Yagi " + std::to_string(num_elements) + "-element";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->front_to_back_ratio_db = 15.0f + static_cast<float>(num_elements);
    pattern->max_gain_dbi = 3.0f + static_cast<float>(num_elements) * 1.5f;
    pattern->gain_db[0].resize(360);
    
    for (int i = 0; i < 360; ++i) {
      float angle = static_cast<float>(i);
      if (angle > 180.0f) angle -= 360.0f;
      
      float gain = 0.0f;
      
      // Main lobe (forward)
      if (std::abs(angle) < beamwidth_deg / 2.0f) {
        gain = -3.0f * std::pow(angle / (beamwidth_deg / 2.0f), 2.0f);
      }
      // Side lobes
      else if (std::abs(angle) < 90.0f) {
        float side_lobe_level = -15.0f - static_cast<float>(num_elements);
        gain = side_lobe_level + 5.0f * std::cos(angle * M_PI / 180.0f);
      }
      // Back lobe
      else {
        gain = -pattern->front_to_back_ratio_db;
      }
      
      pattern->gain_db[0][i] = gain;
    }
    
    return pattern;
  }
  
  // Generate horn antenna (high gain, narrow beam)
  static auto generate_horn(float beamwidth_deg, float gain_dbi = 15.0f)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Horn " + std::to_string(static_cast<int>(gain_dbi)) + " dBi";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = gain_dbi;
    pattern->front_to_back_ratio_db = 40.0f;
    pattern->gain_db[0].resize(360);
    
    for (int i = 0; i < 360; ++i) {
      float angle = static_cast<float>(i);
      if (angle > 180.0f) angle -= 360.0f;
      
      // Horn pattern (very clean, low side lobes)
      float gain = -12.0f * std::pow(angle / beamwidth_deg, 2.0f);
      gain = std::max(gain, -40.0f);
      
      pattern->gain_db[0][i] = gain;
    }
    
    return pattern;
  }
  
  // Generate parabolic reflector antenna (very high gain)
  static auto generate_parabolic(float diameter_m, float frequency_mhz)
      -> std::shared_ptr<antenna_pattern_t> {
    // Calculate gain from diameter
    float wavelength_m = 300.0f / frequency_mhz;
    float efficiency = 0.6f; // Typical parabolic efficiency
    float gain_dbi = 10.0f * std::log10(efficiency * std::pow(M_PI * diameter_m / wavelength_m, 2.0f));
    
    // Calculate beamwidth (degrees)
    float beamwidth_deg = 70.0f * wavelength_m / diameter_m;
    
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Parabolic " + std::to_string(diameter_m) + "m";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->vertical_beamwidth_deg = beamwidth_deg;
    pattern->max_gain_dbi = gain_dbi;
    pattern->frequency_mhz = frequency_mhz;
    pattern->front_to_back_ratio_db = 50.0f;
    pattern->gain_db[0].resize(360);
    
    for (int i = 0; i < 360; ++i) {
      float angle = static_cast<float>(i);
      if (angle > 180.0f) angle -= 360.0f;
      
      // Parabolic pattern (very narrow beam)
      float normalized_angle = angle / beamwidth_deg;
      float gain = -12.0f * normalized_angle * normalized_angle;
      
      // Very low side lobes
      gain = std::max(gain, -50.0f);
      
      pattern->gain_db[0][i] = gain;
    }
    
    return pattern;
  }
  
  // Generate dipole antenna (figure-8 pattern)
  static auto generate_dipole() -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Dipole";
    pattern->horizontal_beamwidth_deg = 78.0f; // Approximate
    pattern->max_gain_dbi = 2.15f; // Standard dipole gain
    pattern->polarization = polarization_t::LINEAR_VERTICAL;
    
    // Create 3D pattern
    pattern->elevation_angles.clear();
    for (int el = -90; el <= 90; el += 5) {
      pattern->elevation_angles.push_back(static_cast<float>(el));
    }
    
    pattern->gain_db.resize(pattern->elevation_angles.size());
    
    for (size_t elev_idx = 0; elev_idx < pattern->elevation_angles.size(); ++elev_idx) {
      float elev = pattern->elevation_angles[elev_idx];
      pattern->gain_db[elev_idx].resize(360);
      
      // Dipole has omnidirectional pattern in azimuth
      // Null at zenith and nadir (elevation ±90°)
      float elev_factor = std::cos(elev * M_PI / 180.0f);
      float gain = 2.15f + 10.0f * std::log10(elev_factor * elev_factor + 0.001f);
      
      for (int azi = 0; azi < 360; ++azi) {
        pattern->gain_db[elev_idx][azi] = gain;
      }
    }
    
    return pattern;
  }
  
  // Generate bi-directional antenna (e.g., for point-to-point links)
  static auto generate_bidirectional(float beamwidth_deg)
      -> std::shared_ptr<antenna_pattern_t> {
    auto pattern = std::make_shared<antenna_pattern_t>();
    pattern->name = "Bi-directional " + std::to_string(static_cast<int>(beamwidth_deg)) + "°";
    pattern->horizontal_beamwidth_deg = beamwidth_deg;
    pattern->gain_db[0].resize(360);
    
    for (int i = 0; i < 360; ++i) {
      float angle = static_cast<float>(i);
      if (angle > 180.0f) angle -= 360.0f;
      
      // Two main lobes at 0° and 180°
      float angle_from_0 = std::abs(angle);
      float angle_from_180 = std::abs(std::abs(angle) - 180.0f);
      
      float gain_0 = -3.0f * std::pow(angle_from_0 / (beamwidth_deg / 2.0f), 2.0f);
      float gain_180 = -3.0f * std::pow(angle_from_180 / (beamwidth_deg / 2.0f), 2.0f);
      
      float gain = std::max(gain_0, gain_180);
      gain = std::max(gain, -30.0f);
      
      pattern->gain_db[0][i] = gain;
    }
    
    return pattern;
  }
};

} // namespace sensor_mapper