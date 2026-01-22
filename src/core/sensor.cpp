#include "sensor.hpp"

namespace sensor_mapper {

// Helper for random ID
static auto generate_id() -> std::string {
  const char charset[] = "0123456789abcdef";
  std::string result;
  result.resize(16);
  for (int i = 0; i < 16; ++i) {
    result[i] = charset[rand() % 16];
  }
  return result;
}

sensor_t::sensor_t(const std::string &name, double latitude, double longitude,
                   double range)
    : m_id(generate_id()), m_name(name), m_latitude(latitude),
      m_longitude(longitude), m_range(range), m_mast_height(10.0),
      m_ground_elevation(0.0), m_use_auto_elevation(true),
      m_color({1.0f, 0.0f, 0.0f}),
      // RF parameters - defaults for typical LoRa/ISM band drone link
      m_tx_power_dbm(20.0),         // 100mW (typical for ISM band)
      m_frequency_mhz(915.0),       // 915 MHz ISM band (US)
      m_tx_antenna_gain_dbi(2.15),  // Dipole antenna
      m_rx_sensitivity_dbm(-120.0), // Typical LoRa sensitivity
      m_rx_antenna_gain_dbi(0.0),   // Drone omni antenna
      m_azimuth_deg(0.0), m_beamwidth_deg(360.0),
      m_propagation_model(PropagationModel::FreeSpace) {
  // Random color logic...
  m_color = {static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
             static_cast<float>(rand()) / static_cast<float>(RAND_MAX),
             static_cast<float>(rand()) / static_cast<float>(RAND_MAX)};
}

auto sensor_t::get_name() const -> const std::string & { return m_name; }

auto sensor_t::set_name(const std::string &name) -> void { m_name = name; }

auto sensor_t::get_latitude() const -> double { return m_latitude; }

auto sensor_t::set_latitude(double latitude) -> void { m_latitude = latitude; }

auto sensor_t::get_longitude() const -> double { return m_longitude; }

auto sensor_t::set_longitude(double longitude) -> void {
  m_longitude = longitude;
}

auto sensor_t::get_range() const -> double { return m_range; }

auto sensor_t::set_range(double range) -> void { m_range = range; }

auto sensor_t::get_mast_height() const -> double { return m_mast_height; }
auto sensor_t::set_mast_height(double height) -> void {
  m_mast_height = height;
}

auto sensor_t::get_ground_elevation() const -> double {
  return m_ground_elevation;
}
auto sensor_t::set_ground_elevation(double elevation) -> void {
  m_ground_elevation = elevation;
}

auto sensor_t::get_use_auto_elevation() const -> bool {
  return m_use_auto_elevation;
}
auto sensor_t::set_use_auto_elevation(bool use_auto) -> void {
  m_use_auto_elevation = use_auto;
}

auto sensor_t::get_color_data() -> float * { return m_color.data(); }

auto sensor_t::get_color() const -> std::array<float, 3> { return m_color; }

auto sensor_t::get_id() const -> const std::string & { return m_id; }
auto sensor_t::set_id(const std::string &id) -> void { m_id = id; }

// RF Propagation Parameter Accessors
auto sensor_t::get_tx_power_dbm() const -> double { return m_tx_power_dbm; }
auto sensor_t::set_tx_power_dbm(double power) -> void {
  m_tx_power_dbm = power;
}

auto sensor_t::get_frequency_mhz() const -> double { return m_frequency_mhz; }
auto sensor_t::set_frequency_mhz(double freq) -> void {
  m_frequency_mhz = freq;
}

auto sensor_t::get_tx_antenna_gain_dbi() const -> double {
  return m_tx_antenna_gain_dbi;
}
auto sensor_t::set_tx_antenna_gain_dbi(double gain) -> void {
  m_tx_antenna_gain_dbi = gain;
}

auto sensor_t::get_rx_sensitivity_dbm() const -> double {
  return m_rx_sensitivity_dbm;
}
auto sensor_t::set_rx_sensitivity_dbm(double sensitivity) -> void {
  m_rx_sensitivity_dbm = sensitivity;
}

auto sensor_t::get_rx_antenna_gain_dbi() const -> double {
  return m_rx_antenna_gain_dbi;
}
auto sensor_t::set_rx_antenna_gain_dbi(double gain) -> void {
  m_rx_antenna_gain_dbi = gain;
}

auto sensor_t::get_azimuth_deg() const -> double { return m_azimuth_deg; }
auto sensor_t::set_azimuth_deg(double azimuth) -> void {
  m_azimuth_deg = azimuth;
}

auto sensor_t::get_beamwidth_deg() const -> double { return m_beamwidth_deg; }
auto sensor_t::set_beamwidth_deg(double beamwidth) -> void {
  m_beamwidth_deg = beamwidth;
}

auto sensor_t::set_custom_pattern(std::shared_ptr<antenna_pattern_t> pattern)
    -> void {
  m_pattern = pattern;
}

auto sensor_t::get_pattern() const -> std::shared_ptr<antenna_pattern_t> {
  return m_pattern;
}

auto sensor_t::get_antenna_gain(double angle_deg) const -> double {
  // If we have a custom pattern, use it
  if (m_pattern && !m_pattern->gain_db.empty() && !m_pattern->gain_db[0].empty()) {
    // angle_deg is deviation from North (0-360)
    // We need deviation from Azimuth
    double rel_angle = angle_deg - m_azimuth_deg;

    // Use the new get_gain method with interpolation
    // Returns gain in dB relative to pattern's max gain
    return m_pattern->get_gain(static_cast<float>(rel_angle), 0.0f, true);
  }

  // Fallback to simple beamwidth model
  double angle_diff = std::abs(angle_deg - m_azimuth_deg);
  if (angle_diff > 180.0)
    angle_diff = 360.0 - angle_diff;

  // Ideally, gain is part of tx_antenna_gain_dbi for the main beam.
  // Here we return ATTENUATION (negative gain) relative to peak.
  // 0.0 means peak gain. -25.0 means side lobe.
  return (angle_diff > m_beamwidth_deg / 2.0) ? -25.0 : 0.0;
}

auto sensor_t::get_antenna_gain(double azimuth_deg, double elevation_deg) const -> double {
  // If we have a custom pattern, use it with 3D support
  if (m_pattern && !m_pattern->gain_db.empty() && !m_pattern->gain_db[0].empty()) {
    // azimuth_deg is deviation from North (0-360)
    // We need deviation from Azimuth
    double rel_angle = azimuth_deg - m_azimuth_deg;

    // Use the new get_gain method with interpolation and elevation support
    return m_pattern->get_gain(static_cast<float>(rel_angle), 
                               static_cast<float>(elevation_deg), true);
  }

  // Fallback to 2D method (ignores elevation)
  return get_antenna_gain(azimuth_deg);
}

auto sensor_t::get_electrical_tilt_deg() const -> double {
  if (m_pattern) {
    return static_cast<double>(m_pattern->electrical_tilt_deg);
  }
  return 0.0;
}

auto sensor_t::set_electrical_tilt_deg(double tilt) -> void {
  if (m_pattern) {
    m_pattern->electrical_tilt_deg = static_cast<float>(tilt);
  }
}

auto sensor_t::get_mechanical_tilt_deg() const -> double {
  if (m_pattern) {
    return static_cast<double>(m_pattern->mechanical_tilt_deg);
  }
  return 0.0;
}

auto sensor_t::set_mechanical_tilt_deg(double tilt) -> void {
  if (m_pattern) {
    m_pattern->mechanical_tilt_deg = static_cast<float>(tilt);
  }
}

auto sensor_t::get_propagation_model() const -> PropagationModel {
  return m_propagation_model;
}
auto sensor_t::set_propagation_model(PropagationModel model) -> void {
  m_propagation_model = model;
}

} // namespace sensor_mapper
