#pragma once

#include "antenna_pattern.hpp"
#include <array>
#include <memory>
#include <string>


namespace sensor_mapper {

enum class PropagationModel {
  FreeSpace = 0,
  HataUrban = 1,
  HataSuburban = 2,
  HataRural = 3
};

class sensor_t {
public:
  sensor_t(const std::string &name, double latitude, double longitude,
           double range);
  ~sensor_t() = default;

  auto get_name() const -> const std::string &;
  auto set_name(const std::string &name) -> void;

  auto get_latitude() const -> double;
  auto set_latitude(double latitude) -> void;

  auto get_longitude() const -> double;
  auto set_longitude(double longitude) -> void;

  auto get_range() const -> double;
  auto set_range(double range) -> void;

  auto get_mast_height() const -> double;
  auto set_mast_height(double height) -> void;

  auto get_ground_elevation() const -> double;
  auto set_ground_elevation(double elevation) -> void;

  auto get_use_auto_elevation() const -> bool;
  auto set_use_auto_elevation(bool use_auto) -> void;

  auto get_color_data() -> float *;
  auto get_color() const -> std::array<float, 3>;

  auto get_id() const -> const std::string &;
  auto set_id(const std::string &id) -> void;

  // RF Propagation Parameters
  auto get_tx_power_dbm() const -> double;
  auto set_tx_power_dbm(double power) -> void;

  auto get_frequency_mhz() const -> double;
  auto set_frequency_mhz(double freq) -> void;

  auto get_tx_antenna_gain_dbi() const -> double;
  auto set_tx_antenna_gain_dbi(double gain) -> void;

  auto get_rx_sensitivity_dbm() const -> double;
  auto set_rx_sensitivity_dbm(double sensitivity) -> void;

  auto get_rx_antenna_gain_dbi() const -> double;
  auto set_rx_antenna_gain_dbi(double gain) -> void;

  // Directional Antenna Parameters
  auto get_azimuth_deg() const -> double;
  auto set_azimuth_deg(double azimuth) -> void;

  auto get_beamwidth_deg() const -> double;
  auto set_beamwidth_deg(double beamwidth) -> void;

  auto set_custom_pattern(std::shared_ptr<antenna_pattern_t> pattern) -> void;
  auto get_pattern() const -> std::shared_ptr<antenna_pattern_t>;
  auto get_antenna_gain(double angle_deg) const -> double;
  auto get_antenna_gain(double azimuth_deg, double elevation_deg) const -> double;
  
  // Antenna Tilt Parameters
  auto get_electrical_tilt_deg() const -> double;
  auto set_electrical_tilt_deg(double tilt) -> void;
  auto get_mechanical_tilt_deg() const -> double;
  auto set_mechanical_tilt_deg(double tilt) -> void;

  // Propagation Model
  auto get_propagation_model() const -> PropagationModel;
  auto set_propagation_model(PropagationModel model) -> void;

private:
  std::string m_id;
  std::string m_name;
  double m_latitude;
  double m_longitude;
  double m_range;
  double m_mast_height;
  double m_ground_elevation;
  bool m_use_auto_elevation;
  std::array<float, 3> m_color;

  // RF Propagation Parameters
  double m_tx_power_dbm;        // Transmit power (dBm)
  double m_frequency_mhz;       // Operating frequency (MHz)
  double m_tx_antenna_gain_dbi; // TX antenna gain (dBi)
  double m_rx_sensitivity_dbm;  // RX sensitivity threshold (dBm)
  double m_rx_antenna_gain_dbi; // RX antenna gain (dBi)

  // Directional Antenna
  double m_azimuth_deg;   // 0-360 degrees (North=0, CW)
  double m_beamwidth_deg; // 0-360 degrees
  std::shared_ptr<antenna_pattern_t> m_pattern;

  // Model
  PropagationModel m_propagation_model;
};

} // namespace sensor_mapper
