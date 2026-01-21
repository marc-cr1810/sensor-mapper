#pragma once

#include <array>
#include <string>

namespace sensor_mapper {

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
};

} // namespace sensor_mapper
