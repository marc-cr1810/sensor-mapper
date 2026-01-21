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

  auto get_altitude() const -> double;
  auto set_altitude(double altitude) -> void;

  auto get_color_data() -> float *;
  auto get_color() const -> std::array<float, 3>;

private:
  std::string m_name;
  double m_latitude;
  double m_longitude;
  double m_range;
  double m_altitude;
  std::array<float, 3> m_color;
};

} // namespace sensor_mapper
