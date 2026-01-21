#pragma once

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

private:
  std::string m_name;
  double m_latitude;
  double m_longitude;
  double m_range;
};

} // namespace sensor_mapper
