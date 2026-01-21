#include "core/sensor.hpp"

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
      m_color({1.0f, 0.0f, 0.0f}) {
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

} // namespace sensor_mapper
