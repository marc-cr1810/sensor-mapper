#include "core/sensor.hpp"

namespace sensor_mapper {

sensor_t::sensor_t(const std::string &name, double latitude, double longitude,
                   double range)
    : m_name(name), m_latitude(latitude), m_longitude(longitude),
      m_range(range) {}

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

} // namespace sensor_mapper
