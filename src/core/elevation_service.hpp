#pragma once

#include "elevation_source.hpp"
#include <vector>
#include <memory>
#include <string>

namespace sensor_mapper
{

class elevation_service_t
{
public:
  elevation_service_t();
  ~elevation_service_t();

  auto get_elevation(double lat, double lon, float &out_height) -> bool;

  // Get terrain profile between two points
  // Returns a vector of pairs: {distance_from_start_m, elevation_m}
  auto get_profile(double start_lat, double start_lon, double end_lat, double end_lon, int num_samples = 100) -> std::vector<std::pair<double, float>>;

  auto update() -> void;

  auto add_source(std::shared_ptr<elevation_source_t> source) -> void;
  auto load_local_file(const std::string &path) -> bool;

  auto get_sources() const -> const std::vector<std::shared_ptr<elevation_source_t>> &
  {
    return m_sources;
  }
  auto get_source_count() const -> size_t
  {
    return m_sources.size();
  }

private:
  std::vector<std::shared_ptr<elevation_source_t>> m_sources;
};

} // namespace sensor_mapper
