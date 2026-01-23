#pragma once

#include "core/elevation_source.hpp"
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
  auto update() -> void;

  auto add_source(std::shared_ptr<elevation_source_t> source) -> void;
  auto load_local_file(const std::string &path) -> bool;

private:
  std::vector<std::shared_ptr<elevation_source_t>> m_sources;
};

} // namespace sensor_mapper
