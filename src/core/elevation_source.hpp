#pragma once

namespace sensor_mapper
{

class elevation_source_t
{
public:
  virtual ~elevation_source_t() = default;

  // Try to get elevation at specific lat/lon. Returns true if found.
  virtual auto get_elevation(double lat, double lon, float &out_height) -> bool = 0;

  // Update routine for async loading
  virtual auto update() -> void = 0;

  // Name/Type of source
  virtual auto get_name() const -> const char * = 0;
};

} // namespace sensor_mapper
