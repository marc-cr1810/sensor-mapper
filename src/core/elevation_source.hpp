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

  // param: min_lat, max_lat, min_lon, max_lon
  virtual auto get_bounds(double &min_lat, double &max_lat, double &min_lon, double &max_lon) const -> bool
  {
    return false;
  }

  // Name/Type of source
  virtual auto get_name() const -> const char * = 0;
};

} // namespace sensor_mapper
