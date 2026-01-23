#pragma once

#include "core/elevation_source.hpp"
#include <string>
#include <vector>
#include <filesystem>

namespace sensor_mapper
{

class geotiff_source_t : public elevation_source_t
{
public:
  explicit geotiff_source_t(const std::filesystem::path &path);
  ~geotiff_source_t() override;

  auto get_elevation(double lat, double lon, float &out_height) -> bool override;
  auto update() -> void override;
  auto get_name() const -> const char * override
  {
    return "GeoTIFF Local";
  }

  auto load() -> bool;

private:
  std::filesystem::path m_path;
  std::vector<float> m_data;
  int m_width = 0;
  int m_height = 0;

  // Georeferencing (ModelTiepoint + PixelScale)
  double m_origin_x = 0.0;
  double m_origin_y = 0.0;
  double m_scale_x = 1.0;
  double m_scale_y = 1.0;

  bool m_loaded = false;
};

} // namespace sensor_mapper
