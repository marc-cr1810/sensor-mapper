#pragma once

#include "core/elevation_source.hpp"
#include "core/crs_transformer.hpp"
#include <string>
#include <vector>
#include <memory>
#include <filesystem>

namespace sensor_mapper
{

class lidar_source_t : public elevation_source_t
{
public:
  explicit lidar_source_t(const std::filesystem::path &path);
  ~lidar_source_t() override;

  auto get_elevation(double lat, double lon, float &out_height) -> bool override;
  auto update() -> void override;
  auto get_name() const -> const char * override
  {
    return "LIDAR Local";
  }

  auto load() -> bool;

private:
  std::filesystem::path m_path;

  // Rasterized DSM from LAS points
  std::vector<float> m_data; // Grid of max Z values
  int m_width = 0;
  int m_height = 0;

  // Bounds
  double m_min_x = 0.0;
  double m_min_y = 0.0;
  double m_max_x = 0.0;
  double m_max_y = 0.0;

  double m_cell_size = 1.0; // Meters per pixel (default)

  bool m_loaded = false;

  // CRS Transformer (handles PROJ or fallback)
  std::unique_ptr<crs_transformer_t> m_transformer;
};

} // namespace sensor_mapper
