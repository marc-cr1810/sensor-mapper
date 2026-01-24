#pragma once

#include "core/elevation_source.hpp"
#include <string>
#include <vector>
#include <filesystem>
#include <future>
#include <mutex>
#include <chrono>

namespace sensor_mapper
{

class geotiff_source_t : public elevation_source_t
{
public:
  explicit geotiff_source_t(const std::filesystem::path &path);
  ~geotiff_source_t() override;

  auto get_elevation(double lat, double lon, float &out_height) -> bool override;
  auto get_bounds(double &min_lat, double &max_lat, double &min_lon, double &max_lon) const -> bool override;
  auto get_bounds_quad(double *lats, double *lons) const -> bool override;
  auto update() -> void override;
  auto get_name() const -> const char * override
  {
    return "GeoTIFF Local";
  }

  auto get_visual_data(int &w, int &h) const -> const float * override;

  auto load() -> bool;
  auto get_scanline(int row) -> const float *;

private:
  std::filesystem::path m_path;

  // Async Loading State
  std::future<bool> m_load_future;
  bool m_loading_started = false;

  // On-demand loading - keep file open
  void *m_tif = nullptr; // TIFF* (void* to avoid including tiffio.h in header)
  std::mutex m_file_mutex;

  // LRU Cache for scanlines
  struct scanline_cache_entry_t
  {
    int row = -1;
    std::vector<float> data;
    std::chrono::steady_clock::time_point last_access;
  };
  std::vector<scanline_cache_entry_t> m_scanline_cache;
  static constexpr size_t MAX_CACHED_SCANLINES = 100;
  int m_width = 0;
  int m_height = 0;

  // Georeferencing (ModelTiepoint + PixelScale)
  double m_origin_x = 0.0;
  double m_origin_y = 0.0;
  double m_scale_x = 1.0;
  double m_scale_y = 1.0;

  // Raw Tiepoint (Maps pixel i,j,k to X,Y,Z)
  double m_tie_i = 0.0, m_tie_j = 0.0, m_tie_k = 0.0;
  double m_tie_x = 0.0, m_tie_y = 0.0, m_tie_z = 0.0;

  std::unique_ptr<class crs_transformer_t> m_transformer;
  bool m_loaded = false;

  // Visualization thumbnail (512x512 downsampled)
  std::vector<float> m_visual_data;
  int m_visual_width = 0;
  int m_visual_height = 0;
};

} // namespace sensor_mapper
