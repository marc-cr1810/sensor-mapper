#include "core/geotiff_source.hpp"
#include "core/crs_transformer.hpp"
#include <tiffio.h>
#include <iostream>
#include <cmath>

namespace sensor_mapper
{

auto geotiff_source_t::load() -> bool
{

  // Suppress warnings for GeoTIFF tags (they're expected and harmless)
  TIFFSetWarningHandler(nullptr);

  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
  {
    std::cerr << "Failed to open GeoTIFF: " << m_path << std::endl;
    return false;
  }

  uint32_t w, h;
  if (!TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w) || !TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h))
  {
    std::cerr << "Failed to read GeoTIFF dimensions: " << m_path << std::endl;
    TIFFClose(tif);
    return false;
  }

  m_width = w;
  m_height = h;

  // Read Georeferencing Tags - use defaults if not present

  // Set safe defaults first
  m_scale_x = 1.0;
  m_scale_y = 1.0;
  m_origin_x = 0.0;
  m_origin_y = 0.0;

  // Read GeoTIFF tags safely using raw data access
  uint32_t scale_count = 0;
  double *scales = nullptr;
  if (TIFFGetField(tif, 33550, &scale_count, &scales) && scale_count >= 3 && scales)
  {
    m_scale_x = scales[0];
    m_scale_y = scales[1];
  }

  uint32_t tiepoint_count = 0;
  void *tiepoint_raw = nullptr;
  if (TIFFGetField(tif, 33922, &tiepoint_count, &tiepoint_raw) && tiepoint_count >= 6 && tiepoint_raw)
  {
    double *tiepoints = (double *)tiepoint_raw;
    m_tie_i = tiepoints[0];
    m_tie_j = tiepoints[1];
    m_tie_k = tiepoints[2];
    m_tie_x = tiepoints[3];
    m_tie_y = tiepoints[4];
    m_tie_z = tiepoints[5];

    // Pixel (tie_i, tie_j) maps to (tie_x, tie_y)
    m_origin_x = m_tie_x - m_tie_i * m_scale_x;
    m_origin_y = m_tie_y + m_tie_j * m_scale_y; // Pixel J increases down, Projected Y increases up
  }

  // Detect CRS from GeoKeys
  std::string epsg_code = "EPSG:32755"; // Default fallback (MGA 55S)
  uint16_t *keys = nullptr;
  uint32_t key_count = 0;
  if (TIFFGetField(tif, 34735, &key_count, &keys) && keys)
  {
    // GeoKeyDirectory header: [Version, Rev, Minor, NumKeys]
    if (key_count >= 4)
    {
      int num_keys = keys[3];
      for (int i = 0; i < num_keys; ++i)
      {
        if (4 + i * 4 + 3 >= key_count)
          break;

        uint16_t key_id = keys[4 + i * 4];
        uint16_t location = keys[5 + i * 4];
        uint16_t count = keys[6 + i * 4];
        uint16_t value_offset = keys[7 + i * 4];

        if (key_id == 3072) // ProjectedCSTypeGeoKey
        {
          if (location == 0) // Value is in offset field
            epsg_code = "EPSG:" + std::to_string(value_offset);
        }
      }
    }
  }

  m_transformer = std::make_unique<crs_transformer_t>();
  if (!m_transformer->init(epsg_code))
  {
    std::cerr << "GeoTIFF: Failed to init transformer for " << epsg_code << std::endl;
  }

  // --- Generate Visualization Thumbnail ---
  // Increased limit for better zoom resolution
  constexpr int MAX_VISUAL_DIM = 4096;
  m_visual_width = std::min((int)w, MAX_VISUAL_DIM);
  m_visual_height = std::min((int)h, MAX_VISUAL_DIM);
  m_visual_data.assign(m_visual_width * m_visual_height, -1.0f);

  float min_ele = 1e10f;
  float max_ele = -1e10f;

  for (int vy = 0; vy < m_visual_height; ++vy)
  {
    int sy = (int)((double)vy / m_visual_height * h);
    float *buf = (float *)_TIFFmalloc(TIFFScanlineSize(tif));
    if (TIFFReadScanline(tif, buf, sy) >= 0)
    {
      for (int vx = 0; vx < m_visual_width; ++vx)
      {
        int sx = (int)((double)vx / m_visual_width * w);
        float val = buf[sx];
        if (val > -1000.0f)
        {
          min_ele = std::min(min_ele, val);
          max_ele = std::max(max_ele, val);
        }
        m_visual_data[vy * m_visual_width + vx] = val;
      }
    }
    _TIFFfree(buf);
  }

  if (max_ele > min_ele)
  {
    for (auto &val : m_visual_data)
    {
      if (val > -1000.0f)
        val = (val - min_ele) / (max_ele - min_ele);
      else
        val = -1.0f;
    }
  }

  TIFFClose(tif);

  m_loaded = true;
  return true;
}

auto geotiff_source_t::get_visual_data(int &w, int &h) const -> const float *
{
  if (!m_loaded)
    return nullptr;
  w = m_visual_width;
  h = m_visual_height;
  return m_visual_data.data();
}

geotiff_source_t::geotiff_source_t(const std::filesystem::path &path) : m_path(path)
{
  // Async load
  m_load_future = std::async(std::launch::async, [this]() { return load(); });
  m_loading_started = true;
}

geotiff_source_t::~geotiff_source_t()
{
  if (m_loading_started && m_load_future.valid())
  {
    m_load_future.wait();
  }

  // Close TIFF file if open
  if (m_tif)
  {
    TIFFClose(static_cast<TIFF *>(m_tif));
    m_tif = nullptr;
  }

  // Clear cache
  m_scanline_cache.clear();
}

auto geotiff_source_t::update() -> void
{
  if (!m_loaded && m_loading_started && m_load_future.valid())
  {
    if (m_load_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      bool success = m_load_future.get();
      if (success)
      {
        m_loaded = true;
      }
      else
      {
        std::cerr << "GeoTIFF: Async load failed for " << m_path << std::endl;
        m_loading_started = false;
      }
    }
  }
}

auto geotiff_source_t::get_bounds(double &min_lat, double &max_lat, double &min_lon, double &max_lon) const -> bool
{
  double lats[4], lons[4];
  if (!get_bounds_quad(lats, lons))
    return false;

  min_lat = 90.0;
  max_lat = -90.0;
  min_lon = 180.0;
  max_lon = -180.0;

  for (int i = 0; i < 4; ++i)
  {
    min_lat = std::min(min_lat, lats[i]);
    max_lat = std::max(max_lat, lats[i]);
    min_lon = std::min(min_lon, lons[i]);
    max_lon = std::max(max_lon, lons[i]);
  }
  return true;
}

auto geotiff_source_t::get_bounds_quad(double *lats, double *lons) const -> bool
{
  if (!m_loaded || !m_transformer || !m_transformer->is_valid())
    return false;

  // Quad Corners (Easting, Northing)
  double pts_x[4] = {
      m_origin_x,                       // TL
      m_origin_x + m_width * m_scale_x, // TR
      m_origin_x + m_width * m_scale_x, // BR
      m_origin_x                        // BL
  };
  double pts_y[4] = {
      m_origin_y,                        // TL
      m_origin_y,                        // TR
      m_origin_y - m_height * m_scale_y, // BR
      m_origin_y - m_height * m_scale_y  // BL
  };

  for (int i = 0; i < 4; ++i)
  {
    if (!m_transformer->transform(pts_x[i], pts_y[i], lats[i], lons[i]))
      return false;
  }

  return true;
}

auto geotiff_source_t::get_scanline(int row) -> const float *
{
  std::lock_guard<std::mutex> lock(m_file_mutex);

  if (row < 0 || row >= m_height)
    return nullptr;

  // Check cache first
  for (auto &entry : m_scanline_cache)
  {
    if (entry.row == row)
    {
      // Update access time
      entry.last_access = std::chrono::steady_clock::now();
      return entry.data.data();
    }
  }

  // Not in cache - open file fresh for thread safety
  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
    return nullptr;

  std::vector<float> scanline(m_width);

  // Allocate buffer for reading
  float *buf = (float *)_TIFFmalloc(TIFFScanlineSize(tif));
  if (!buf)
  {
    TIFFClose(tif);
    return nullptr;
  }

  // Read scanline
  if (TIFFReadScanline(tif, buf, row) < 0)
  {
    _TIFFfree(buf);
    TIFFClose(tif);
    return nullptr;
  }

  // Copy to our vector
  for (int i = 0; i < m_width; ++i)
  {
    scanline[i] = buf[i];
  }
  _TIFFfree(buf);
  TIFFClose(tif); // Close immediately after reading

  // Add to cache (evict oldest if full)
  if (m_scanline_cache.size() >= MAX_CACHED_SCANLINES)
  {
    // Find oldest entry
    auto oldest = std::min_element(m_scanline_cache.begin(), m_scanline_cache.end(), [](const scanline_cache_entry_t &a, const scanline_cache_entry_t &b) { return a.last_access < b.last_access; });

    // Replace oldest
    oldest->row = row;
    oldest->data = std::move(scanline);
    oldest->last_access = std::chrono::steady_clock::now();
    return oldest->data.data();
  }
  else
  {
    // Add new entry
    m_scanline_cache.push_back({row, std::move(scanline), std::chrono::steady_clock::now()});
    return m_scanline_cache.back().data.data();
  }
}

auto geotiff_source_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  if (!m_loaded || !m_transformer || !m_transformer->is_valid())
    return false;

  // Transform Lat/Lon -> Projected Coordinate (UTM)
  double proj_x, proj_y;
  if (!m_transformer->transform(lat, lon, proj_x, proj_y, false))
    return false;

  // Inverse mapping: Projected -> Raster
  // projected_x = origin_x + pixel_x * scale_x  => pixel_x = (proj_x - origin_x) / scale_x
  // projected_y = origin_y - pixel_y * scale_y  => pixel_y = (origin_y - proj_y) / scale_y

  double px = (proj_x - m_origin_x) / m_scale_x;
  double py = (m_origin_y - proj_y) / m_scale_y;

  int x = static_cast<int>(std::floor(px));
  int y = static_cast<int>(std::floor(py));

  if (x >= 0 && x < m_width && y >= 0 && y < m_height)
  {
    const float *row_data = get_scanline(y);
    if (!row_data)
      return false;

    out_height = row_data[x];
    if (out_height < -1000.0f) // Nodata
      return false;

    return true;
  }

  return false;
}

auto geotiff_source_t::get_visual_data_window(double min_lat, double max_lat, double min_lon, double max_lon, int &out_w, int &out_h, std::vector<float> &out_data) -> bool
{
  if (!m_loaded || !m_transformer || !m_transformer->is_valid())
    return false;

  // Determine window in Raster Coordinates
  // We need to project the 4 corners to find the raster bounding box
  double lats[4] = {max_lat, max_lat, min_lat, min_lat}; // TL, TR, BR, BL
  double lons[4] = {min_lon, max_lon, max_lon, min_lon};

  int min_x = m_width, max_x = 0;
  int min_y = m_height, max_y = 0;

  for (int i = 0; i < 4; ++i)
  {
    double proj_x, proj_y;
    if (m_transformer->transform(lats[i], lons[i], proj_x, proj_y, false))
    {
      double px = (proj_x - m_origin_x) / m_scale_x;
      double py = (m_origin_y - proj_y) / m_scale_y;

      int x = static_cast<int>(std::floor(px));
      int y = static_cast<int>(std::floor(py));

      min_x = std::min(min_x, x);
      max_x = std::max(max_x, x);
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }
  }

  // Clamp to image bounds
  min_x = std::max(0, min_x);
  max_x = std::min(m_width - 1, max_x);
  min_y = std::max(0, min_y);
  max_y = std::min(m_height - 1, max_y);

  if (min_x >= max_x || min_y >= max_y)
    return false;

  int raw_w = max_x - min_x + 1;
  int raw_h = max_y - min_y + 1;

  // Limit output resolution (e.g. max 1024x1024 per tile/view)
  // If the requested area is huge, we downsample.
  // If it's small, we get native resolution.
  constexpr int MAX_RES = 1024;

  // Actually, we should aim for what was requested or a reasonable limit.
  // Let's set out_w/out_h to MIN(raw, MAX_RES) while preserving aspect ratio?
  // Or just sample a fixed grid?
  // Let's try to match aspect ratio of the window.

  double aspect = (double)raw_w / raw_h;
  if (raw_w > MAX_RES || raw_h > MAX_RES)
  {
    if (raw_w > raw_h)
    {
      out_w = MAX_RES;
      out_h = static_cast<int>(MAX_RES / aspect);
    }
    else
    {
      out_h = MAX_RES;
      out_w = static_cast<int>(MAX_RES * aspect);
    }
  }
  else
  {
    out_w = raw_w;
    out_h = raw_h;
  }

  if (out_w <= 0 || out_h <= 0)
    return false;

  out_data.resize(out_w * out_h);

  // Read and resample
  // For simplicity, nearest neighbor or simple average

  std::lock_guard<std::mutex> lock(m_file_mutex);
  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
    return false;

  // Optimization: Read scanlines needed
  // We need to iterate over output pixels and map to input

  // Cache check is hard here as we might skip rows.
  // Better to read line by line of source if we can, or just read needed lines.
  // If we are downsampling heavily, we skip lines.

  // Allocate buffer for one scanline (max width)
  float *buf = (float *)_TIFFmalloc(TIFFScanlineSize(tif));
  if (!buf)
  {
    TIFFClose(tif);
    return false;
  }

  // Precompute Y mapping to minimize seek
  std::vector<int> source_rows(out_h);
  for (int oy = 0; oy < out_h; ++oy)
  {
    double fy = (double)oy / out_h;
    source_rows[oy] = min_y + static_cast<int>(fy * raw_h);
  }

  int current_row = -1;

  float min_val = 1e10f, max_val = -1e10f;

  for (int oy = 0; oy < out_h; ++oy)
  {
    int sy = source_rows[oy];
    if (sy != current_row)
    {
      if (TIFFReadScanline(tif, buf, sy) < 0)
      {
        // Error or end
        break;
      }
      current_row = sy;
    }

    for (int ox = 0; ox < out_w; ++ox)
    {
      double fx = (double)ox / out_w;
      int sx = min_x + static_cast<int>(fx * raw_w);

      if (sx >= 0 && sx < m_width)
      {
        float val = buf[sx];
        // Stats for normalization
        if (val > -1000.0f)
        {
          min_val = std::min(min_val, val);
          max_val = std::max(max_val, val);
        }
        out_data[oy * out_w + ox] = val;
      }
      else
      {
        out_data[oy * out_w + ox] = -1.0f; // invalid
      }
    }
  }

  _TIFFfree(buf);
  TIFFClose(tif);

  // Normalize
  if (max_val > min_val)
  {
    for (auto &v : out_data)
    {
      if (v > -1000.0f)
        v = (v - min_val) / (max_val - min_val);
      else
        v = -1.0f;
    }
  }
  else
  {
    // Flat or empty
    std::fill(out_data.begin(), out_data.end(), 0.0f);
  }

  return true;
}

} // namespace sensor_mapper
