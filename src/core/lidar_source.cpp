#include "core/lidar_source.hpp"
#include "lasreader.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

#ifdef HAVE_PROJ
#include <proj.h>
#endif

namespace sensor_mapper
{

lidar_source_t::lidar_source_t(const std::filesystem::path &path) : m_path(path)
{
  // Launch load asynchronously
  m_load_future = std::async(std::launch::async, [this]() { return load(); });
  m_loading_started = true;
}

lidar_source_t::~lidar_source_t()
{
  // If we are destroying while loading, we should wait or detach?
  // Ideally we wait to avoid using 'this' after destruction.
  if (m_loading_started && m_load_future.valid())
  {
    m_load_future.wait();
  }

#ifdef HAVE_PROJ
  if (m_transformer)
  {
    // m_transformer cleans up itself
  }
#endif
}

auto lidar_source_t::update() -> void
{
  if (!m_loaded && m_loading_started && m_load_future.valid())
  {
    // Check if ready (non-blocking)
    if (m_load_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      bool success = m_load_future.get();
      if (success)
      {
        m_loaded = true;
      }
      else
      {
        // Failed to load
        std::cerr << "LIDAR: Async load failed for " << m_path << std::endl;
        // Mark as not loading/invalid, so we don't check again
        m_loading_started = false;
      }
    }
  }
}

auto lidar_source_t::load() -> bool
{
  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(m_path.string().c_str());

  LASreader *lasreader = lasreadopener.open();
  if (!lasreader)
  {
    std::cerr << "Failed to open LAS file: " << m_path << std::endl;
    return false;
  }

  // Get Bounds
  m_min_x = lasreader->header.min_x;
  m_min_y = lasreader->header.min_y;
  m_max_x = lasreader->header.max_x;
  m_max_y = lasreader->header.max_y;

  // Determine grid size
  // cell_size = 1.0 meter (initial guess)
  m_cell_size = 1.0;

  double w_meters = m_max_x - m_min_x;
  double h_meters = m_max_y - m_min_y;

  // Safety Check: Avoid massive allocations if points are garbage or too spread out
  if (w_meters > 1000000.0 || h_meters > 1000000.0)
  {
    std::cerr << "LIDAR: Extents too large (" << w_meters << "x" << h_meters << "m). File might be corrupt or huge." << std::endl;
    lasreader->close();
    delete lasreader;
    return false;
  }

  // Calculate required dimensions
  unsigned long long req_w = static_cast<unsigned long long>(std::ceil(w_meters / m_cell_size));
  unsigned long long req_h = static_cast<unsigned long long>(std::ceil(h_meters / m_cell_size));
  unsigned long long total_pixels = req_w * req_h;

  // Limit to ~256MB (approx 64 million floats)
  const unsigned long long MAX_PIXELS = 64 * 1024 * 1024;

  if (total_pixels > MAX_PIXELS)
  {
    // Auto-scale up cell size to fit
    // new_total = (w/s) * (h/s) = total / s^2
    // s^2 = total / MAX
    // s = sqrt(total / MAX)
    double scale_factor = std::sqrt(static_cast<double>(total_pixels) / static_cast<double>(MAX_PIXELS));
    m_cell_size *= (scale_factor * 1.05); // Add 5% buffer

    std::cout << "LIDAR: Grid too large, scaling cell_size to " << m_cell_size << "m" << std::endl;
  }

  m_width = static_cast<int>(std::ceil(w_meters / m_cell_size));
  m_height = static_cast<int>(std::ceil(h_meters / m_cell_size));

  if (m_width <= 0 || m_height <= 0)
  {
    lasreader->close();
    delete lasreader;
    return false;
  }

  // Allocate grid, init with lowest possible Z
  // Use try-catch for allocation
  try
  {
    m_data.resize(m_width * m_height, -std::numeric_limits<float>::infinity());
  }
  catch (const std::bad_alloc &e)
  {
    std::cerr << "LIDAR: Memory allocation failed: " << e.what() << std::endl;
    lasreader->close();
    delete lasreader;
    return false;
  }

  // Iterate points
  while (lasreader->read_point())
  {
    double x = lasreader->point.get_x();
    double y = lasreader->point.get_y();
    double z = lasreader->point.get_z();

    // Map to grid
    int gx = static_cast<int>((x - m_min_x) / m_cell_size);
    int gy = static_cast<int>((m_max_y - y) / m_cell_size); // Flip Y so 0 is top

    if (gx >= 0 && gx < m_width && gy >= 0 && gy < m_height)
    {
      // DSM: Keep max Z
      float current = m_data[gy * m_width + gx];
      if (z > current)
      {
        m_data[gy * m_width + gx] = static_cast<float>(z);
      }
    }
  }

  lasreader->close();
  delete lasreader;

  lasreader->close();
  delete lasreader;

  // Initialize Transformer logic (PROJ or Fallback)
  m_transformer = std::make_unique<crs_transformer_t>();

  // 1. Try OGC WKT
  std::string source_crs_str;
  if (lasreader->header.vlr_geo_ogc_wkt != nullptr)
  {
    source_crs_str = lasreader->header.vlr_geo_ogc_wkt;
    std::cout << "LIDAR: Found OGC WKT." << std::endl;
  }
  // 2. Try GeoKeys (EPSG)
  else if (lasreader->header.vlr_geo_keys && lasreader->header.vlr_geo_key_entries)
  {
    for (int i = 0; i < lasreader->header.vlr_geo_keys->number_of_keys; i++)
    {
      const auto &key = lasreader->header.vlr_geo_key_entries[i];
      if (key.key_id == 3072)
      { // ProjectedCSTypeGeoKey
        if (key.tiff_tag_location == 0 && key.count == 1)
        {
          source_crs_str = "EPSG:" + std::to_string(key.value_offset);
          std::cout << "LIDAR: Found EPSG Code: " << source_crs_str << std::endl;
          break;
        }
      }
    }
  }

  // Fallback: Infer from coordinates? (Not reliable for exact zone, but maybe user provides sidecar?)
  // For now, if string empty, we assume Lat/Lon or fail.

  if (!source_crs_str.empty())
  {
    if (m_transformer->init(source_crs_str, "EPSG:4326"))
    {
      std::cout << "LIDAR: CRS Transformer initialized." << std::endl;
    }
    else
    {
      std::cerr << "LIDAR: Failed to initialize CRS Transformer for " << source_crs_str << std::endl;
    }
  }

  m_loaded = true;
  std::cout << "Loaded LIDAR: " << m_path << " (" << m_width << "x" << m_height << ")" << std::endl;
  return true;
}

auto lidar_source_t::get_bounds(double &min_lat, double &max_lat, double &min_lon, double &max_lon) const -> bool
{
  if (!m_loaded)
    return false;

  if (m_transformer && m_transformer->is_valid())
  {
    double pts_x[4] = {m_min_x, m_max_x, m_max_x, m_min_x};
    double pts_y[4] = {m_min_y, m_min_y, m_max_y, m_max_y};

    min_lat = 90.0;
    max_lat = -90.0;
    min_lon = 180.0;
    max_lon = -180.0;

    bool any_valid = false;
    for (int i = 0; i < 4; ++i)
    {
      double lat, lon;
      // Transformer converts Source (Projected) -> WGS84 (Lat/Lon)
      // Note: crs_transformer output order: out_x=Lat, out_y=Lon (based on our analysis of crs_transformer.cpp)
      if (m_transformer->transform(pts_x[i], pts_y[i], lat, lon))
      {
        if (lat < min_lat)
          min_lat = lat;
        if (lat > max_lat)
          max_lat = lat;
        if (lon < min_lon)
          min_lon = lon;
        if (lon > max_lon)
          max_lon = lon;
        any_valid = true;
      }
    }
    return any_valid;
  }
  else
  {
    // Fallback: Assume LAS is already WGS84 (X=Lon, Y=Lat)
    min_lon = m_min_x;
    max_lon = m_max_x;
    min_lat = m_min_y;
    max_lat = m_max_y;
    return true;
  }
}

auto lidar_source_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  if (!m_loaded)
    return false;

  double x, y;

  if (m_transformer && m_transformer->is_valid())
  {
    // Transform Lat/Lon (WGS84) -> Projected X/Y
    // Note: We initialize the transformer with "EPSG:4326" -> source_crs_str (File CRS).
    // The transform() method will thus convert input Lat/Lon to the file's projected X/Y coordinates.
    // Note on Axis Order: For EPSG:4326 in PROJ 6+, the order is often Lat, Lon (X=Lat).
    // We pass lat, lon accordingly.
    double tx, ty;
    // Attempt transform
    if (m_transformer->transform(lat, lon, tx, ty))
    {
      x = tx;
      y = ty;
    }
    else
    {
      return false;
    }
  }
  else
  {
    // Assume already Lat/Lon
    x = lon;
    y = lat;
  }

  // Look up in rasterized grid
  double px = (x - m_min_x) / m_cell_size;
  double py = (m_max_y - y) / m_cell_size;

  int gx = static_cast<int>(std::floor(px));
  int gy = static_cast<int>(std::floor(py));

  if (gx >= 0 && gx < m_width && gy >= 0 && gy < m_height)
  {
    float val = m_data[gy * m_width + gx];
    if (val > -100000.0f) // Valid check
    {
      out_height = val;
      return true;
    }
  }

  return false;
}

} // namespace sensor_mapper
