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
  load();
}

lidar_source_t::~lidar_source_t()
{
#ifdef HAVE_PROJ
  if (m_transformation)
  {
    proj_destroy(static_cast<PJ *>(m_transformation));
  }
  if (m_proj_ctx)
  {
    proj_context_destroy(static_cast<PJ_CONTEXT *>(m_proj_ctx));
  }
#endif
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
  // cell_size = 1.0 meter (could be configurable or inferred)
  m_cell_size = 1.0;

  double w_meters = m_max_x - m_min_x;
  double h_meters = m_max_y - m_min_y;

  m_width = static_cast<int>(std::ceil(w_meters / m_cell_size));
  m_height = static_cast<int>(std::ceil(h_meters / m_cell_size));

  if (m_width <= 0 || m_height <= 0)
  {
    lasreader->close();
    delete lasreader;
    return false;
  }

  // Allocate grid, init with lowest possible Z
  m_data.resize(m_width * m_height, -std::numeric_limits<float>::infinity());

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

#ifdef HAVE_PROJ
  // Initialize PROJ for coordinate transformations
  m_proj_ctx = proj_context_create();

  // Try to determine the CRS from the LAS header
  std::string source_crs_str;
  const char *source_crs = nullptr;

  // 1. Check for OGC WKT (LAS 1.4+)
  if (lasreader->header.vlr_geo_ogc_wkt != nullptr)
  {
    source_crs = lasreader->header.vlr_geo_ogc_wkt;
    std::cout << "LIDAR: Found OGC WKT in header" << std::endl;
  }
  // 2. Check for GeoKeys (LAS < 1.4 or backward compat)
  else if (lasreader->header.vlr_geo_keys && lasreader->header.vlr_geo_key_entries)
  {
    for (int i = 0; i < lasreader->header.vlr_geo_keys->number_of_keys; i++)
    {
      const auto &key = lasreader->header.vlr_geo_key_entries[i];
      // 3072 = ProjectedCSTypeGeoKey
      if (key.key_id == 3072)
      {
        if (key.tiff_tag_location == 0 && key.count == 1)
        {
          source_crs_str = "EPSG:" + std::to_string(key.value_offset);
          source_crs = source_crs_str.c_str();
          std::cout << "LIDAR: Found ProjectedCSTypeGeoKey: " << source_crs_str << std::endl;
          break;
        }
      }
    }
  }

  // Fallback: Check if coordinates look like they need projection but we found no CRS
  if (!source_crs && (m_max_x > 180.0 || m_min_x < -180.0 || m_max_y > 90.0 || m_min_y < -90.0))
  {
    std::cerr << "LIDAR: Warning: Coordinates appear to be projected (not lat/lon) but no CRS found in header." << std::endl;
    std::cerr << "LIDAR: Use las2las to add CRS info, e.g. 'las2las -i input.las -epsg 28355 -o output.las'" << std::endl;
  }

  if (source_crs)
  {
    PJ *transform = proj_create_crs_to_crs(static_cast<PJ_CONTEXT *>(m_proj_ctx),
                                           source_crs,  // File's CRS
                                           "EPSG:4326", // WGS84 (lat/lon)
                                           nullptr);

    if (transform)
    {
      // We want to transform FROM source TO WGS84, but proj_trans with PJ_FWD does Source -> Target
      // Our member is m_transformation.
      // Wait, let's double check usage in get_elevation.
      // get_elevation calls proj_trans(..., PJ_FWD, input) where input is WGS84 (lat/lon).
      // So we need a transformer from WGS84 -> Source (inverse of what we just created or swap args)

      // Let's create WGS84 -> Source
      m_transformation = proj_create_crs_to_crs(static_cast<PJ_CONTEXT *>(m_proj_ctx),
                                                "EPSG:4326", // Source: WGS84
                                                source_crs,  // Target: File CRS
                                                nullptr);
      if (m_transformation)
      {
        m_has_valid_crs = true;
        std::cout << "LIDAR: Created coordinate transformation (WGS84 -> " << (source_crs ? source_crs : "unknown") << ")" << std::endl;
      }
      else
      {
        std::cerr << "LIDAR: Failed to create PROJ transformation." << std::endl;
      }
    }
    else
    {
      std::cerr << "LIDAR: Failed to parse source CRS: " << source_crs << std::endl;
    }
  }
  else
  {
    // Lat/Lon or unknown
    m_has_valid_crs = false;
    if (m_max_x <= 180.0 && m_min_x >= -180.0 && m_max_y <= 90.0 && m_min_y >= -90.0)
    {
      std::cout << "LIDAR: File appears to be in lat/lon coordinates" << std::endl;
    }
  }
#endif

  m_loaded = true;
  std::cout << "Loaded LIDAR: " << m_path << " (" << m_width << "x" << m_height << ")" << std::endl;
  return true;
}

auto lidar_source_t::update() -> void
{
  // No async udpates
}

auto lidar_source_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  if (!m_loaded)
    return false;

  double x, y;

#ifdef HAVE_PROJ
  // If we have a valid transformation, use it
  if (m_has_valid_crs && m_transformation)
  {
    // Transform from WGS84 (lat/lon) to file's CRS
    PJ_COORD input, output;
    input.lpzt.lam = lon; // Longitude
    input.lpzt.phi = lat; // Latitude
    input.lpzt.z = 0;
    input.lpzt.t = 0;

    output = proj_trans(static_cast<PJ *>(m_transformation), PJ_FWD, input);

    // Check if transformation failed
    if (output.xy.x == HUGE_VAL || output.xy.y == HUGE_VAL)
    {
      return false;
    }

    x = output.xy.x;
    y = output.xy.y;
  }
  else
  {
    // No transformation, assume file is in lat/lon
    x = lon;
    y = lat;
  }
#else
  // Without PROJ, only support lat/lon files
  // Simple heuristic check
  bool is_lat_lon = (m_min_x >= -180.0 && m_max_x <= 180.0 && m_min_y >= -90.0 && m_max_y <= 90.0);

  if (!is_lat_lon)
  {
    // File is in projected coordinates but we can't transform
    return false;
  }

  x = lon;
  y = lat;
#endif

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
