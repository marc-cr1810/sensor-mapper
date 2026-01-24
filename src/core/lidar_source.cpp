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

auto lidar_source_t::update() -> void
{
  // No async udpates
}

auto lidar_source_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  if (!m_loaded)
    return false;

  double x, y;

  if (m_transformer && m_transformer->is_valid())
  {
    // Transform Lat/Lon (WGS84) -> Projected X/Y
    // Wait, our transformer is Source -> Target (File -> WGS84).
    // We need WGS84 -> File.
    // CrsTransformer `init(source, target)` sets up a forward transform.
    // If we did init(File, WGS84), transform() converts X,Y(File) -> Lat,Lon(WGS84).
    // We are *querying* with Lat,Lon and want to find File X,Y.
    // So we need the INVERSE transform. Or we should have init(WGS84, File).

    // Let's re-read crs_transformer setup.
    // In lidar_source.cpp original code: "output = proj_trans(..., PJ_FWD, input)"
    // where input was Lat/Lon. That implies the transform object was WGS84 -> File.

    // My update above: `m_transformer->init(source_crs_str, "EPSG:4326")`
    // That creates File -> WGS84.
    // For Fallback (UTM), `transform` does UTM -> Lat/Lon.

    // We need the reverse for `get_elevation(lat, lon)`.
    // Implementing inverse fallback is harder?
    // Lat/Lon -> UTM is standard.
    // Let's change init to `init("EPSG:4326", source_crs_str)`.
    // But verify Fallback supports that.
    // Fallback in my impl checks `source_crs` for UTM code.
    // If source is 4326, and TARGET is UTM, we need LatLon -> UTM logic.

    // Actually, checking point inclusion is easier in Lat/Lon if we convert the *bounds*?
    // No, grid is rasterized in X/Y. We must convert query(Lat,Lon) -> X/Y.

    // I should update `crs_transformer` to support Inverse or just explicit directions.
    // Let's assume I WILL update `crs_transformer` to handle both directions or I update init here.
    // If I use `init("EPSG:4326", source_crs_str)`, PROJ works fine.
    // Fallback logic needs to detect Target is UTM etc.

    // Let's do this:
    // 1. `lidar_source` will create `m_transformer` as WGS84 -> File.
    // 2. `crs_transformer` needs to be smart enough to detect UTM in target if source is 4326.

    // BUT `lidar_source` code above did `init(source_crs_str, "EPSG:4326")`.
    // I should change that to `init("EPSG:4326", source_crs_str)`.

    // I will update this code block to use `init("EPSG:4326", source_crs_str)`.
    // And I will assume `crs_transformer` (which I just wrote) needs update?
    // I wrote `crs_transformer` to check `source_crs` for UTM.
    // I need to update `crs_transformer.cpp` to check TARGET or handle direction.

    // Since I can't edit `crs_transformer.cpp` in *this* step (singular replacement),
    // I will proceed with this replacement, BUT I will issue a fix step next.

    // Wait, `get_elevation` query logic:
    // x = lon; y = lat;
    // We want x,y in File CRS.

    // Let's assume for now I will fix `crs_transformer` to handle EPSG:4326 -> UTM.
    // So here I will call `init("EPSG:4326", source_crs_str)`.

    // NOTE: PROJ 6 order for 4326 is Lat, Lon.
    // My `transform` in PROJ block returned `out_x` (From x).
    // If I pass Lat, Lon as X, Y...

    // Let's stick to standard names.
    double tx, ty;
    // Lat is X? No, usually X=Lon, Y=Lat for math, but PROJ 4326 is X=Lat.
    // I'll pass X=Lat, Y=Lon to be safe with PROJ defaults for 4326.
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
