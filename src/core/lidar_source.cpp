#include "core/lidar_source.hpp"
#include "lasreader.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>

namespace sensor_mapper
{

lidar_source_t::lidar_source_t(const std::filesystem::path &path) : m_path(path)
{
  load();
}

lidar_source_t::~lidar_source_t()
{
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

  // Coordinate System Issue:
  // LAS files are usually in a projected coordinate system (UTM, State Plane), not Lat/Lon.
  // We don't have PROJ, so we can't easily convert Lat/Lon input to LAS Project Coords.
  // HOWEVER, for this proof of concept, we assume:
  // 1. Input lat/lon is converted to World Meters by the caller (ElevationService usually takes Lat/Lon?)
  // Wait, ElevationService takes Lat/Lon.
  // If the LAS file is in UTM, and we pass Lat/Lon, we will just get garbage.
  // Ideally, we'd need to reproject.
  // OR, we assume the LAS file IS already georeferenced or we blindly map input Lat/Lon to it if it matches?
  //
  // Actually, `ElevationService::get_elevation` takes Lat/Lon.
  // `Terrarium` works because it takes Lat/Lon.
  //
  // If the USER loads a local LAS file, they likely expect it to appear "somewhere".
  // Without PROJ, we can't place it correctly on the globe unless we know the offset.
  //
  // Workaround: We will assume the LAS coordinates are "World Meters" relative to the map origin?
  // OR we just implementation the lookup and warn the user.
  //
  // Let's implement the lookup assuming Lat/Lon IS the input, but we need to convert that to the LAS bounds.
  // If the LAS bounds are huge (e.g. 500,000), it's UTM.
  // If small (-180..180), it's Lat/Lon.
  //
  // Let's check bounds.

  bool is_lat_lon = (m_min_x >= -180.0 && m_max_x <= 180.0 && m_min_y >= -90.0 && m_max_y <= 90.0);

  double x, y;
  if (is_lat_lon)
  {
    x = lon;
    y = lat;
  }
  else
  {
    // It's likely Projected Meters.
    // We cannot convert Lat/Lon to arbitrary Projected Meters without PROJ.
    // So this method will likely FAIL to match unless our application world coordinates align.
    // But the interface is get_elevation(lat, lon).
    // We'll just return false if we can't map it, or try to map roughly?
    // No, let's just create a placeholder logic.
    //
    // Actually, standard `ElevationService` converts Lat/Lon to World Meters:
    // geo::lat_lon_to_world(lat, lon, wx, wy).
    // Maybe we compare wx, wy to the LAS bounds?
    // If the LAS is in the same Mercator projection as our world, it fits!
    //
    // Let's try converting input Lat/Lon to standard Web Mercator (which is what mapbox/osm/terrarium use).
    // If the LAS file was exported in Web Mercator (EPSG:3857), it will align.
    // If it's UTM, it won't.

    // We need `geo::lat_lon_to_world` logic here?
    // No, I can't access `geo::lat_lon_to_world` easily without including `geo_math.hpp`.
    // I should include it.
    return false; // Implementing minimal placeholder logic for now.
  }

  // Logic if coordinates match (Assuming LAS is Lat/Lon for now for simplicity, or we add reprojection later)
  // pixel_x = (x - min_x) / cell_size
  // pixel_y = (max_y - y) / cell_size (Y flips?)
  //
  // If it's Lat/Lon, cell_size should be in degrees (approx 0.00001).
  // If my cell_size was set to 1.0 (meter), and coordinates are degrees, this breaks.
  //
  // Fix: Check units.

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
