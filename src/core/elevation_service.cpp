#include "core/elevation_service.hpp"
#include "core/terrarium_source.hpp"
#include "core/geotiff_source.hpp"
#include "core/lidar_source.hpp"
#include "core/geo_math.hpp"
#include <algorithm>
#include <filesystem>

namespace sensor_mapper
{

elevation_service_t::elevation_service_t()
{
  // Add default online source
  add_source(std::make_shared<terrarium_source_t>());
}

elevation_service_t::~elevation_service_t()
{
}

auto elevation_service_t::add_source(std::shared_ptr<elevation_source_t> source) -> void
{
  m_sources.push_back(source);
}

auto elevation_service_t::load_local_file(const std::string &path) -> bool
{
  // Check extension to decide source type?
  // For now assume .tif/.tiff is GeoTIFF

  std::filesystem::path p(path);
  auto ext = p.extension().string();

  // Basic case-insensitive check (simplified)
  if (ext == ".tif" || ext == ".tiff" || ext == ".TIF" || ext == ".TIFF")
  {
    auto source = std::make_shared<geotiff_source_t>(p);
    // Simple check if loaded? getter?
    // We assume if it failed inside, it's just an empty source.
    // Ideally geotiff_source_t should expose is_valid()
    add_source(source);
    return true;
  }
  else if (ext == ".las" || ext == ".laz" || ext == ".LAS" || ext == ".LAZ")
  {
    auto source = std::make_shared<lidar_source_t>(p);
    add_source(source);
    return true;
  }

  return false;
}

auto elevation_service_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  // Iterate sources (LIFO for priority)
  for (auto it = m_sources.rbegin(); it != m_sources.rend(); ++it)
  {
    if ((*it)->get_elevation(lat, lon, out_height))
    {
      return true;
    }
  }
  return false;
}

auto elevation_service_t::get_profile(double start_lat, double start_lon, double end_lat, double end_lon, int num_samples) -> std::vector<std::pair<double, float>>
{
  std::vector<std::pair<double, float>> profile;
  profile.reserve(num_samples);

  double total_dist = geo::distance(start_lat, start_lon, end_lat, end_lon);

  for (int i = 0; i < num_samples; ++i)
  {
    double t = (double)i / (num_samples - 1);
    double lat, lon;
    geo::interpolate(start_lat, start_lon, end_lat, end_lon, t, lat, lon);

    float ele = 0.0f;
    if (!get_elevation(lat, lon, ele))
    {
      ele = 0.0f; // Default if no data
    }

    profile.push_back({total_dist * t, ele});
  }
  return profile;
}

auto elevation_service_t::update() -> void

{
  for (auto &source : m_sources)
  {
    source->update();
  }
}

} // namespace sensor_mapper
