#pragma once

#include <memory>
#include <string>
#include <vector>

namespace sensor_mapper
{

// Building material types for attenuation calculation
enum class material_type_e
{
  UNKNOWN,
  CONCRETE,
  BRICK,
  WOOD,
  GLASS,
  METAL
};

// Geographic point
struct geo_point_t
{
  double lat;
  double lon;
};

// Building structure
struct building_t
{
  std::vector<geo_point_t> footprint; // Polygon vertices (lat/lon)
  double height_m;                    // Building height in meters
  material_type_e material;
  std::string id; // OSM building ID
  std::string name;
  std::string address;
};

// Building intersection result
struct building_intersection_t
{
  building_t *building;
  geo_point_t entry_point;
  geo_point_t exit_point;
  double distance_through_m;
  double incident_angle_deg; // Angle of ray vs building wall
};

class building_service_t
{
public:
  building_service_t();
  ~building_service_t();

  // Fetch buildings in bounding box (async)
  auto fetch_buildings(double min_lat, double max_lat, double min_lon, double max_lon) -> void;

  // Check if buildings are loaded for area
  auto has_buildings_for_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> bool;

  // Get all buildings within a bounding box
  auto get_buildings_in_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> std::vector<const building_t *>;

  // Get buildings intersecting a ray path
  auto get_buildings_on_path(geo_point_t start, geo_point_t end) const -> std::vector<building_intersection_t>;

  // Get building at specific location (for sensor placement)
  // Returns nullptr if no building found
  auto get_building_at(double lat, double lon) const -> const building_t *;

  // Check if we need to fetch data for this location
  auto is_area_loaded(double lat, double lon) const -> bool;

  // Update async operations. Returns true if new data was loaded.
  auto update() -> bool;

  // Clear cache
  auto clear() -> void;

  // Get loading status {active_fetches, queued_fetches}
  auto get_loading_status() const -> std::pair<int, int>;

private:
  struct impl_t;
  std::unique_ptr<impl_t> m_impl;
};

} // namespace sensor_mapper
