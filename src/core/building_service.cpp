#include "building_service.hpp"
#include <algorithm>
#include <cmath>
#include <cpr/cpr.h>
#include <format>
#include <future>
#include <iostream>
#include <map>

#include <nlohmann/json.hpp>

namespace sensor_mapper
{

// Constants for spatial grid
constexpr double GRID_CELL_SIZE = 0.002; // Roughly 200m at equator

using json = nlohmann::json;

// Pimpl implementation
struct building_service_t::impl_t
{
  std::vector<building_t> buildings;
  std::map<std::string, bool> loaded_areas; // Track loaded bounding boxes

  // Spatial Index: Grid Key (lat_idx, lon_idx) -> List of building indices
  using grid_key_t = std::pair<int, int>;
  std::map<grid_key_t, std::vector<size_t>> spatial_grid;

  struct pending_fetch_t
  {
    std::string area_key;
    std::future<std::string> data_future;
  };
  std::vector<pending_fetch_t> pending_fetches;

  // Helper: Create area key from bbox
  auto make_area_key(double min_lat, double max_lat, double min_lon, double max_lon) const -> std::string
  {
    return std::format("{:.4f},{:.4f},{:.4f},{:.4f}", min_lat, max_lat, min_lon, max_lon);
  }

  // Helper: Calculate grid key for a coordinate
  auto get_grid_key(double lat, double lon) const -> grid_key_t
  {
    return {static_cast<int>(std::floor(lat / GRID_CELL_SIZE)), static_cast<int>(std::floor(lon / GRID_CELL_SIZE))};
  }

  // Helper: Add building to spatial index
  auto add_to_spatial_index(size_t index) -> void
  {
    const auto &b = buildings[index];
    if (b.footprint.empty())
      return;

    // Simple approach: Add to key of the first point
    auto key = get_grid_key(b.footprint[0].lat, b.footprint[0].lon);
    spatial_grid[key].push_back(index);

    // Note: For large buildings, we should add to multiple cells,
    // but fast lookup is prioritized here.
  }

  auto get_candidates(double lat, double lon) const -> const std::vector<size_t> *
  {
    auto key = get_grid_key(lat, lon);
    auto it = spatial_grid.find(key);
    if (it != spatial_grid.end())
    {
      return &it->second;
    }
    return nullptr;
  }

  // Helper: Parse OSM JSON response
  auto parse_osm_buildings_json(const std::string &json_data) -> std::vector<building_t>
  {
    std::vector<building_t> result;

    try
    {
      auto j = json::parse(json_data);
      std::map<int64_t, geo_point_t> nodes;

      if (!j.contains("elements"))
        return result;

      // First pass: Collect nodes
      for (const auto &el : j["elements"])
      {
        if (el["type"] == "node")
        {
          nodes[el["id"]] = {el["lat"], el["lon"]};
        }
      }

      // Second pass: Collect ways (buildings)
      for (const auto &el : j["elements"])
      {
        if (el["type"] == "way" && el.contains("tags") && el["tags"].contains("building"))
        {
          building_t building;
          building.id = std::to_string(el["id"].get<int64_t>());

          // Defaults
          building.material = material_type_e::CONCRETE;
          building.height_m = 10.0;

          // Parse tags
          const auto &tags = el["tags"];

          if (tags.contains("height"))
          {
            try
            {
              std::string h_str = tags["height"];
              // Remove " m" if present
              auto pos = h_str.find(" ");
              if (pos != std::string::npos)
                h_str = h_str.substr(0, pos);
              building.height_m = std::stod(h_str);
            }
            catch (...)
            {
            }
          }
          else if (tags.contains("building:levels"))
          {
            try
            {
              double levels = std::stod(tags["building:levels"].get<std::string>());
              building.height_m = levels * 3.0;
            }
            catch (...)
            {
            }
          }

          // Construct footprint
          if (el.contains("nodes"))
          {
            for (const auto &node_id : el["nodes"])
            {
              auto it = nodes.find(node_id);
              if (it != nodes.end())
              {
                building.footprint.push_back(it->second);
              }
            }
          }

          if (building.footprint.size() >= 3)
          {
            result.push_back(building);
          }
        }
      }
    }
    catch (const std::exception &e)
    {
      std::cerr << "JSON Parsing Error: " << e.what() << std::endl;
    }

    return result;
  }

  // Helper: Check if point is inside polygon (ray-casting algorithm)
  auto point_in_polygon(const geo_point_t &point, const std::vector<geo_point_t> &polygon) const -> bool
  {
    int crossings = 0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i)
    {
      size_t j = (i + 1) % n;

      if (((polygon[i].lat <= point.lat && point.lat < polygon[j].lat) || (polygon[j].lat <= point.lat && point.lat < polygon[i].lat)) &&
          (point.lon < (polygon[j].lon - polygon[i].lon) * (point.lat - polygon[i].lat) / (polygon[j].lat - polygon[i].lat) + polygon[i].lon))
      {
        crossings++;
      }
    }

    return (crossings % 2) == 1;
  }
};

building_service_t::building_service_t() : m_impl(std::make_unique<impl_t>())
{
}

building_service_t::~building_service_t() = default;

auto building_service_t::fetch_buildings(double min_lat, double max_lat, double min_lon, double max_lon) -> void
{
  auto area_key = m_impl->make_area_key(min_lat, max_lat, min_lon, max_lon);

  // Check if already loaded or loading
  if (m_impl->loaded_areas.count(area_key))
  {
    return;
  }

  // Check if already fetching
  for (const auto &fetch : m_impl->pending_fetches)
  {
    if (fetch.area_key == area_key)
    {
      return;
    }
  }

  // Mark as being loaded
  m_impl->loaded_areas[area_key] = false;

  // Build Overpass API query (JSON mode)
  std::string query = std::format("[out:json];"
                                  "("
                                  "  way[\"building\"]({:.5f},{:.5f},{:.5f},{:.5f});"
                                  "  relation[\"building\"]({:.5f},{:.5f},{:.5f},{:.5f});"
                                  ");"
                                  "out body;>;out skel qt;",
                                  min_lat, min_lon, max_lat, max_lon, min_lat, min_lon, max_lat, max_lon);

  std::cout << "OSM Query: " << query << std::endl;

  std::string url = "https://overpass-api.de/api/interpreter";

  // Start async fetch
  m_impl->pending_fetches.push_back({area_key, std::async(std::launch::async,
                                                          [url, query]()
                                                          {
                                                            cpr::Response r = cpr::Post(cpr::Url{url}, cpr::Body{query}, cpr::Header{{"User-Agent", "SensorMapper/0.1"}});

                                                            if (r.status_code == 200)
                                                            {
                                                              return r.text;
                                                            }
                                                            std::cout << "OSM API error: " << r.status_code << std::endl;
                                                            return std::string();
                                                          })});

  std::cout << "Fetching buildings for area: " << area_key << std::endl;
}

auto building_service_t::has_buildings_for_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> bool
{
  auto area_key = m_impl->make_area_key(min_lat, max_lat, min_lon, max_lon);
  auto it = m_impl->loaded_areas.find(area_key);
  return it != m_impl->loaded_areas.end() && it->second;
}

auto building_service_t::get_buildings_in_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> std::vector<const building_t *>
{
  std::vector<const building_t *> result;

  // Determine grid cells covering the area
  auto start_key = m_impl->get_grid_key(min_lat, min_lon);
  auto end_key = m_impl->get_grid_key(max_lat, max_lon);

  for (int lat_idx = start_key.first; lat_idx <= end_key.first; ++lat_idx)
  {
    for (int lon_idx = start_key.second; lon_idx <= end_key.second; ++lon_idx)
    {
      auto it = m_impl->spatial_grid.find({lat_idx, lon_idx});
      if (it == m_impl->spatial_grid.end())
        continue;

      for (size_t idx : it->second)
      {
        const auto &building = m_impl->buildings[idx];
        // Bounding box check
        if (building.footprint.empty())
          continue;

        const auto &pt = building.footprint[0];
        if (pt.lat >= min_lat && pt.lat <= max_lat && pt.lon >= min_lon && pt.lon <= max_lon)
        {
          result.push_back(&building);
        }
      }
    }
  }

  // Remove duplicates resulting from grid overlap
  std::sort(result.begin(), result.end());
  result.erase(std::unique(result.begin(), result.end()), result.end());

  return result;
}

auto building_service_t::is_area_loaded(double /*lat*/, double /*lon*/) const -> bool
{
  // Optimization: For now returns true if *any* building contains this point
  // or if we have processed a fetch nearby.
  return false;
}

auto building_service_t::get_building_at(double lat, double lon) const -> const building_t *
{
  geo_point_t point = {lat, lon};

  // 1. Get candidates from spatial index
  auto *candidates = m_impl->get_candidates(lat, lon);
  if (!candidates)
    return nullptr;

  // 2. Exact check
  for (size_t idx : *candidates)
  {
    const auto &building = m_impl->buildings[idx];
    if (m_impl->point_in_polygon(point, building.footprint))
    {
      return &building;
    }
  }
  return nullptr;
}

auto building_service_t::get_buildings_on_path(geo_point_t start, geo_point_t end) const -> std::vector<building_intersection_t>
{
  std::vector<building_intersection_t> result;

  // Simple haversine distance helper
  auto dist_m = [](geo_point_t p1, geo_point_t p2) -> double
  {
    double R = 6378137.0;
    double dLat = (p2.lat - p1.lat) * M_PI / 180.0;
    double dLon = (p2.lon - p1.lon) * M_PI / 180.0;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) + std::cos(p1.lat * M_PI / 180.0) * std::cos(p2.lat * M_PI / 180.0) * std::sin(dLon / 2) * std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
  };

  // Helper: Segment-Segment Intersection
  auto intersect = [](geo_point_t p1, geo_point_t p2, geo_point_t p3, geo_point_t p4, geo_point_t &out) -> bool
  {
    double x1 = p1.lon, y1 = p1.lat;
    double x2 = p2.lon, y2 = p2.lat;
    double x3 = p3.lon, y3 = p3.lat;
    double x4 = p4.lon, y4 = p4.lat;

    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (std::abs(denom) < 1e-9)
      return false;

    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
    {
      out.lon = x1 + ua * (x2 - x1);
      out.lat = y1 + ua * (y2 - y1);
      return true;
    }
    return false;
  };

  // Check each building
  // Optimization: Use spatial index traversal (Ray Traversal / Bresenham's line
  // algo on grid) For now, simpler: check start and end point cells? No, ray
  // can cross 10 cells. Fallback: Linear scan on ALL buildings (slow) or
  // better: Traversal. Given urgency, let's keep linear scan BUT restrict it?
  // We don't have a ray_traversal implemented in impl_t yet.
  // Let's implement a simple "Gather all relevant cells" check.

  // Quick hack: Bounding box of ray
  double min_lat = std::min(start.lat, end.lat);
  double max_lat = std::max(start.lat, end.lat);
  double min_lon = std::min(start.lon, end.lon);
  double max_lon = std::max(start.lon, end.lon);

  auto start_key = m_impl->get_grid_key(min_lat, min_lon);
  auto end_key = m_impl->get_grid_key(max_lat, max_lon);

  // Use a set to avoid duplicates
  std::vector<const building_t *> candidates;
  std::vector<size_t> candidate_indices;

  for (int lat_idx = start_key.first; lat_idx <= end_key.first; ++lat_idx)
  {
    for (int lon_idx = start_key.second; lon_idx <= end_key.second; ++lon_idx)
    {
      auto it = m_impl->spatial_grid.find({lat_idx, lon_idx});
      if (it != m_impl->spatial_grid.end())
      {
        candidate_indices.insert(candidate_indices.end(), it->second.begin(), it->second.end());
      }
    }
  }
  std::sort(candidate_indices.begin(), candidate_indices.end());
  candidate_indices.erase(std::unique(candidate_indices.begin(), candidate_indices.end()), candidate_indices.end());

  for (size_t idx : candidate_indices)
  {
    const auto &building = m_impl->buildings[idx];

    // Quick bbox check
    bool possible = false;
    for (const auto &pt : building.footprint)
    {
      // Loose check
      if ((pt.lat > min_lat - 0.0001 && pt.lat < max_lat + 0.0001 && pt.lon > min_lon - 0.0001 && pt.lon < max_lon + 0.0001))
      {
        possible = true;
        break;
      }
    }
    if (!possible)
    {
      // Double check start/end inside building
      if (!m_impl->point_in_polygon(start, building.footprint) && !m_impl->point_in_polygon(end, building.footprint))
        continue;
    }

    std::vector<geo_point_t> intersections;
    const auto &poly = building.footprint;

    for (size_t i = 0; i < poly.size(); ++i)
    {
      geo_point_t p1 = poly[i];
      geo_point_t p2 = poly[(i + 1) % poly.size()];
      geo_point_t ix;
      if (intersect(start, end, p1, p2, ix))
      {
        intersections.push_back(ix);
      }
    }

    // Sort intersections by distance from start
    std::sort(intersections.begin(), intersections.end(), [&](const geo_point_t &a, const geo_point_t &b) { return dist_m(start, a) < dist_m(start, b); });

    // Remove duplicates (corner cases)
    auto last = std::unique(intersections.begin(), intersections.end(),
                            [&](const geo_point_t &a, const geo_point_t &b)
                            {
                              return dist_m(a, b) < 0.1; // 10cm tolerance
                            });
    intersections.erase(last, intersections.end());

    if (intersections.size() >= 2)
    {
      // Take pairs
      for (size_t i = 0; i + 1 < intersections.size(); i += 2)
      {
        building_intersection_t bi;
        bi.building = const_cast<building_t *>(&building);
        bi.entry_point = intersections[i];
        bi.exit_point = intersections[i + 1];
        bi.distance_through_m = dist_m(bi.entry_point, bi.exit_point);
        bi.incident_angle_deg = 0.0; // TODO calculate angle
        result.push_back(bi);
      }
    }
  }

  return result;
}

auto building_service_t::update() -> bool
{
  auto it = m_impl->pending_fetches.begin();
  bool new_data = false;

  while (it != m_impl->pending_fetches.end())
  {
    if (it->data_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      std::string data = it->data_future.get();

      // Capture area key before erasing or using (safety)
      std::string current_key = it->area_key;

      if (!data.empty())
      {
        auto new_buildings = m_impl->parse_osm_buildings_json(data);

        size_t start_idx = m_impl->buildings.size();
        m_impl->buildings.insert(m_impl->buildings.end(), new_buildings.begin(), new_buildings.end());

        // Index the new buildings
        for (size_t i = 0; i < new_buildings.size(); ++i)
        {
          m_impl->add_to_spatial_index(start_idx + i);
        }

        std::cout << "Loaded " << new_buildings.size() << " buildings for area: " << current_key << std::endl;
        new_data = true;
      }

      // Mark area as loaded using the local key
      m_impl->loaded_areas[current_key] = true;

      it = m_impl->pending_fetches.erase(it);
    }
    else
    {
      ++it;
    }
  }
  return new_data;
}

auto building_service_t::clear() -> void
{
  m_impl->buildings.clear();
  m_impl->loaded_areas.clear();
  m_impl->pending_fetches.clear();
}

} // namespace sensor_mapper
