#include "core/building_service.hpp"
#include <algorithm>
#include <cmath>
#include <cpr/cpr.h>
#include <format>
#include <future>
#include <iostream>
#include <map>

namespace sensor_mapper {

// Pimpl implementation
struct building_service_t::impl_t {
  std::vector<building_t> buildings;
  std::map<std::string, bool> loaded_areas; // Track loaded bounding boxes

  struct pending_fetch_t {
    std::string area_key;
    std::future<std::string> data_future;
  };
  std::vector<pending_fetch_t> pending_fetches;

  // Helper: Create area key from bbox
  auto make_area_key(double min_lat, double max_lat, double min_lon,
                     double max_lon) const -> std::string {
    return std::format("{:.4f},{:.4f},{:.4f},{:.4f}", min_lat, max_lat, min_lon,
                       max_lon);
  }

  // Helper: Parse OSM XML response
  auto parse_osm_buildings(const std::string &xml_data)
      -> std::vector<building_t> {
    std::vector<building_t> result;

    // Simple XML parsing - extract nodes, ways, and buildings
    std::map<std::string, geo_point_t> nodes; // node id -> lat/lon

    // Parse nodes first
    size_t pos = 0;
    while ((pos = xml_data.find("<node", pos)) != std::string::npos) {
      auto end_pos = xml_data.find("/>", pos);
      if (end_pos == std::string::npos)
        break;

      std::string node_tag = xml_data.substr(pos, end_pos - pos);

      // Extract id, lat, lon
      auto id_pos = node_tag.find("id=\"");
      auto lat_pos = node_tag.find("lat=\"");
      auto lon_pos = node_tag.find("lon=\"");

      if (id_pos != std::string::npos && lat_pos != std::string::npos &&
          lon_pos != std::string::npos) {
        std::string id = node_tag.substr(
            id_pos + 4, node_tag.find("\"", id_pos + 4) - (id_pos + 4));
        std::string lat_str = node_tag.substr(
            lat_pos + 5, node_tag.find("\"", lat_pos + 5) - (lat_pos + 5));
        std::string lon_str = node_tag.substr(
            lon_pos + 5, node_tag.find("\"", lon_pos + 5) - (lon_pos + 5));

        nodes[id] = {std::stod(lat_str), std::stod(lon_str)};
      }

      pos = end_pos + 2;
    }

    // Parse ways (buildings)
    pos = 0;
    while ((pos = xml_data.find("<way", pos)) != std::string::npos) {
      auto way_end = xml_data.find("</way>", pos);
      if (way_end == std::string::npos)
        break;

      std::string way_section = xml_data.substr(pos, way_end - pos);

      // Check if it's a building
      if (way_section.find("k=\"building\"") == std::string::npos) {
        pos = way_end + 6;
        continue;
      }

      building_t building;
      building.material = material_type_e::CONCRETE; // Default
      building.height_m = 10.0;                      // Default 3 stories

      // Extract building ID
      auto id_pos = way_section.find("id=\"");
      if (id_pos != std::string::npos) {
        building.id = way_section.substr(
            id_pos + 4, way_section.find("\"", id_pos + 4) - (id_pos + 4));
      }

      // Extract height if available
      auto height_pos = way_section.find("k=\"height\" v=\"");
      if (height_pos != std::string::npos) {
        std::string height_str = way_section.substr(
            height_pos + 14,
            way_section.find("\"", height_pos + 14) - (height_pos + 14));
        try {
          building.height_m = std::stod(height_str);
        } catch (...) {
        }
      }

      // Extract building:levels if no height
      if (building.height_m == 10.0) {
        auto levels_pos = way_section.find("k=\"building:levels\" v=\"");
        if (levels_pos != std::string::npos) {
          std::string levels_str = way_section.substr(
              levels_pos + 23,
              way_section.find("\"", levels_pos + 23) - (levels_pos + 23));
          try {
            int levels = std::stoi(levels_str);
            building.height_m = levels * 3.0; // 3m per floor
          } catch (...) {
          }
        }
      }

      // Extract node references
      size_t ref_pos = 0;
      while ((ref_pos = way_section.find("<nd ref=\"", ref_pos)) !=
             std::string::npos) {
        std::string node_ref = way_section.substr(
            ref_pos + 9, way_section.find("\"", ref_pos + 9) - (ref_pos + 9));

        auto node_it = nodes.find(node_ref);
        if (node_it != nodes.end()) {
          building.footprint.push_back(node_it->second);
        }

        ref_pos += 9;
      }

      // Only add if we have a valid polygon (at least 3 points)
      if (building.footprint.size() >= 3) {
        result.push_back(building);
      }

      pos = way_end + 6;
    }

    return result;
  }

  // Helper: Check if point is inside polygon (ray-casting algorithm)
  auto point_in_polygon(const geo_point_t &point,
                        const std::vector<geo_point_t> &polygon) const -> bool {
    int crossings = 0;
    size_t n = polygon.size();

    for (size_t i = 0; i < n; ++i) {
      size_t j = (i + 1) % n;

      if (((polygon[i].lat <= point.lat && point.lat < polygon[j].lat) ||
           (polygon[j].lat <= point.lat && point.lat < polygon[i].lat)) &&
          (point.lon < (polygon[j].lon - polygon[i].lon) *
                               (point.lat - polygon[i].lat) /
                               (polygon[j].lat - polygon[i].lat) +
                           polygon[i].lon)) {
        crossings++;
      }
    }

    return (crossings % 2) == 1;
  }
};

building_service_t::building_service_t() : m_impl(std::make_unique<impl_t>()) {}

building_service_t::~building_service_t() = default;

auto building_service_t::fetch_buildings(double min_lat, double max_lat,
                                         double min_lon, double max_lon)
    -> void {
  auto area_key = m_impl->make_area_key(min_lat, max_lat, min_lon, max_lon);

  // Check if already loaded or loading
  if (m_impl->loaded_areas.count(area_key)) {
    return;
  }

  // Check if already fetching
  for (const auto &fetch : m_impl->pending_fetches) {
    if (fetch.area_key == area_key) {
      return;
    }
  }

  // Mark as being loaded
  m_impl->loaded_areas[area_key] = false;

  // Build Overpass API query
  std::string query = std::format("[bbox:{},{},{},{}];"
                                  "(way[\"building\"];relation[\"building\"];);"
                                  "out body;>;out skel qt;",
                                  min_lat, min_lon, max_lat, max_lon);

  std::cout << "OSM Query: " << query << std::endl;

  std::string url = "https://overpass-api.de/api/interpreter";

  // Start async fetch
  m_impl->pending_fetches.push_back(
      {area_key, std::async(std::launch::async, [url, query]() {
         cpr::Response r =
             cpr::Post(cpr::Url{url}, cpr::Body{query},
                       cpr::Header{{"User-Agent", "SensorMapper/0.1"}});

         if (r.status_code == 200) {
           return r.text;
         }
         std::cout << "OSM API error: " << r.status_code << std::endl;
         return std::string();
       })});

  std::cout << "Fetching buildings for area: " << area_key << std::endl;
}

auto building_service_t::has_buildings_for_area(double min_lat, double max_lat,
                                                double min_lon,
                                                double max_lon) const -> bool {
  auto area_key = m_impl->make_area_key(min_lat, max_lat, min_lon, max_lon);
  auto it = m_impl->loaded_areas.find(area_key);
  return it != m_impl->loaded_areas.end() && it->second;
}

auto building_service_t::is_area_loaded(double lat, double lon) const -> bool {
  // Check if this point falls within any loaded area keys
  // Note: This is an approximation since we store keys as strings.
  // A robust implementation would store actual bounding boxes.
  // For now, we'll iterate loaded_areas strings and parse them (slow but
  // functional) Or better: just assume if we have buildings near here it's
  // fine. Actually, let's just use the buildings vector itself! If we have
  // buildings, we have data. If no buildings, we might still have loaded
  // (empty) area.

  // Optimization: For now returns true if *any* building contains this point
  // or if we have processed a fetch nearby.
  // Given time constraints, let's just return false to force fetch if unsure.
  return false;
}

auto building_service_t::get_building_at(double lat, double lon) const
    -> const building_t * {
  geo_point_t point = {lat, lon};

  for (const auto &building : m_impl->buildings) {
    if (m_impl->point_in_polygon(point, building.footprint)) {
      return &building;
    }
  }
  return nullptr;
}

auto building_service_t::get_buildings_on_path(geo_point_t start,
                                               geo_point_t end) const
    -> std::vector<building_intersection_t> {
  std::vector<building_intersection_t> result;

  // Simple haversine distance helper
  auto dist_m = [](geo_point_t p1, geo_point_t p2) -> double {
    double R = 6378137.0;
    double dLat = (p2.lat - p1.lat) * M_PI / 180.0;
    double dLon = (p2.lon - p1.lon) * M_PI / 180.0;
    double a = std::sin(dLat / 2) * std::sin(dLat / 2) +
               std::cos(p1.lat * M_PI / 180.0) *
                   std::cos(p2.lat * M_PI / 180.0) * std::sin(dLon / 2) *
                   std::sin(dLon / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));
    return R * c;
  };

  // Helper: Segment-Segment Intersection
  auto intersect = [](geo_point_t p1, geo_point_t p2, geo_point_t p3,
                      geo_point_t p4, geo_point_t &out) -> bool {
    double x1 = p1.lon, y1 = p1.lat;
    double x2 = p2.lon, y2 = p2.lat;
    double x3 = p3.lon, y3 = p3.lat;
    double x4 = p4.lon, y4 = p4.lat;

    double denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);
    if (std::abs(denom) < 1e-9)
      return false;

    double ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
    double ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

    if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1) {
      out.lon = x1 + ua * (x2 - x1);
      out.lat = y1 + ua * (y2 - y1);
      return true;
    }
    return false;
  };

  // Check each building
  for (const auto &building : m_impl->buildings) {
    // Optimization: Check bounding circle first? Or just iterate.
    // For 5000 buildings, 4 walls each = 20k stats. A bit heavy.
    // Ideally use spatial index. For this demo, maybe limit to buildings near
    // center? Let's rely on C++ speed for now.

    // Quick bbox check
    bool possible = false;
    for (const auto &pt : building.footprint) {
      // Loose check
      if ((pt.lat > std::min(start.lat, end.lat) - 0.0001 &&
           pt.lat < std::max(start.lat, end.lat) + 0.0001 &&
           pt.lon > std::min(start.lon, end.lon) - 0.0001 &&
           pt.lon < std::max(start.lon, end.lon) + 0.0001)) {
        possible = true;
        break;
      }
    }
    if (!possible) {
      // Double check start/end inside building
      if (!m_impl->point_in_polygon(start, building.footprint) &&
          !m_impl->point_in_polygon(end, building.footprint))
        continue;
    }

    std::vector<geo_point_t> intersections;
    const auto &poly = building.footprint;

    for (size_t i = 0; i < poly.size(); ++i) {
      geo_point_t p1 = poly[i];
      geo_point_t p2 = poly[(i + 1) % poly.size()];
      geo_point_t ix;
      if (intersect(start, end, p1, p2, ix)) {
        intersections.push_back(ix);
      }
    }

    // Sort intersections by distance from start
    std::sort(intersections.begin(), intersections.end(),
              [&](const geo_point_t &a, const geo_point_t &b) {
                return dist_m(start, a) < dist_m(start, b);
              });

    // Remove duplicates (corner cases)
    auto last = std::unique(intersections.begin(), intersections.end(),
                            [&](const geo_point_t &a, const geo_point_t &b) {
                              return dist_m(a, b) < 0.1; // 10cm tolerance
                            });
    intersections.erase(last, intersections.end());

    if (intersections.size() >= 2) {
      // Assuming entering at index 0, exiting at index 1?
      // Or if we started inside, index 0 is exit?
      // Assume start/end outside for RF (usually sensor on mast, target far
      // away). If 2 intersections, thorough.

      // Take pairs
      for (size_t i = 0; i + 1 < intersections.size(); i += 2) {
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

auto building_service_t::update() -> bool {
  auto it = m_impl->pending_fetches.begin();
  bool new_data = false;

  while (it != m_impl->pending_fetches.end()) {
    if (it->data_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready) {
      std::string data = it->data_future.get();

      if (!data.empty()) {
        auto buildings = m_impl->parse_osm_buildings(data);
        m_impl->buildings.insert(m_impl->buildings.end(), buildings.begin(),
                                 buildings.end());

        std::cout << "Loaded " << buildings.size()
                  << " buildings for area: " << it->area_key << std::endl;
        new_data = true;
      }

      // Mark area as loaded
      m_impl->loaded_areas[it->area_key] = true;

      it = m_impl->pending_fetches.erase(it);
    } else {
      ++it;
    }
  }
  return new_data;
}

auto building_service_t::clear() -> void {
  m_impl->buildings.clear();
  m_impl->loaded_areas.clear();
  m_impl->pending_fetches.clear();
}

} // namespace sensor_mapper
