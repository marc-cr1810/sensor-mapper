#pragma once

#include <cmath>
#include <numbers>

namespace sensor_mapper {
namespace geo {
constexpr double PI = std::numbers::pi;
constexpr double EARTH_RADIUS = 6378137.0;

// Convert Latitude/Longitude to Web Mercator World Coordinate (0.0 to 1.0)
inline auto lat_lon_to_world(double lat, double lon, double &out_x,
                             double &out_y) -> void {
  out_x = (lon + 180.0) / 360.0;

  double sin_lat = std::sin(lat * PI / 180.0);
  // Clamp to prevent singularity at poles
  if (sin_lat > 0.9999)
    sin_lat = 0.9999;
  if (sin_lat < -0.9999)
    sin_lat = -0.9999;

  out_y = 0.5 - std::log((1.0 + sin_lat) / (1.0 - sin_lat)) / (4.0 * PI);
}

// Convert World Coordinate (0.0 to 1.0) to Tile Indices
inline auto world_to_tile(double wx, double wy, int zoom, int &out_tx,
                          int &out_ty) -> void {
  double scale = static_cast<double>(1 << zoom);
  out_tx = static_cast<int>(std::floor(wx * scale));
  out_ty = static_cast<int>(std::floor(wy * scale));
}

inline auto tile_to_world(int tx, int ty, int zoom, double &out_wx,
                          double &out_wy) -> void {
  double n = static_cast<double>(1 << zoom);
  out_wx = tx / n;
  out_wy = ty / n;
}

// Convert Web Mercator World Coordinate (0.0 to 1.0) back to Latitude/Longitude
inline auto world_to_lat_lon(double wx, double wy, double &out_lat,
                             double &out_lon) -> void {
  out_lon = (wx * 360.0) - 180.0;

  double n = PI - 2.0 * PI * wy;
  out_lat = (180.0 / PI) * std::atan(0.5 * (std::exp(n) - std::exp(-n)));
}

// Move a point by distance (meters) and bearing (degrees)
inline auto destination_point(double lat, double lon, double dist_m,
                              double bearing_deg, double &out_lat,
                              double &out_lon) -> void {
  double r_earth = EARTH_RADIUS;
  double lat_rad = lat * PI / 180.0;
  double lon_rad = lon * PI / 180.0;
  double bearing_rad = bearing_deg * PI / 180.0;
  double angular_dist = dist_m / r_earth;

  double lat2_rad = std::asin(std::sin(lat_rad) * std::cos(angular_dist) +
                              std::cos(lat_rad) * std::sin(angular_dist) *
                                  std::cos(bearing_rad));
  double lon2_rad =
      lon_rad +
      std::atan2(
          std::sin(bearing_rad) * std::sin(angular_dist) * std::cos(lat_rad),
          std::cos(angular_dist) - std::sin(lat_rad) * std::sin(lat2_rad));

  out_lat = lat2_rad * 180.0 / PI;
  out_lon = lon2_rad * 180.0 / PI;
}

// Calculate bearing from point A to point B in degrees
inline auto bearing(double lat1, double lon1, double lat2, double lon2)
    -> double {
  double lat1_rad = lat1 * PI / 180.0;
  double lat2_rad = lat2 * PI / 180.0;
  double delta_lon_rad = (lon2 - lon1) * PI / 180.0;

  double y = std::sin(delta_lon_rad) * std::cos(lat2_rad);
  double x = std::cos(lat1_rad) * std::sin(lat2_rad) -
             std::sin(lat1_rad) * std::cos(lat2_rad) * std::cos(delta_lon_rad);
  double theta = std::atan2(y, x);

  // Convert to degrees and normalize to 0-360
  double bearing_deg = theta * 180.0 / PI;
  if (bearing_deg < 0.0)
    bearing_deg += 360.0;
  return bearing_deg;
}

// Calculate distance between two points in meters (Haversine)
inline auto distance(double lat1, double lon1, double lat2, double lon2)
    -> double {
  double lat1_rad = lat1 * PI / 180.0;
  double lat2_rad = lat2 * PI / 180.0;
  double delta_lat = (lat2 - lat1) * PI / 180.0;
  double delta_lon = (lon2 - lon1) * PI / 180.0;

  double a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
             std::cos(lat1_rad) * std::cos(lat2_rad) *
                 std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
  double c = 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
  return EARTH_RADIUS * c;
}

} // namespace geo
} // namespace sensor_mapper
