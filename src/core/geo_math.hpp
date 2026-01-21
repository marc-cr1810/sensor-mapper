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
} // namespace geo
} // namespace sensor_mapper
