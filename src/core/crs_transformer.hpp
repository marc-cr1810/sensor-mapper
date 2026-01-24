#pragma once

#include <string>
#include <memory>

namespace sensor_mapper
{

class crs_transformer_t
{
public:
  // Initialize transformer from Source CRS to Target CRS (usually EPSG:4326 for WGS84)
  // Supports "EPSG:XXXX" formats.
  crs_transformer_t();
  ~crs_transformer_t();

  auto init(const std::string &source_crs, const std::string &target_crs = "EPSG:4326") -> bool;

  // Transform coordinate. Returns true on success.
  // forward = true: source -> target (usually UTM -> Lat/Lon)
  // forward = false: target -> source (usually Lat/Lon -> UTM)
  auto transform(double x, double y, double &out_x, double &out_y, bool forward = true) const -> bool;

  auto is_valid() const -> bool;

private:
  struct impl_t;
  std::unique_ptr<impl_t> m_impl;
};

} // namespace sensor_mapper
