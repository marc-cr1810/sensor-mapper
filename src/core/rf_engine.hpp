#pragma once

#include "elevation_service.hpp"
#include "building_service.hpp"
#include "sensor.hpp"
#include <array>
#include <future>
#include <memory>
#include <vector>

namespace sensor_mapper
{

struct coverage_grid_t
{
  int width;
  int height;
  double min_lat;
  double max_lat;
  double min_lon;
  double max_lon;
  std::vector<float> signal_dbm; // Flattened 2D array
};

class rf_engine_t
{
public:
  rf_engine_t();
  ~rf_engine_t();

  // Async coverage calculation
  auto compute_coverage(const std::vector<sensor_t> &sensors, elevation_service_t *elevation_service, const building_service_t *building_service, double min_lat, double max_lat, double min_lon, double max_lon, int width, int height)
      -> std::future<std::shared_ptr<coverage_grid_t>>;

  // Propagation Models
  // calculate_fresnel_zone remains useful utility?
  // Yes, keep fresnel zone here or move to geo_math?
  // rf_models might be better place, but let's keep it here for now to minimize churn.

  // Calculate radius of the nth Fresnel zone at a specific distance point

  // d1: distance from first endpoint (meters)
  // d2: distance from second endpoint (meters)
  // wavelength: signal wavelength (meters)
  static auto calculate_fresnel_zone(double d1, double d2, double wavelength, int n = 1) -> double;

private:
  struct impl_t;
  // std::unique_ptr<impl_t> m_impl; // No state needed yet, just threads
};

} // namespace sensor_mapper
