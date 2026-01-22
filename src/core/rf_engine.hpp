#pragma once

#include "sensor.hpp"
#include <array>
#include <future>
#include <memory>
#include <vector>


namespace sensor_mapper {

struct coverage_grid_t {
  int width;
  int height;
  double min_lat;
  double max_lat;
  double min_lon;
  double max_lon;
  std::vector<float> signal_dbm; // Flattened 2D array
};

class rf_engine_t {
public:
  rf_engine_t();
  ~rf_engine_t();

  // Async coverage calculation
  auto compute_coverage(const std::vector<sensor_t> &sensors, double min_lat,
                        double max_lat, double min_lon, double max_lon,
                        int width, int height)
      -> std::future<std::shared_ptr<coverage_grid_t>>;

  // Propagation Models
  static auto calculate_fspl(double d_km, double f_mhz) -> double;

  static auto calculate_hata(double d_km, double f_mhz, double h_tx,
                             double h_rx, PropagationModel model) -> double;

private:
  struct impl_t;
  // std::unique_ptr<impl_t> m_impl; // No state needed yet, just threads
};

} // namespace sensor_mapper
