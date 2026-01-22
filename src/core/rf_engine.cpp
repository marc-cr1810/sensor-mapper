#include "rf_engine.hpp"
#include "geo_math.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>


namespace sensor_mapper {

rf_engine_t::rf_engine_t() {}
rf_engine_t::~rf_engine_t() {}

auto rf_engine_t::calculate_fspl(double d_km, double f_mhz) -> double {
  if (d_km <= 0.001)
    return 0.0;
  return 20.0 * std::log10(d_km) + 20.0 * std::log10(f_mhz) + 32.44;
}

auto rf_engine_t::calculate_hata(double d_km, double f_mhz, double h_tx,
                                 double h_rx, PropagationModel model)
    -> double {
  if (d_km <= 0.001)
    return 0.0;

  // Clamp values to valid ranges for Hata or allow extension
  double log_f = std::log10(f_mhz);
  double log_h_tx = std::log10(std::max(1.0, h_tx));
  double log_d = std::log10(d_km);

  // Correction factor a(h_rx) for small/medium city
  double a_h_rx = (1.1 * log_f - 0.7) * h_rx - (1.56 * log_f - 0.8);

  double loss_urban = 69.55 + 26.16 * log_f - 13.82 * log_h_tx - a_h_rx +
                      (44.9 - 6.55 * log_h_tx) * log_d;

  if (model == PropagationModel::HataUrban) {
    return loss_urban;
  } else if (model == PropagationModel::HataSuburban) {
    double term = std::log10(f_mhz / 28.0);
    return loss_urban - 2.0 * term * term - 5.4;
  } else { // Rural
    return loss_urban - 4.78 * log_f * log_f + 18.33 * log_f - 40.94;
  }
}

auto rf_engine_t::compute_coverage(const std::vector<sensor_t> &sensors,
                                   double min_lat, double max_lat,
                                   double min_lon, double max_lon, int width,
                                   int height)
    -> std::future<std::shared_ptr<coverage_grid_t>> {

  // Copy sensors to avoid race conditions
  auto sensor_copy = sensors;

  return std::async(std::launch::async, [=]() {
    auto grid = std::make_shared<coverage_grid_t>();
    grid->width = width;
    grid->height = height;
    grid->min_lat = min_lat;
    grid->max_lat = max_lat;
    grid->min_lon = min_lon;
    grid->max_lon = max_lon;
    grid->signal_dbm.resize(width * height, -200.0f); // Init to noise

    // Pre-calculate sensor world positions (optional optimization, but we do
    // lat/lon dists)

    for (int y = 0; y < height; ++y) {
      double t = (double)y / (height - 1);
      double cell_lat = min_lat + t * (max_lat - min_lat);

      for (int x = 0; x < width; ++x) {
        double u = (double)x / (width - 1);
        double cell_lon = min_lon + u * (max_lon - min_lon);

        double max_dbm = -200.0;

        for (const auto &sensor : sensor_copy) {
          double dist_m =
              geo::distance(sensor.get_latitude(), sensor.get_longitude(),
                            cell_lat, cell_lon);

          if (dist_m > sensor.get_range())
            continue;

          double angle =
              geo::bearing(sensor.get_latitude(), sensor.get_longitude(),
                           cell_lat, cell_lon);

          // Antenna Pattern
          double azimuth = sensor.get_azimuth_deg();
          double beamwidth = sensor.get_beamwidth_deg();
          double angle_diff = std::abs(angle - azimuth);
          if (angle_diff > 180.0)
            angle_diff = 360.0 - angle_diff;

          double pattern_loss = (angle_diff > beamwidth / 2.0) ? 25.0 : 0.0;

          // Path Loss
          double d_km = dist_m / 1000.0;
          double path_loss = 0.0;
          double h_tx = sensor.get_mast_height(); // + ground elevation if we
                                                  // had it here efficiently

          if (sensor.get_propagation_model() == PropagationModel::FreeSpace) {
            path_loss = calculate_fspl(d_km, sensor.get_frequency_mhz());
          } else {
            // Assume 2m RX height
            path_loss = calculate_hata(d_km, sensor.get_frequency_mhz(), h_tx,
                                       2.0, sensor.get_propagation_model());
          }

          double rx =
              sensor.get_tx_power_dbm() + sensor.get_tx_antenna_gain_dbi() +
              sensor.get_rx_antenna_gain_dbi() - path_loss - pattern_loss;

          if (rx > max_dbm)
            max_dbm = rx;
        }

        grid->signal_dbm[y * width + x] = static_cast<float>(max_dbm);
      }
    }
    return grid;
  });
}

} // namespace sensor_mapper
