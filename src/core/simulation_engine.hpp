#pragma once

#include "sensor.hpp"
#include "elevation_service.hpp"
#include "building_service.hpp"
#include "rf_models.hpp"
#include <vector>
#include <memory>

namespace sensor_mapper
{

struct SimulationResult
{
  double distance_m;
  double interpolated_lat;
  double interpolated_lon;
  double altitude_agl;
  double max_signal_dbm;
  int best_sensor_idx; // -1 if no signal
  bool los_blocked;
};

class SimulationEngine
{
public:
  SimulationEngine() = default;
  ~SimulationEngine() = default;

  // Run simulation
  // path: List of {lat, lon} points (waypoints)
  // step_size_m: Interpolation step (e.g. 10.0 meters)
  // drone_tx_power_dbm: Transmit power of the drone
  // drone_altitude_agl_m: Altitude of the drone Above Ground Level
  // frequency_mhz: Carrier frequency (if not using per-sensor freq, but currently sensor_t doesn't have freq, so we pass one global or per sensor?)
  //                Wait, existing rf_models use global frequency or freq passed in.
  //                RfEngine usually takes a frequency. Let's pass a default frequency for simplicty or use a global setting.
  //                Actually, let's assume 2400MHz if not specified, or allow override.
  std::vector<SimulationResult> run_simulation(const std::vector<std::pair<double, double>> &path, const std::vector<sensor_t> &sensors, elevation_service_t &elevation_service, building_service_t *building_service,
                                               double step_size_m = 10.0, double drone_tx_power_dbm = 20.0, double drone_altitude_agl_m = 50.0, double frequency_mhz = 2400.0);

private:
  // Helper to interpolate path
  std::vector<std::tuple<double, double, double>> interpolate_path(const std::vector<std::pair<double, double>> &path, double step_size_m);
};

} // namespace sensor_mapper
