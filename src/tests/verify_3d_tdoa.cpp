#include "../core/tdoa_solver.hpp"
#include "../core/sensor.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include <cassert>

using namespace sensor_mapper;

int main()
{
  std::cout << "Starting 3D TDOA Verification..." << std::endl;

  tdoa_solver_t solver;
  std::vector<sensor_t> sensors;

  // Define 4 sensors in a square (~1km box)
  // 0.01 degrees is ~1.1km
  double base_lat = 0.0;
  double base_lon = 0.0;

  sensors.emplace_back("S1", base_lat, base_lon, 5000.0);
  sensors.back().set_ground_elevation(10.0);
  sensors.back().set_mast_height(0.0); // Total 10m
  sensors.back().set_use_auto_elevation(false);

  sensors.emplace_back("S2", base_lat + 0.01, base_lon, 5000.0);
  sensors.back().set_ground_elevation(20.0);
  sensors.back().set_mast_height(0.0); // Total 20m
  sensors.back().set_use_auto_elevation(false);

  sensors.emplace_back("S3", base_lat, base_lon + 0.01, 5000.0);
  sensors.back().set_ground_elevation(30.0);
  sensors.back().set_mast_height(0.0); // Total 30m
  sensors.back().set_use_auto_elevation(false);

  sensors.emplace_back("S4", base_lat + 0.01, base_lon + 0.01, 5000.0);
  sensors.back().set_ground_elevation(100.0); // 100m breaks planarity strongly
  sensors.back().set_mast_height(0.0);
  sensors.back().set_use_auto_elevation(false);

  // Define Target
  double target_lat = base_lat + 0.005;
  double target_lon = base_lon + 0.005;
  double target_alt = 150.0; // 150m altitude

  std::cout << "Target: " << target_lat << ", " << target_lon << ", " << target_alt << "m" << std::endl;

  // Calculate Truth TDOA
  auto tdoa_truth = solver.calculate_tdoa(sensors, target_lat, target_lon, target_alt);

  // Solve
  double init_lat = base_lat + 0.002;
  double init_lon = base_lon + 0.002;
  double init_alt = 50.0; // Start low

  auto result = solver.solve_position(sensors, tdoa_truth, init_lat, init_lon, init_alt);

  std::cout << "Result: " << result.latitude << ", " << result.longitude << ", " << result.altitude << "m" << std::endl;
  std::cout << "Converged: " << result.converged << " Iterations: " << result.iterations << std::endl;

  // Verify
  double lat_err = std::abs(result.latitude - target_lat);
  double lon_err = std::abs(result.longitude - target_lon);
  double alt_err = std::abs(result.altitude - target_alt);

  std::cout << "Errors: Lat=" << lat_err << " Lon=" << lon_err << " Alt=" << alt_err << std::endl;

  bool success = true;
  if (lat_err > 0.00001)
    success = false;
  if (lon_err > 0.00001)
    success = false;
  if (alt_err > 1.0)
    success = false; // Allow 1m vertical error

  if (success)
  {
    std::cout << "[SUCCESS] 3D TDOA Verified!" << std::endl;
    return 0;
  }
  else
  {
    std::cout << "[FAILURE] Errors too high." << std::endl;
    return 1;
  }
}
