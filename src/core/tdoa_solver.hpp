#pragma once

#include "sensor.hpp"
#include <vector>
#include <cmath>
#include <optional>

namespace sensor_mapper
{

// Speed of light in m/s
constexpr double SPEED_OF_LIGHT = 299792458.0;

// Convert time difference (nanoseconds) to distance difference (meters)
inline double tdoa_to_distance(double tdoa_ns)
{
  return (tdoa_ns * 1e-9) * SPEED_OF_LIGHT;
}

// Convert distance difference (meters) to time difference (nanoseconds)
inline double distance_to_tdoa(double distance_m)
{
  return (distance_m / SPEED_OF_LIGHT) * 1e9;
}

struct tdoa_result_t
{
  double latitude = 0.0;
  double longitude = 0.0;
  double error_estimate_m = 0.0;               // Estimated positioning error in meters
  double gdop = 0.0;                           // Geometric dilution of precision
  std::vector<std::vector<double>> covariance; // 2x2 Covariance matrix for error ellipse
  bool converged = false;                      // Whether solver converged
  int iterations = 0;                          // Number of solver iterations
};

class tdoa_solver_t
{
public:
  tdoa_solver_t() = default;
  ~tdoa_solver_t() = default;

  // Solve TDOA positioning from time measurements
  // tdoa_ns: Time differences in nanoseconds relative to first sensor
  // initial_lat/lon: Initial guess for iterative solver
  auto solve_position(const std::vector<sensor_t> &sensors, const std::vector<double> &tdoa_ns, double initial_lat, double initial_lon) -> tdoa_result_t;

  // Calculate TDOA values for a known position (for testing/simulation)
  // Returns time differences in nanoseconds relative to first sensor
  auto calculate_tdoa(const std::vector<sensor_t> &sensors, double target_lat, double target_lon) const -> std::vector<double>;

  // Calculate GDOP (Geometric Dilution of Precision) at a specific location
  // Lower GDOP = better geometry, typically < 3 is good, > 10 is poor
  auto calculate_gdop(const std::vector<sensor_t> &sensors, double lat, double lon) const -> double;

  // Estimate expected positioning error at a location
  // timing_jitter_ns: RMS timing measurement error (GPS-disciplined: ~10ns, typical: 50ns)
  auto estimate_positioning_error(const std::vector<sensor_t> &sensors, double lat, double lon, double timing_jitter_ns = 10.0) const -> double;

  // Sample points along a hyperbola for visualization
  // Returns lat/lon points along the hyperbola representing constant TDOA between two sensors
  auto sample_hyperbola(const sensor_t &sensor1, const sensor_t &sensor2, double tdoa_ns, int num_samples = 100) const -> std::vector<std::pair<double, double>>;

private:
  // Calculate distance from a point to a sensor in meters
  auto calculate_distance(const sensor_t &sensor, double lat, double lon) const -> double;

  // Calculate Jacobian matrix for TDOA positioning (for least-squares)
  auto calculate_jacobian(const std::vector<sensor_t> &sensors, double lat, double lon) const -> std::vector<std::vector<double>>;

  // Matrix operations for least squares solver
  auto matrix_transpose(const std::vector<std::vector<double>> &matrix) const -> std::vector<std::vector<double>>;
  auto matrix_multiply(const std::vector<std::vector<double>> &a, const std::vector<std::vector<double>> &b) const -> std::vector<std::vector<double>>;
  auto matrix_inverse_2x2(const std::vector<std::vector<double>> &matrix) const -> std::optional<std::vector<std::vector<double>>>;
};

} // namespace sensor_mapper
