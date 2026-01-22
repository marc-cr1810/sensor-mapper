#include "tdoa_solver.hpp"
#include "geo_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace sensor_mapper
{

auto tdoa_solver_t::calculate_distance(const sensor_t &sensor, double lat, double lon) const -> double
{
  return geo::distance(sensor.get_latitude(), sensor.get_longitude(), lat, lon);
}

auto tdoa_solver_t::calculate_tdoa(const std::vector<sensor_t> &sensors, double target_lat, double target_lon) const -> std::vector<double>
{
  if (sensors.empty())
    return {};

  std::vector<double> tdoa_ns;
  tdoa_ns.reserve(sensors.size());

  // Calculate distance from target to each sensor
  std::vector<double> distances;
  distances.reserve(sensors.size());
  for (const auto &sensor : sensors)
  {
    distances.push_back(calculate_distance(sensor, target_lat, target_lon));
  }

  // TDOA is relative to first sensor
  double reference_distance = distances[0];
  for (size_t i = 0; i < sensors.size(); ++i)
  {
    double distance_diff = distances[i] - reference_distance;
    tdoa_ns.push_back(distance_to_tdoa(distance_diff));
  }

  return tdoa_ns;
}

auto tdoa_solver_t::solve_position(const std::vector<sensor_t> &sensors, const std::vector<double> &tdoa_ns, double initial_lat, double initial_lon) -> tdoa_result_t
{
  tdoa_result_t result;

  if (sensors.size() < 3 || tdoa_ns.size() != sensors.size())
  {
    return result; // Need at least 3 sensors
  }

  // Gauss-Newton iterative solver
  double lat = initial_lat;
  double lon = initial_lon;

  const int max_iterations = 50;
  const double convergence_threshold = 1e-8; // radians

  for (int iter = 0; iter < max_iterations; ++iter)
  {
    // Calculate current TDOA estimates
    auto estimated_tdoa = calculate_tdoa(sensors, lat, lon);

    // Calculate residuals (error vector)
    std::vector<double> residuals;
    residuals.reserve(sensors.size() - 1);
    for (size_t i = 1; i < sensors.size(); ++i)
    {
      residuals.push_back(tdoa_ns[i] - estimated_tdoa[i]);
    }

    // Calculate Jacobian
    auto J = calculate_jacobian(sensors, lat, lon);

    // Solve: (J^T * J) * delta = J^T * residuals
    auto JT = matrix_transpose(J);
    auto JTJ = matrix_multiply(JT, J);
    auto JTJ_inv = matrix_inverse_2x2(JTJ);

    if (!JTJ_inv.has_value())
    {
      // Singular matrix - poor geometry
      result.converged = false;
      return result;
    }

    // Calculate J^T * residuals
    std::vector<double> JT_residuals(2, 0.0);
    for (size_t i = 0; i < JT.size(); ++i)
    {
      for (size_t j = 0; j < residuals.size(); ++j)
      {
        JT_residuals[i] += JT[i][j] * residuals[j];
      }
    }

    // Calculate delta = JTJ_inv * JT_residuals
    double delta_lat = (*JTJ_inv)[0][0] * JT_residuals[0] + (*JTJ_inv)[0][1] * JT_residuals[1];
    double delta_lon = (*JTJ_inv)[1][0] * JT_residuals[0] + (*JTJ_inv)[1][1] * JT_residuals[1];

    // Update position
    lat += delta_lat;
    lon += delta_lon;

    result.iterations = iter + 1;

    // Check convergence
    double delta_magnitude = std::sqrt(delta_lat * delta_lat + delta_lon * delta_lon);
    if (delta_magnitude < convergence_threshold)
    {
      result.converged = true;
      break;
    }
  }

  result.latitude = lat;
  result.longitude = lon;
  result.gdop = calculate_gdop(sensors, lat, lon);
  result.error_estimate_m = estimate_positioning_error(sensors, lat, lon);

  return result;
}

auto tdoa_solver_t::calculate_jacobian(const std::vector<sensor_t> &sensors, double lat, double lon) const -> std::vector<std::vector<double>>
{
  // Jacobian for TDOA positioning: d(TDOA)/d(lat), d(TDOA)/d(lon)
  // We have (n-1) TDOA measurements (relative to first sensor)

  const double epsilon = 1e-6; // Small perturbation in degrees

  std::vector<std::vector<double>> J;
  J.reserve(sensors.size() - 1);

  auto tdoa_at_point = calculate_tdoa(sensors, lat, lon);
  auto tdoa_lat_plus = calculate_tdoa(sensors, lat + epsilon, lon);
  auto tdoa_lon_plus = calculate_tdoa(sensors, lat, lon + epsilon);

  for (size_t i = 1; i < sensors.size(); ++i)
  {
    double d_tdoa_d_lat = (tdoa_lat_plus[i] - tdoa_at_point[i]) / epsilon;
    double d_tdoa_d_lon = (tdoa_lon_plus[i] - tdoa_at_point[i]) / epsilon;
    J.push_back({d_tdoa_d_lat, d_tdoa_d_lon});
  }

  return J;
}

auto tdoa_solver_t::calculate_gdop(const std::vector<sensor_t> &sensors, double lat, double lon) const -> double
{
  if (sensors.size() < 3)
    return std::numeric_limits<double>::infinity();

  auto J = calculate_jacobian(sensors, lat, lon);
  auto JT = matrix_transpose(J);
  auto JTJ = matrix_multiply(JT, J);
  auto JTJ_inv = matrix_inverse_2x2(JTJ);

  if (!JTJ_inv.has_value())
  {
    return std::numeric_limits<double>::infinity();
  }

  // GDOP = sqrt(trace(covariance_matrix))
  // For 2D: GDOP = sqrt(cov[0][0] + cov[1][1])
  double gdop = std::sqrt((*JTJ_inv)[0][0] + (*JTJ_inv)[1][1]);

  return gdop;
}

auto tdoa_solver_t::estimate_positioning_error(const std::vector<sensor_t> &sensors, double lat, double lon, double timing_jitter_ns) const -> double
{
  double gdop = calculate_gdop(sensors, lat, lon);

  // Convert timing jitter to distance error
  double range_error_m = tdoa_to_distance(timing_jitter_ns);

  // Position error = GDOP * range_error
  // Multiply by sqrt(2) for conservative estimate (accounts for independent measurements)
  return gdop * range_error_m * std::sqrt(2.0);
}

auto tdoa_solver_t::sample_hyperbola(const sensor_t &sensor1, const sensor_t &sensor2, double tdoa_ns, int num_samples) const -> std::vector<std::pair<double, double>>
{
  std::vector<std::pair<double, double>> points;

  // Get sensor positions
  double lat1 = sensor1.get_latitude();
  double lon1 = sensor1.get_longitude();
  double lat2 = sensor2.get_latitude();
  double lon2 = sensor2.get_longitude();

  // Distance difference in meters
  double distance_diff = tdoa_to_distance(tdoa_ns);

  // Distance between sensors
  double baseline = geo::distance(lat1, lon1, lat2, lon2);

  // Hyperbola parameters
  // For a hyperbola with foci at sensor1 and sensor2:
  // |d1 - d2| = 2a, where 2a = distance_diff
  double a = std::abs(distance_diff) / 2.0;
  double c = baseline / 2.0; // Half the distance between foci

  // Hyperbola only exists if a < c
  if (a >= c)
    return points;

  // Center point between sensors
  double center_lat = (lat1 + lat2) / 2.0;
  double center_lon = (lon1 + lon2) / 2.0;

  // Angle from sensor1 to sensor2
  double bearing_val = geo::bearing(lat1, lon1, lat2, lon2);

  // Sample the hyperbola
  // Use parametric approach: sample along a line perpendicular to baseline
  double b = std::sqrt(c * c - a * a); // Semi-minor axis

  points.reserve(num_samples);

  for (int i = 0; i < num_samples; ++i)
  {
    // Sample parameter t from -range to +range
    double t = -3.0 + (6.0 * i / (num_samples - 1)); // Sample around hyperbola

    // Hyperbola equation: x = ±a*cosh(t), y = b*sinh(t)
    // We'll sample both branches
    for (int branch = -1; branch <= 1; branch += 2)
    {
      double x = branch * a * std::cosh(t); // Along baseline direction
      double y = b * std::sinh(t);          // Perpendicular to baseline

      // Rotate and translate to world coordinates
      double rotated_x = x * std::cos(bearing_val * M_PI / 180.0) - y * std::sin(bearing_val * M_PI / 180.0);
      double rotated_y = x * std::sin(bearing_val * M_PI / 180.0) + y * std::cos(bearing_val * M_PI / 180.0);

      // Convert to lat/lon (approximate for small areas)
      double lat = center_lat + (rotated_y / 111320.0); // 1 degree latitude ≈ 111.32 km
      double lon = center_lon + (rotated_x / (111320.0 * std::cos(center_lat * M_PI / 180.0)));

      points.push_back({lat, lon});
    }
  }

  return points;
}

auto tdoa_solver_t::matrix_transpose(const std::vector<std::vector<double>> &matrix) const -> std::vector<std::vector<double>>
{
  if (matrix.empty())
    return {};

  size_t rows = matrix.size();
  size_t cols = matrix[0].size();

  std::vector<std::vector<double>> result(cols, std::vector<double>(rows));

  for (size_t i = 0; i < rows; ++i)
  {
    for (size_t j = 0; j < cols; ++j)
    {
      result[j][i] = matrix[i][j];
    }
  }

  return result;
}

auto tdoa_solver_t::matrix_multiply(const std::vector<std::vector<double>> &a, const std::vector<std::vector<double>> &b) const -> std::vector<std::vector<double>>
{
  if (a.empty() || b.empty() || a[0].size() != b.size())
    return {};

  size_t rows = a.size();
  size_t cols = b[0].size();
  size_t inner = b.size();

  std::vector<std::vector<double>> result(rows, std::vector<double>(cols, 0.0));

  for (size_t i = 0; i < rows; ++i)
  {
    for (size_t j = 0; j < cols; ++j)
    {
      for (size_t k = 0; k < inner; ++k)
      {
        result[i][j] += a[i][k] * b[k][j];
      }
    }
  }

  return result;
}

auto tdoa_solver_t::matrix_inverse_2x2(const std::vector<std::vector<double>> &matrix) const -> std::optional<std::vector<std::vector<double>>>
{
  if (matrix.size() != 2 || matrix[0].size() != 2)
    return std::nullopt;

  double a = matrix[0][0];
  double b = matrix[0][1];
  double c = matrix[1][0];
  double d = matrix[1][1];

  double det = a * d - b * c;

  if (std::abs(det) < 1e-12)
    return std::nullopt; // Singular matrix

  std::vector<std::vector<double>> inv(2, std::vector<double>(2));
  inv[0][0] = d / det;
  inv[0][1] = -b / det;
  inv[1][0] = -c / det;
  inv[1][1] = a / det;

  return inv;
}

} // namespace sensor_mapper
