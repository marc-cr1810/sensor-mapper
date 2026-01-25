#include "tdoa_solver.hpp"
#include "geo_math.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>

namespace sensor_mapper
{

auto tdoa_solver_t::calculate_distance(const sensor_t &sensor, double lat, double lon, double alt) const -> double
{
  double dist_2d = geo::distance(sensor.get_latitude(), sensor.get_longitude(), lat, lon);
  double sensor_alt = sensor.get_ground_elevation() + sensor.get_mast_height();
  double alt_diff = alt - sensor_alt;
  return std::sqrt(dist_2d * dist_2d + alt_diff * alt_diff);
}

auto tdoa_solver_t::calculate_tdoa(const std::vector<sensor_t> &sensors, double target_lat, double target_lon, double target_alt) const -> std::vector<double>
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
    distances.push_back(calculate_distance(sensor, target_lat, target_lon, target_alt));
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

auto tdoa_solver_t::solve_position(const std::vector<sensor_t> &sensors, const std::vector<double> &tdoa_ns, double initial_lat, double initial_lon, double initial_alt) -> tdoa_result_t
{
  tdoa_result_t result;

  // Need at least 4 sensors for 3D TDOA (x,y,z, t_bias - but t_bias eliminated by differencing, so 3 vars -> 3 independent diffs -> 4 sensors)
  // Actually with N sensors we have N-1 TDOA measurements. To solve for 3 variables we need 3 measurements.
  // So 4 sensors total are required for unique 3D solution.
  if (sensors.size() < 4 || tdoa_ns.size() != sensors.size())
  {
    return result;
  }

  // Gauss-Newton iterative solver
  double lat = initial_lat;
  double lon = initial_lon;
  double alt = initial_alt;

  const int max_iterations = 50;

  std::optional<std::vector<std::vector<double>>> final_cov;

  for (int iter = 0; iter < max_iterations; ++iter)
  {
    // Calculate current TDOA estimates
    auto estimated_tdoa = calculate_tdoa(sensors, lat, lon, alt);

    // Calculate residuals (error vector)
    // Residual = Measured - Estimated
    std::vector<double> residuals_m;
    residuals_m.reserve(sensors.size() - 1);
    for (size_t i = 1; i < sensors.size(); ++i)
    {
      // Convert time residual (ns) to distance residual (m)
      double tdoa_diff = tdoa_ns[i] - estimated_tdoa[i];
      residuals_m.push_back(tdoa_to_distance(tdoa_diff));
    }

    // Calculate Jacobian (dRangeDiff / dPosMeters)
    // J is (N-1) x 3
    auto J = calculate_jacobian(sensors, lat, lon, alt);

    // Solve: (J^T * J) * delta = J^T * residuals
    auto JT = matrix_transpose(J);
    auto JTJ = matrix_multiply(JT, J); // 3x3 matrix

    // Check determinant / inverse
    auto JTJ_inv = matrix_inverse_3x3(JTJ);

    if (!JTJ_inv.has_value())
    {
      // Singular matrix - poor geometry
      result.converged = false;
      return result;
    }

    final_cov = JTJ_inv;

    // Calculate J^T * residuals (3x1 vector)
    std::vector<double> JT_residuals(3, 0.0);
    for (size_t i = 0; i < JT.size(); ++i)
    {
      for (size_t j = 0; j < residuals_m.size(); ++j)
      {
        JT_residuals[i] += JT[i][j] * residuals_m[j];
      }
    }

    // Calculate delta = JTJ_inv * JT_residuals
    // Delta is now in METERS (North, East, Up)
    double delta_north_m = (*JTJ_inv)[0][0] * JT_residuals[0] + (*JTJ_inv)[0][1] * JT_residuals[1] + (*JTJ_inv)[0][2] * JT_residuals[2];
    double delta_east_m = (*JTJ_inv)[1][0] * JT_residuals[0] + (*JTJ_inv)[1][1] * JT_residuals[1] + (*JTJ_inv)[1][2] * JT_residuals[2];
    double delta_up_m = (*JTJ_inv)[2][0] * JT_residuals[0] + (*JTJ_inv)[2][1] * JT_residuals[1] + (*JTJ_inv)[2][2] * JT_residuals[2];

    // Step Clamping (Damping)
    double delta_mag_m = std::sqrt(delta_north_m * delta_north_m + delta_east_m * delta_east_m + delta_up_m * delta_up_m);
    const double max_step_m = 500.0;

    if (delta_mag_m > max_step_m)
    {
      double scale = max_step_m / delta_mag_m;
      delta_north_m *= scale;
      delta_east_m *= scale;
      delta_up_m *= scale;
      delta_mag_m = max_step_m;
    }

    // Update position (convert meters to degrees)
    double deg_per_m_lat = 1.0 / 111111.0;
    double deg_per_m_lon = 1.0 / (111111.0 * std::cos(lat * M_PI / 180.0));

    lat += delta_north_m * deg_per_m_lat;
    lon += delta_east_m * deg_per_m_lon;
    alt += delta_up_m;

    result.iterations = iter + 1;

    // Check convergence
    if (delta_mag_m < 0.01) // Converge when step is < 1cm
    {
      result.converged = true;
      break;
    }
  }

  result.latitude = lat;
  result.longitude = lon;
  result.altitude = alt;
  result.gdop = calculate_gdop(sensors, lat, lon, alt);
  result.error_estimate_m = estimate_positioning_error(sensors, lat, lon, alt);
  if (final_cov.has_value())
  {
    result.covariance = *final_cov;
  }

  return result;
}

auto tdoa_solver_t::calculate_jacobian(const std::vector<sensor_t> &sensors, double lat, double lon, double alt) const -> std::vector<std::vector<double>>
{
  const double epsilon_m = 1.0; // Small perturbation in meters for numerical derivative

  const double meters_per_deg_lat = 111111.0;
  double meters_per_deg_lon = 111111.0 * std::cos(lat * M_PI / 180.0);
  if (std::abs(meters_per_deg_lon) < 1.0)
    meters_per_deg_lon = 1.0; // Avoid div/0 at poles

  double d_lat = epsilon_m / meters_per_deg_lat;
  double d_lon = epsilon_m / meters_per_deg_lon;
  double d_alt = epsilon_m;

  std::vector<std::vector<double>> J;
  J.reserve(sensors.size() - 1);

  // We need d(RangeDiff) / d(Pos)

  auto tdoa_base = calculate_tdoa(sensors, lat, lon, alt);
  auto tdoa_lat_p = calculate_tdoa(sensors, lat + d_lat, lon, alt);
  auto tdoa_lon_p = calculate_tdoa(sensors, lat, lon + d_lon, alt);
  auto tdoa_alt_p = calculate_tdoa(sensors, lat, lon, alt + d_alt);

  for (size_t i = 1; i < sensors.size(); ++i)
  {
    double r_base = tdoa_to_distance(tdoa_base[i]);
    double r_lat = tdoa_to_distance(tdoa_lat_p[i]);
    double r_lon = tdoa_to_distance(tdoa_lon_p[i]);
    double r_alt = tdoa_to_distance(tdoa_alt_p[i]);

    // Derivatives: d(RangeDiff) / d(DistanceAlongAxis)
    double dr_dx = (r_lat - r_base) / epsilon_m; // North-South derivative
    double dr_dy = (r_lon - r_base) / epsilon_m; // East-West derivative
    double dr_dz = (r_alt - r_base) / epsilon_m; // Up-Down

    J.push_back({dr_dx, dr_dy, dr_dz});
  }

  return J;
}

auto tdoa_solver_t::calculate_gdop(const std::vector<sensor_t> &sensors, double lat, double lon, double alt) const -> double
{
  if (sensors.size() < 4)
    return std::numeric_limits<double>::infinity();

  auto J = calculate_jacobian(sensors, lat, lon, alt);
  auto JT = matrix_transpose(J);
  auto JTJ = matrix_multiply(JT, J);
  auto JTJ_inv = matrix_inverse_3x3(JTJ);

  if (!JTJ_inv.has_value())
  {
    return std::numeric_limits<double>::infinity();
  }

  // GDOP = sqrt(trace(covariance_matrix))
  // For 3D: GDOP = sqrt(cov[0][0] + cov[1][1] + cov[2][2])
  double trace = (*JTJ_inv)[0][0] + (*JTJ_inv)[1][1] + (*JTJ_inv)[2][2];
  if (trace < 0)
    return std::numeric_limits<double>::infinity();

  return std::sqrt(trace);
}

auto tdoa_solver_t::estimate_positioning_error(const std::vector<sensor_t> &sensors, double lat, double lon, double alt, double timing_jitter_ns) const -> double
{
  double gdop = calculate_gdop(sensors, lat, lon, alt);
  double range_error_m = tdoa_to_distance(timing_jitter_ns);
  return gdop * range_error_m * std::sqrt(2.0);
}

auto tdoa_solver_t::calculate_2d_gdop(const std::vector<sensor_t> &sensors, double lat, double lon, double fixed_alt) const -> double
{
  if (sensors.size() < 3)
    return std::numeric_limits<double>::infinity();

  const double epsilon_m = 1.0;
  const double meters_per_deg_lat = 111111.0;
  double meters_per_deg_lon = 111111.0 * std::cos(lat * M_PI / 180.0);
  if (std::abs(meters_per_deg_lon) < 1.0)
    meters_per_deg_lon = 1.0;

  double d_lat = epsilon_m / meters_per_deg_lat;
  double d_lon = epsilon_m / meters_per_deg_lon;

  // Calculate 2D Jacobian (N-1) x 2
  std::vector<std::vector<double>> J;
  J.reserve(sensors.size() - 1);

  auto tdoa_base = calculate_tdoa(sensors, lat, lon, fixed_alt);
  auto tdoa_lat_p = calculate_tdoa(sensors, lat + d_lat, lon, fixed_alt);
  auto tdoa_lon_p = calculate_tdoa(sensors, lat, lon + d_lon, fixed_alt);

  for (size_t i = 1; i < sensors.size(); ++i)
  {
    double r_base = tdoa_to_distance(tdoa_base[i]);
    double r_lat = tdoa_to_distance(tdoa_lat_p[i]);
    double r_lon = tdoa_to_distance(tdoa_lon_p[i]);

    double dr_dx = (r_lat - r_base) / epsilon_m;
    double dr_dy = (r_lon - r_base) / epsilon_m;

    J.push_back({dr_dx, dr_dy});
  }

  auto JT = matrix_transpose(J);
  auto JTJ = matrix_multiply(JT, J); // 2x2
  auto JTJ_inv = matrix_inverse_2x2(JTJ);

  if (!JTJ_inv.has_value())
  {
    return std::numeric_limits<double>::infinity();
  }

  double trace = (*JTJ_inv)[0][0] + (*JTJ_inv)[1][1];
  if (trace < 0)
    return std::numeric_limits<double>::infinity();

  return std::sqrt(trace);
}

auto tdoa_solver_t::estimate_2d_positioning_error(const std::vector<sensor_t> &sensors, double lat, double lon, double fixed_alt, double timing_jitter_ns) const -> double
{
  double gdop = calculate_2d_gdop(sensors, lat, lon, fixed_alt);
  double range_error_m = tdoa_to_distance(timing_jitter_ns);
  return gdop * range_error_m * std::sqrt(2.0);
}

auto tdoa_solver_t::solve_position_2d(const std::vector<sensor_t> &sensors, const std::vector<double> &tdoa_ns, double initial_lat, double initial_lon, double fixed_alt) -> tdoa_result_t
{
  tdoa_result_t result;
  // Need at least 3 sensors for 2D (x,y, t_bias -> 2 diffs -> 3 sensors)
  if (sensors.size() < 3 || tdoa_ns.size() != sensors.size())
    return result;

  double lat = initial_lat;
  double lon = initial_lon;
  const int max_iterations = 50;

  std::optional<std::vector<std::vector<double>>> final_cov;

  for (int iter = 0; iter < max_iterations; ++iter)
  {
    auto estimated_tdoa = calculate_tdoa(sensors, lat, lon, fixed_alt);
    std::vector<double> residuals_m;
    residuals_m.reserve(sensors.size() - 1);
    for (size_t i = 1; i < sensors.size(); ++i)
    {
      double tdoa_diff = tdoa_ns[i] - estimated_tdoa[i];
      residuals_m.push_back(tdoa_to_distance(tdoa_diff));
    }

    // Calculate 2D Jacobian manually here to avoid re-alloc
    const double epsilon_m = 1.0;
    const double meters_per_deg_lat = 111111.0;
    double meters_per_deg_lon = 111111.0 * std::cos(lat * M_PI / 180.0);
    if (std::abs(meters_per_deg_lon) < 1.0)
      meters_per_deg_lon = 1.0;

    auto tdoa_lat_p = calculate_tdoa(sensors, lat + epsilon_m / meters_per_deg_lat, lon, fixed_alt);
    auto tdoa_lon_p = calculate_tdoa(sensors, lat, lon + epsilon_m / meters_per_deg_lon, fixed_alt);

    std::vector<std::vector<double>> J;
    J.reserve(sensors.size() - 1);
    for (size_t i = 1; i < sensors.size(); ++i)
    {
      double r_base = tdoa_to_distance(estimated_tdoa[i]); // Recalculate or use from above? distance_to_tdoa is linear? No.
      // Wait, estimated_tdoa has distances converted to time.
      // We need distances.
      // Better: J calculation is identical to calculate_2d_gdop logic.
      // Let's just duplicate the logic concisely.
      double dr_dx = (tdoa_to_distance(tdoa_lat_p[i]) - tdoa_to_distance(estimated_tdoa[i])) / epsilon_m;
      double dr_dy = (tdoa_to_distance(tdoa_lon_p[i]) - tdoa_to_distance(estimated_tdoa[i])) / epsilon_m;
      J.push_back({dr_dx, dr_dy});
    }

    auto JT = matrix_transpose(J);
    auto JTJ = matrix_multiply(JT, J);
    auto JTJ_inv = matrix_inverse_2x2(JTJ);

    if (!JTJ_inv.has_value())
    {
      result.converged = false;
      return result;
    }
    final_cov = JTJ_inv;

    std::vector<double> JT_residuals(2, 0.0);
    for (size_t i = 0; i < 2; ++i)
    {
      for (size_t j = 0; j < residuals_m.size(); ++j)
      {
        JT_residuals[i] += JT[i][j] * residuals_m[j];
      }
    }

    double delta_north_m = (*JTJ_inv)[0][0] * JT_residuals[0] + (*JTJ_inv)[0][1] * JT_residuals[1];
    double delta_east_m = (*JTJ_inv)[1][0] * JT_residuals[0] + (*JTJ_inv)[1][1] * JT_residuals[1];

    // Clamping
    double delta_mag_m = std::sqrt(delta_north_m * delta_north_m + delta_east_m * delta_east_m);
    const double max_step_m = 500.0;
    if (delta_mag_m > max_step_m)
    {
      double scale = max_step_m / delta_mag_m;
      delta_north_m *= scale;
      delta_east_m *= scale;
      delta_mag_m = max_step_m;
    }

    lat += delta_north_m * (1.0 / meters_per_deg_lat);
    lon += delta_east_m * (1.0 / meters_per_deg_lon);

    result.iterations = iter + 1;
    if (delta_mag_m < 0.01)
    {
      result.converged = true;
      break;
    }
  }

  result.latitude = lat;
  result.longitude = lon;
  result.altitude = fixed_alt;
  result.gdop = calculate_2d_gdop(sensors, lat, lon, fixed_alt);
  result.error_estimate_m = estimate_2d_positioning_error(sensors, lat, lon, fixed_alt);
  if (final_cov.has_value())
  {
    // Pad 2x2 to 3x3 for compatibility? Or just store 2x2 if struct allows?
    // Struct expects vector<vector<double>>.
    // Let's pad with zeros for Z to avoid crashes if UI expects 3x3
    std::vector<std::vector<double>> cov3x3(3, std::vector<double>(3, 0.0));
    cov3x3[0][0] = (*final_cov)[0][0];
    cov3x3[0][1] = (*final_cov)[0][1];
    cov3x3[1][0] = (*final_cov)[1][0];
    cov3x3[1][1] = (*final_cov)[1][1];
    result.covariance = cov3x3;
  }
  return result;
}

auto tdoa_solver_t::sample_hyperbola(const sensor_t &sensor1, const sensor_t &sensor2, double tdoa_ns, double slice_alt_m, int num_samples) const -> std::vector<std::pair<double, double>>
{
  std::vector<std::pair<double, double>> points;
  double lat1 = sensor1.get_latitude();
  double lon1 = sensor1.get_longitude();
  double lat2 = sensor2.get_latitude();
  double lon2 = sensor2.get_longitude();

  double distance_diff = tdoa_to_distance(tdoa_ns);
  double baseline = geo::distance(lat1, lon1, lat2, lon2);

  double a = std::abs(distance_diff) / 2.0;
  double c = baseline / 2.0;

  if (a >= c)
    return points;

  double center_lat = (lat1 + lat2) / 2.0;
  double center_lon = (lon1 + lon2) / 2.0;
  double bearing_val = geo::bearing(lat1, lon1, lat2, lon2);
  double b = std::sqrt(c * c - a * a);
  double t_range = 6.0;

  points.reserve(num_samples);

  for (int i = 0; i < num_samples; ++i)
  {
    double t = -t_range + (2.0 * t_range * i / (num_samples - 1));
    double x = a * std::cosh(t);
    if (distance_diff < 0)
      x = -x;
    double y = b * std::sinh(t);

    double theta_rad = bearing_val * M_PI / 180.0;
    double delta_north = x * std::cos(theta_rad) - y * std::sin(theta_rad);
    double delta_east = x * std::sin(theta_rad) + y * std::cos(theta_rad);

    double deg_lat = delta_north / 111320.0;
    double deg_lon = delta_east / (111320.0 * std::cos(center_lat * M_PI / 180.0));
    points.push_back({center_lat + deg_lat, center_lon + deg_lon});
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
    for (size_t j = 0; j < cols; ++j)
      result[j][i] = matrix[i][j];
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
    for (size_t j = 0; j < cols; ++j)
      for (size_t k = 0; k < inner; ++k)
        result[i][j] += a[i][k] * b[k][j];
  return result;
}

auto tdoa_solver_t::matrix_inverse_3x3(const std::vector<std::vector<double>> &m) const -> std::optional<std::vector<std::vector<double>>>
{
  if (m.size() != 3 || m[0].size() != 3)
    return std::nullopt;

  double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  if (std::abs(det) < 1e-12)
    return std::nullopt;

  double invdet = 1.0 / det;
  std::vector<std::vector<double>> minv(3, std::vector<double>(3));

  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;

  return minv;
}

auto tdoa_solver_t::matrix_inverse_2x2(const std::vector<std::vector<double>> &m) const -> std::optional<std::vector<std::vector<double>>>
{
  if (m.size() != 2 || m[0].size() != 2)
    return std::nullopt;
  double det = m[0][0] * m[1][1] - m[0][1] * m[1][0];
  if (std::abs(det) < 1e-12)
    return std::nullopt;

  double invdet = 1.0 / det;
  std::vector<std::vector<double>> minv(2, std::vector<double>(2));
  minv[0][0] = m[1][1] * invdet;
  minv[0][1] = -m[0][1] * invdet;
  minv[1][0] = -m[1][0] * invdet;
  minv[1][1] = m[0][0] * invdet;
  return minv;
}

} // namespace sensor_mapper
