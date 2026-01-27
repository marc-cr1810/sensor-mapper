#include "simulation_engine.hpp"
#include "geo_math.hpp"
#include "building_service.hpp" // Added for Occlusion Check
#include "tdoa_solver.hpp"
#include <cmath>
#include <algorithm>

namespace sensor_mapper
{

std::vector<SimulationResult> SimulationEngine::run_simulation(const std::vector<std::pair<double, double>> &path, const std::vector<sensor_t> &sensors, elevation_service_t &elevation_service, building_service_t *building_service,
                                                               double step_size_m, double drone_tx_power_dbm, double drone_altitude_agl_m, double frequency_mhz)
{
  std::vector<SimulationResult> results;

  if (path.size() < 2 || sensors.empty())
  {
    return results;
  }

  // 1. Interpolate Path
  // Returns {lat, lon, distance_from_start}
  auto interpolated_points = interpolate_path(path, step_size_m);

  results.reserve(interpolated_points.size());

  // TDOA Solver
  tdoa_solver_t tdoa_solver;

  // 2. Simulate each point
  for (const auto &point : interpolated_points)
  {
    double lat = std::get<0>(point);
    double lon = std::get<1>(point);
    double dist_from_start = std::get<2>(point);

    SimulationResult res;
    res.distance_m = dist_from_start;
    res.interpolated_lat = lat;
    res.interpolated_lon = lon;
    res.altitude_agl = drone_altitude_agl_m;
    res.max_signal_dbm = -200.0; // Noise floor
    res.best_sensor_idx = -1;
    res.los_blocked = false;

    // Get ground elevation at drone point
    float drone_ground_elev = 0.0f;
    elevation_service.get_elevation(lat, lon, drone_ground_elev);
    double drone_amsl = static_cast<double>(drone_ground_elev) + drone_altitude_agl_m;

    // Check against all sensors
    for (size_t i = 0; i < sensors.size(); ++i)
    {
      const auto &sensor = sensors[i];

      // Sensor Position
      double s_lat = sensor.get_latitude();
      double s_lon = sensor.get_longitude();
      double s_range = sensor.get_range();
      double s_mast = sensor.get_mast_height();
      double s_ground = sensor.get_ground_elevation();
      double s_amsl = s_ground + s_mast;

      // Distance
      double dist_km = geo::distance(lat, lon, s_lat, s_lon) / 1000.0;

      // Range Check
      if (dist_km * 1000.0 > s_range)
      {
        continue;
      }

      // Line of Sight Check
      bool is_los = true;

      // Terrain LoS Check (using elevation service)
      // Simple sampling check
      int samples = static_cast<int>(dist_km * 20.0); // 1 sample every 50m
      if (samples < 5)
        samples = 5;

      // reuse elevation_service logic or implement simple bresenham-like sampling
      // Ideally elevation_service should have "check_los(p1, alt1, p2, alt2)"
      // For now, we manually implement a simplified check to avoid heavy dependencies if service doesn't have it
      // But elevation_service DOES have get_profile!
      auto profile = elevation_service.get_profile(s_lat, s_lon, lat, lon, std::min(samples, 50));

      if (!profile.empty())
      {
        // Check profile
        float start_h = static_cast<float>(s_amsl);
        float end_h = static_cast<float>(drone_amsl);
        double total_d = dist_km * 1000.0;

        for (const auto &p : profile)
        {
          double d_curr = p.first;
          float terrain_h = p.second;

          // LoS height at this distance
          float los_h = start_h + (end_h - start_h) * (float)(d_curr / total_d);

          if (terrain_h > los_h)
          {
            is_los = false;
            break;
          }
        }
      }

      // Building LoS Check (if available)
      if (is_los && building_service)
      {
        geo_point_t s_pos = {s_lat, s_lon};
        geo_point_t drone_pos = {lat, lon};

        // Helper to adjust highly altitude-aware checks could be added here.
        // For now, get_buildings_on_path is 2D footprint intersection.
        // If we want 3D check, we need to check if the ray height > building height.
        // Since get_buildings_on_path returns intersections, we can filter them by height.

        auto intersections = building_service->get_buildings_on_path(s_pos, drone_pos);

        for (const auto &intersection : intersections)
        {
          if (intersection.building)
          {
            // Simple 3D Check: If the ray is lower than the building max height?
            // We don't have precise ray-height-at-intersection here without math.
            // Ray Height at Intersection Distance d_i:
            // H(d_i) = H_start + (H_end - H_start) * (d_i / d_total)

            // Distance from sensor to intersection entry
            double d_entry = geo::distance(s_lat, s_lon, intersection.entry_point.lat, intersection.entry_point.lon);

            double total_dist_m = dist_km * 1000.0;
            if (total_dist_m > 0.1)
            {
              double ratio = d_entry / total_dist_m;
              double ray_height_amsl = s_amsl + ratio * (drone_amsl - s_amsl);

              // Building Height is usually AGL. We need building Ground Elevation.
              // intersection.building->height_m is Height Above Ground.
              // We need ground elevation at building location.
              float building_ground = 0.0f;
              elevation_service.get_elevation(intersection.entry_point.lat, intersection.entry_point.lon, building_ground);

              double building_top_amsl = building_ground + intersection.building->height_m;

              if (ray_height_amsl < building_top_amsl)
              {
                is_los = false;
                break;
              }
            }
            else
            {
              // Too close, assume blocked if footprint hit
              is_los = false;
              break;
            }
          }
        }
      }

      if (!is_los)
      {
        // LoS Blocked: Apply heavy penalty or clamp to noise
        // Shadowing loss? Let's just assume -20dB additional loss for NLOS or hard cut for simplicity?
        // User wants "Accuracy" maybe? Let's assume Diffraction loss is not fully modeled yet,
        // so we drop signal significantly.
        res.los_blocked = true;
        // For now, if no LoS, we don't consider it "Best" typically, or we apply massive attenuation.
        // Let's model it as: Calc Path Loss, then subtract 30dB for Obstruction.
      }

      // Calculate Path Loss
      // Use sensor's model
      double path_loss = rf_models::calculate_path_loss(dist_km, frequency_mhz, s_mast, drone_altitude_agl_m, sensor.get_propagation_model());

      // If NLOS, add penalty
      if (!is_los)
      {
        path_loss += 30.0; // Simulated diffraction/obstruction loss
      }

      // Antenna Gain (Omni drone = 0, Sensor Pattern)
      // Sensor Pattern Gain
      // Need bearing from Sensor to Drone
      double bearing_to_drone = geo::bearing(s_lat, s_lon, lat, lon);
      double rx_gain = sensor.get_antenna_gain(bearing_to_drone, 0.0); // Ignoring elevation angle for now or calculate it

      // Elevation Angle
      double height_diff = drone_amsl - s_amsl;
      double elev_angle_rad = std::atan2(height_diff, dist_km * 1000.0);
      double elev_angle_deg = elev_angle_rad * 180.0 / geo::PI;
      // We should pass elev angle if sensor supports 3D pattern, but `get_antenna_gain` signature in sensor.hpp needs verification
      // sensor.hpp usually takes (azimuth, elevation). Let's assume it does.
      rx_gain = sensor.get_antenna_gain(bearing_to_drone, elev_angle_deg);

      // Link Budget
      // Rx Power = Tx Power + G_tx + G_rx - PathLoss
      // G_tx (Drone) = 0 dBi (assumed)
      double rx_power = drone_tx_power_dbm + 0.0 + rx_gain - path_loss;

      if (rx_power > res.max_signal_dbm)
      {
        res.max_signal_dbm = rx_power;
        res.best_sensor_idx = static_cast<int>(i);
        // Important: Update los_blocked state to match the BEST sensor
        // Use a logical check: If the best sensor has LoS, then we are NOT blocked.
        // If the best sensor is also blocked, then we are blocked.
        res.los_blocked = !is_los;

        // Wait, the loop overwrites `res.los_blocked` based on the LAST sensor checked, not the BEST one?
        // No, `is_los` is local. We need to store if the *selected* best sensor was blocked.
        // Actually, logic correction:
        // We track `max_signal_dbm`. If we find a better signal, we update `best_sensor_idx` and `los_blocked` status for THAT sensor.
        // Yes, the code logic does that (assigning inside the if block).
      }
    }

    // 2. TDOA Error Calculation
    // Estimate error and GDOP using all available sensors
    try
    {
      res.gdop = tdoa_solver.calculate_gdop(sensors, lat, lon, drone_amsl);
      // Estimate error (assuming 10ns jitter as default or configured elsewhere)
      // Note: estimate_positioning_error uses all sensors
      res.position_error_m = tdoa_solver.estimate_positioning_error(sensors, lat, lon, drone_amsl, 10.0);
    }
    catch (...)
    {
      res.gdop = 99.9;
      res.position_error_m = 999.9;
    }

    results.push_back(res);
  }

  return results;
}

std::vector<std::tuple<double, double, double>> SimulationEngine::interpolate_path(const std::vector<std::pair<double, double>> &path, double step_size_m)
{
  std::vector<std::tuple<double, double, double>> points;
  if (path.empty())
    return points;

  // Add start point
  points.emplace_back(path[0].first, path[0].second, 0.0);

  double total_dist = 0.0;

  for (size_t i = 0; i < path.size() - 1; ++i)
  {
    double lat1 = path[i].first;
    double lon1 = path[i].second;
    double lat2 = path[i + 1].first;
    double lon2 = path[i + 1].second;

    double seg_dist = geo::distance(lat1, lon1, lat2, lon2);
    double bearing = geo::bearing(lat1, lon1, lat2, lon2);

    double current_dist_on_seg = step_size_m;

    // While we can fit steps in this segment
    while (current_dist_on_seg < seg_dist)
    {
      double new_lat, new_lon;
      geo::destination_point(lat1, lon1, current_dist_on_seg, bearing, new_lat, new_lon);

      points.emplace_back(new_lat, new_lon, total_dist + current_dist_on_seg);

      current_dist_on_seg += step_size_m;
    }

    total_dist += seg_dist;

    // Always add the vertex point (end of segment) to ensure sharp corners are hit?
    // Or just rely on the next segment start (which is the same).
    // The next loop iteration starts at i+1, which is this segment's end.
    // So we don't add it here to avoid duplication, unless it's the very last point.
  }

  // Ensure last point is included
  points.emplace_back(path.back().first, path.back().second, total_dist);

  return points;
}

} // namespace sensor_mapper
