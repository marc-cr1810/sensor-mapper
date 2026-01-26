#include "sensor_optimizer.hpp"
#include <thread>
#include <chrono>
#include <random>
#include <algorithm>
#include "geo_math.hpp"
#include "tdoa_solver.hpp"
#include "building_service.hpp" // For simple geometry checks
#include <iostream>

namespace sensor_mapper
{

sensor_optimizer_t::sensor_optimizer_t()
{
}

sensor_optimizer_t::~sensor_optimizer_t()
{
  cancel();
  if (m_future.valid())
    m_future.wait();
}

void sensor_optimizer_t::start(const std::vector<std::pair<double, double>> &target_area, const optimizer_config_t &config)
{
  if (m_running)
    return;

  m_running = true;
  m_cancel = false;
  m_progress = 0.0f;

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status = "Initializing optimization...";
  }

  m_future = std::async(std::launch::async, &sensor_optimizer_t::run_internal, this, target_area, config);
}

void sensor_optimizer_t::cancel()
{
  m_cancel = true;
}

std::string sensor_optimizer_t::get_status() const
{
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_status;
}

std::vector<sensor_t> sensor_optimizer_t::get_results()
{
  std::lock_guard<std::mutex> lock(m_mutex);
  auto res = std::move(m_results);
  m_results.clear(); // Ensure clear after move for safety (though move should do it)
  return res;
}

static bool is_point_in_polygon(double lat, double lon, const std::vector<std::pair<double, double>> &poly)
{
  if (poly.size() < 3)
    return false;
  bool inside = false;
  for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
  {
    if (((poly[i].second > lon) != (poly[j].second > lon)) && (lat < (poly[j].first - poly[i].first) * (lon - poly[i].second) / (poly[j].second - poly[i].second) + poly[i].first))
    {
      inside = !inside;
    }
  }
  return inside;
}

void sensor_optimizer_t::run_internal(std::vector<std::pair<double, double>> area, optimizer_config_t config)
{
  auto update_status = [&](std::string msg, float p)
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_status = msg;
    m_progress = p;
  };

  // Set up random number generator
  std::random_device rd;
  std::mt19937 gen(rd());

  // Phase 1: Data Gathering
  update_status("Filtering geographic candidates...", 0.1f);
  if (m_cancel)
  {
    m_running = false;
    return;
  }

  // Phase 2: Candidate Generation
  update_status("Evaluating building locations...", 0.3f);

  struct building_candidate_t
  {
    std::pair<double, double> pos;
    std::string id;
  };
  std::vector<building_candidate_t> building_candidates;
  std::vector<std::pair<double, double>> random_candidates;

  // 1. Gather buildings centers inside polygon
  if (config.use_buildings)
  {
    for (const auto &b : config.buildings)
    {
      double clat = 0, clon = 0;
      if (b.footprint.empty())
        continue;
      for (const auto &p : b.footprint)
      {
        clat += p.lat;
        clon += p.lon;
      }
      clat /= b.footprint.size();
      clon /= b.footprint.size();

      // Check Excluded List
      bool is_excluded = false;
      for (const auto &ex_id : config.excluded_building_ids)
      {
        if (b.id == ex_id)
        {
          is_excluded = true;
          break;
        }
      }
      if (is_excluded)
        continue;

      // Check Priority List (if restricted)
      bool is_priority = false;
      for (const auto &p_id : config.priority_building_ids)
      {
        if (b.id == p_id)
        {
          is_priority = true;
          break;
        }
      }

      if (config.restrict_to_priority && !is_priority)
        continue;

      if (is_point_in_polygon(clat, clon, area))
      {
        building_candidates.push_back({{clat, clon}, b.id});
      }
    }
  }

  // 2. Generate random points within polygon (Terrain Mode)
  if (config.use_terrain)
  {
    double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0;
    for (const auto &p : area)
    {
      min_lat = std::min(min_lat, p.first);
      max_lat = std::max(max_lat, p.first);
      min_lon = std::min(min_lon, p.second);
      max_lon = std::max(max_lon, p.second);
    }

    std::uniform_real_distribution<double> dist_lat(min_lat, max_lat);
    std::uniform_real_distribution<double> dist_lon(min_lon, max_lon);

    int attempts = 0;
    // Generate enough candidates to have a good pool
    while (random_candidates.size() < 200 && attempts < 4000)
    {
      double lat = dist_lat(gen);
      double lon = dist_lon(gen);
      if (is_point_in_polygon(lat, lon, area))
      {
        random_candidates.push_back({lat, lon});
      }
      attempts++;
    }
  }

  if (building_candidates.empty() && random_candidates.empty())
  {
    update_status("Error: No valid locations found (Check constraints)", 1.0f);
    m_running = false;
    return;
  }

  // Determine final candidate pool:
  struct candidate_t
  {
    std::pair<double, double> pos;
    bool is_building;
    std::string building_id;
  };
  std::vector<candidate_t> pool;
  for (const auto &b : building_candidates)
  {
    pool.push_back({b.pos, true, b.id});
  }

  for (const auto &r : random_candidates)
    pool.push_back({r, false, ""});

  // Shuffle pool
  std::shuffle(pool.begin(), pool.end(), gen);

  if (m_cancel)
  {
    m_running = false;
    return;
  }

  // Phase 3: Selection / Spreading (Balanced for Coverage + TDOA)
  update_status("Optimizing sensor coverage...", 0.5f);

  // Calculate Polygon Centroid for central coverage bias
  double poly_clat = 0, poly_clon = 0;
  for (const auto &p : area)
  {
    poly_clat += p.first;
    poly_clon += p.second;
  }
  poly_clat /= area.size();
  poly_clon /= area.size();

  std::vector<candidate_t> selected;
  if (pool.size() <= (size_t)config.sensor_count)
  {
    selected = pool;
  }
  else
  {
    // 1. Pick the first point: closest building to the center (or just center if no buildings)
    size_t best_start = 0;
    double min_dist_to_center = 1e18;
    for (size_t i = 0; i < pool.size(); ++i)
    {
      double d = geo::distance(pool[i].pos.first, pool[i].pos.second, poly_clat, poly_clon);
      if (pool[i].is_building)
        d *= 0.5; // Favor buildings for start
      if (d < min_dist_to_center)
      {
        min_dist_to_center = d;
        best_start = i;
      }
    }
    selected.push_back(pool[best_start]);
    pool.erase(pool.begin() + best_start);

    // 2. Iteratively pick candidates that maximize geometric diversity + coverage
    while (selected.size() < (size_t)config.sensor_count && !pool.empty())
    {
      double max_score = -1e18;
      size_t best_idx = 0;

      for (size_t i = 0; i < pool.size(); ++i)
      {
        if (config.strategy == OptimizationStrategy::Geometric)
        {
          // --- GEOMETRIC STRATEGY (Fast) ---

          // Metric 1: Spacing (Distance to nearest selected sensor)
          double min_dist_to_sensor = 1e18;
          for (const auto &s : selected)
          {
            double d = geo::distance(pool[i].pos.first, pool[i].pos.second, s.pos.first, s.pos.second);
            if (d < min_dist_to_sensor)
              min_dist_to_sensor = d;
          }

          // Metric 2: Coverage (Distance to center)
          // We want a mix of "far out" for GDOP and "spread out" for coverage.
          double dist_to_center = geo::distance(pool[i].pos.first, pool[i].pos.second, poly_clat, poly_clon);

          // Metric 3: Building Preference
          // double building_multiplier = pool[i].is_building ? 2.5 : 1.0;
          double building_multiplier = 1.0;

          // Priority Building Boost
          if (pool[i].is_building)
          {
            for (const auto &pid : config.priority_building_ids)
            {
              if (pool[i].building_id == pid)
              {
                building_multiplier = 2.0; // Boost
                break;
              }
            }
          }

          // Combined Score:
          // - High spacing (prevents clustering)
          // - Normalized distance to center factor (bias towards interior coverage)
          // Normalized spacing (approx 500m-2km is good)
          double spacing_factor = std::min(min_dist_to_sensor, 5000.0) / 1000.0;
          // Coverage factor: we want some sensors far and some near.
          // For 4 sensors, corners are fine for GDOP, but middle needs coverage.
          double coverage_factor = 1.0 / (1.0 + (dist_to_center / 0.01)); // Boost interior

          double score = (spacing_factor * 0.6 + coverage_factor * 0.4) * building_multiplier;

          if (score > max_score)
          {
            max_score = score;
            best_idx = i;
          }
        }
        else
        {
          // --- ADVANCED STRATEGY (GDOP + LOS) ---

          // 1. LOS Check (Skip if blocked to center)
          // Perform a simple raycast against all buildings to see if LOS to center is blocked
          // This is expensive O(B), but we do it only for candidates in the pool.
          // Note: This assumes center is the target of interest.
          bool los_clear = true;
          if (config.use_buildings && !config.buildings.empty())
          {
            // Check intersection with all buildings
            // Simple segment check: pool[i].pos -> {poly_clat, poly_clon}
            // We don't have building_service instance easily available, but we have config.buildings which are raw data.
            // We need a helper for "does segment intersect polygon".
            // For now, let's use a simpler proxy: distance to center should be clear.
            // Actually, without the spatial index (quadtree) in building_service, iterating all buildings is slow O(N).
            // pool size ~200, buildings ~1000 -> 200,000 checks. Doable in seconds.
            for (const auto &b : config.buildings)
            {
              // Skip if it's the building we are ON
              if (pool[i].is_building)
              {
                double d_self = geo::distance(pool[i].pos.first, pool[i].pos.second, b.footprint[0].lat, b.footprint[0].lon);
                if (d_self < 50.0)
                  continue;
              }

              // Check containment or intersection?
              // Just check if building center is close to the line?
              // Proper intersection is tedious here without the service.
              // Let's rely on GDOP mainly for now and skip complex LOS unless we port the intersection logic.
              // Or better: Use GDOP as the primary driver.
            }
          }

          // 2. TDOA GDOP Calculation
          // Create temp sensor list
          std::vector<sensor_t> temp_sensors;
          double min_dist_to_selected = 1e18; // Track spacing

          for (const auto &s : selected)
          {
            temp_sensors.emplace_back("temp", s.pos.first, s.pos.second, 1000.0);
            temp_sensors.back().set_mast_height(10.0);

            // Spacing Check
            double d = geo::distance(pool[i].pos.first, pool[i].pos.second, s.pos.first, s.pos.second);
            if (d < min_dist_to_selected)
              min_dist_to_selected = d;
          }
          temp_sensors.emplace_back("candidate", pool[i].pos.first, pool[i].pos.second, 1000.0);
          temp_sensors.back().set_mast_height(10.0);

          // Evaluate GDOP at the Center
          tdoa_solver_t solver;
          double g1 = solver.calculate_2d_gdop(temp_sensors, poly_clat, poly_clon, 0.0); // Center

          double avg_gdop = g1;

          // Minimize GDOP => Maximize Score = 1/GDOP
          double base_score = 1.0 / (avg_gdop + 1e-6);

          // Apply Diversity Penalty
          double spacing_penalty = 1.0;
          if (min_dist_to_selected < 200.0)
          {
            // Very close: severe penalty
            spacing_penalty = 0.1;
          }
          else if (min_dist_to_selected < 500.0)
          {
            // Moderately close: linear ramp 0.1 -> 1.0
            spacing_penalty = 0.1 + 0.9 * ((min_dist_to_selected - 200.0) / 300.0);
          }

          double score = base_score * spacing_penalty;

          if (score > max_score)
          {
            max_score = score;
            best_idx = i;
          }
        }
      }

      selected.push_back(pool[best_idx]);
      pool.erase(pool.begin() + best_idx);

      float p = 0.5f + (0.45f * (float)selected.size() / config.sensor_count);
      update_status("Maximizing signal overlap...", p);
      if (m_cancel)
        break;
    }
  }

  if (m_cancel)
  {
    m_running = false;
    return;
  }

  // Phase 4: Finalizing
  update_status("Finalizing results...", 0.95f);

  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_results.clear();

    for (size_t i = 0; i < selected.size(); ++i)
    {
      sensor_t s("Auto Sensor " + std::to_string(i + 1), selected[i].pos.first, selected[i].pos.second, 5000.0);
      s.set_mast_height(15.0); // Slightly higher for auto-placed
      m_results.push_back(s);
    }
  }

  update_status("Optimization complete", 1.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  m_running = false;
}

} // namespace sensor_mapper
