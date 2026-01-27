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

// --- Intersection Helpers ---

// Check if segments (p1, p2) and (p3, p4) intersect
static bool segments_intersect(std::pair<double, double> p1, std::pair<double, double> p2, std::pair<double, double> p3, std::pair<double, double> p4)
{
  auto ccw = [](std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c) { return (c.second - a.second) * (b.first - a.first) > (b.second - a.second) * (c.first - a.first); };
  return (ccw(p1, p3, p4) != ccw(p2, p3, p4)) && (ccw(p1, p2, p3) != ccw(p1, p2, p4));
}

// Check if Line of Sight is blocked by any building
// Returns true if blocked
static bool is_los_blocked(const std::pair<double, double> &start, const std::pair<double, double> &end, const std::vector<building_t> &buildings)
{
  // Simple bounding box check could go here if we had spatial index, but we iterate all for now.
  // Optimization: Pre-filter buildings near the line?
  // User noted it's expensive, but O(N) for N=1000 is tiny (microseconds).

  for (const auto &b : buildings)
  {
    if (b.footprint.size() < 3)
      continue;

    // Check intersection with any wall of the building
    for (size_t i = 0; i < b.footprint.size(); ++i)
    {
      size_t j = (i + 1) % b.footprint.size();
      std::pair<double, double> w1 = {b.footprint[i].lat, b.footprint[i].lon};
      std::pair<double, double> w2 = {b.footprint[j].lat, b.footprint[j].lon};

      if (segments_intersect(start, end, w1, w2))
      {
        return true; // Blocked
      }
    }
  }
  return false;
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
    std::cout << "Optimizer: Evaluating " << config.buildings.size() << " buildings..." << std::endl;
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
    std::cout << "Optimizer: Found " << building_candidates.size() << " buildings inside target area." << std::endl;
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

  std::shuffle(pool.begin(), pool.end(), gen);

  if (m_cancel)
  {
    m_running = false;
    return;
  }

  // Phase 3: Selection / Spreading (Balanced for Coverage + TDOA)
  update_status("Optimizing sensor coverage...", 0.5f);

  // Calculate Polygon Centroid
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
    // 1. Pick the first point
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

    // Pre-calculate target points once
    static std::vector<std::pair<double, double>> target_points;
    target_points.clear();
    target_points.push_back({poly_clat, poly_clon}); // Center

    // Add corners (Min/Max lat/lon of area)
    double min_lat = 1e9, max_lat = -1e9, min_lon = 1e9, max_lon = -1e9;
    for (const auto &p : area)
    {
      min_lat = std::min(min_lat, p.first);
      max_lat = std::max(max_lat, p.first);
      min_lon = std::min(min_lon, p.second);
      max_lon = std::max(max_lon, p.second);
    }
    target_points.push_back({min_lat, min_lon});
    target_points.push_back({min_lat, max_lon});
    target_points.push_back({max_lat, min_lon});
    target_points.push_back({max_lat, max_lon});

    // 2. Iteratively pick candidates
    while (selected.size() < (size_t)config.sensor_count && !pool.empty())
    {
      double max_score = -1e18;
      size_t best_idx = 0;

      for (size_t i = 0; i < pool.size(); ++i)
      {
        if (config.objective == OptimizationObjective::FastGeometric)
        {
          // --- GEOMETRIC STRATEGY (Fast) ---
          double min_dist_to_sensor = 1e18;
          for (const auto &s : selected)
          {
            double d = geo::distance(pool[i].pos.first, pool[i].pos.second, s.pos.first, s.pos.second);
            if (d < min_dist_to_sensor)
              min_dist_to_sensor = d;
          }

          double dist_to_center = geo::distance(pool[i].pos.first, pool[i].pos.second, poly_clat, poly_clon);
          double building_multiplier = 1.0;

          if (pool[i].is_building)
          {
            for (const auto &pid : config.priority_building_ids)
            {
              if (pool[i].building_id == pid)
              {
                building_multiplier = 2.0;
                break;
              }
            }
          }

          double spacing_factor = std::min(min_dist_to_sensor, 5000.0) / 1000.0;
          double coverage_factor = 1.0 / (1.0 + (dist_to_center / 0.01));
          double score = (spacing_factor * 0.6 + coverage_factor * 0.4) * building_multiplier;

          if (score > max_score)
          {
            max_score = score;
            best_idx = i;
          }
        }
        else
        {
          // --- ADVANCED OBJECTIVES ---
          double score = 0.0;

          // 1. Coverage Score (LoS Percentage)
          double cov_score = 0.0;
          if (config.objective == OptimizationObjective::Coverage || config.objective == OptimizationObjective::Balanced)
          {
            int visible_count = 0;
            if (config.use_buildings && !config.buildings.empty())
            {
              for (const auto &tp : target_points)
              {
                if (!is_los_blocked(pool[i].pos, tp, config.buildings))
                  visible_count++;
              }
              cov_score = (double)visible_count / (double)target_points.size();
            }
            else
            {
              cov_score = 1.0; // Assume clear path if no buildings
            }
          }

          // 2. TDOA/GDOP Score
          double gdop_score = 0.0;
          if (config.objective == OptimizationObjective::TDOA || config.objective == OptimizationObjective::Balanced)
          {
            std::vector<sensor_t> temp_sensors;
            for (const auto &s : selected)
            {
              temp_sensors.emplace_back("temp", s.pos.first, s.pos.second, 1000.0);
              temp_sensors.back().set_mast_height(10.0);
            }
            temp_sensors.emplace_back("candidate", pool[i].pos.first, pool[i].pos.second, 1000.0);

            // Basic GDOP at center
            tdoa_solver_t solver;
            double g = solver.calculate_2d_gdop(temp_sensors, poly_clat, poly_clon, 0.0); // Center
            double avg_gdop = g;
            double base_score = 1.0 / (avg_gdop + 1e-6);
            gdop_score = base_score;

            // Normalize GDOP score roughly (1.0 = good GDOP ~ 1.5, 0.1 = bad GDOP ~ 10)
            if (gdop_score > 1.0)
              gdop_score = 1.0;
          }

          // 3. Spacing Constraint (Always applies to some degree)
          double min_dist = 1e18;
          for (const auto &s : selected)
          {
            double d = geo::distance(pool[i].pos.first, pool[i].pos.second, s.pos.first, s.pos.second);
            if (d < min_dist)
              min_dist = d;
          }
          double spacing_penalty = 1.0;
          if (min_dist < 200.0)
            spacing_penalty = 0.1;
          else if (min_dist < 500.0)
            spacing_penalty = 0.1 + 0.9 * ((min_dist - 200.0) / 300.0);

          // Final Scoring
          if (config.objective == OptimizationObjective::Coverage)
          {
            // Prioritize coverage, then spacing
            score = cov_score * spacing_penalty;
          }
          else if (config.objective == OptimizationObjective::TDOA)
          {
            // Prioritize TDOA, enforce minimal LoS (must see center)
            bool center_visible = true;
            // Actually re-check center specifically? target_points[0] is center.
            if (config.use_buildings && !config.buildings.empty() && is_los_blocked(pool[i].pos, target_points[0], config.buildings))
              center_visible = false;

            if (!center_visible)
              score = 0.0;
            else
              score = gdop_score * spacing_penalty;
          }
          else
          {
            // Balanced: Mix Coverage and GDOP
            score = (0.6 * cov_score + 0.4 * gdop_score) * spacing_penalty;
          }

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
      s.set_mast_height(15.0);
      s.set_locked(true);
      m_results.push_back(s);
    }
  }

  update_status("Optimization complete", 1.0f);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  m_running = false;
}

} // namespace sensor_mapper
