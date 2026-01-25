#pragma once

#include <vector>
#include <string>
#include <atomic>
#include <future>
#include <mutex>
#include "../core/sensor.hpp"
#include "../core/building_service.hpp"

namespace sensor_mapper
{

struct optimizer_config_t
{
  bool use_buildings = false;
  bool use_terrain = false;
  int sensor_count = 3;

  // Data for the background thread
  std::vector<building_t> buildings;
};

class sensor_optimizer_t
{
public:
  sensor_optimizer_t();
  ~sensor_optimizer_t();

  // Start optimization in background thread
  void start(const std::vector<std::pair<double, double>> &target_area, const optimizer_config_t &config);

  // Check if optimization is currently running
  bool is_running() const
  {
    return m_running;
  }

  // Get progress (0.0 to 1.0)
  float get_progress() const
  {
    return m_progress;
  }

  // Get current status message
  std::string get_status() const;

  // Cancel current operation
  void cancel();

  // Get results if finished
  std::vector<sensor_t> get_results();

private:
  void run_internal(std::vector<std::pair<double, double>> area, optimizer_config_t config);

  std::atomic<bool> m_running{false};
  std::atomic<bool> m_cancel{false};
  std::atomic<float> m_progress{0.0f};

  mutable std::mutex m_mutex;
  std::string m_status;
  std::vector<sensor_t> m_results;

  std::future<void> m_future;
};

} // namespace sensor_mapper
