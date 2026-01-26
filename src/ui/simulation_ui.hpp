#pragma once

#include "imgui.h"
#include "map_widget.hpp"
#include "../core/simulation_engine.hpp"
#include "../core/sensor.hpp"
#include <vector>
#include <memory>

namespace sensor_mapper
{

class SimulationUI
{
public:
  SimulationUI();

  // Render the panel
  // Returns true if open, false if closed by user
  bool render(bool &is_open, map_widget_t &map, const std::vector<sensor_t> &sensors, elevation_service_t &elevation_service, building_service_t *building_service);

private:
  std::unique_ptr<SimulationEngine> m_sim_engine;

  // Settings
  float m_drone_tx_power_dbm = 20.0f;
  float m_drone_alt_m = 50.0f;
  float m_step_size_m = 10.0f;
  float m_frequency_mhz = 2400.0f;

  // Results
  std::vector<SimulationResult> m_last_results;
  bool m_has_results = false;

  // Graphing helpers
  void render_results_graph(map_widget_t &map);
};

} // namespace sensor_mapper
