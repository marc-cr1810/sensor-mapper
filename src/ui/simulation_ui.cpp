#include "simulation_ui.hpp"
#include <vector>

namespace sensor_mapper
{

SimulationUI::SimulationUI()
{
  m_sim_engine = std::make_unique<SimulationEngine>();
}

bool SimulationUI::render(bool &is_open, map_widget_t &map, const std::vector<sensor_t> &sensors, elevation_service_t &elevation_service, building_service_t *building_service)
{
  if (!is_open)
    return false;

  ImGui::SetNextWindowSize(ImVec2(400, 500), ImGuiCond_FirstUseEver);
  // Restore normal behavior: FirstUseEver allows user to move it.
  ImGui::SetNextWindowPos(ImVec2(100, 100), ImGuiCond_FirstUseEver);

  if (ImGui::Begin("Drone Simulation", &is_open))
  {
    ImGui::TextDisabled("CONFIGURATION");
    ImGui::DragFloat("Drone Tx Power (dBm)", &m_drone_tx_power_dbm, 0.5f, -30.0f, 40.0f);
    ImGui::DragFloat("Drone Altitude (AGL)", &m_drone_alt_m, 1.0f, 1.0f, 500.0f);
    ImGui::DragFloat("Sim Step Size (m)", &m_step_size_m, 1.0f, 1.0f, 100.0f);
    ImGui::DragFloat("Freq (MHz)", &m_frequency_mhz, 1.0f, 100.0f, 6000.0f);

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    ImGui::TextDisabled("PATH");

    // Check if map is in drawing mode
    bool is_drawing = (map.get_tool_state() == map_widget_t::tool_state_t::DronePath_Draw);
    const auto &path = map.get_drone_path();

    if (is_drawing)
    {
      if (ImGui::Button("Finish Drawing"))
      {
        map.set_tool_state(map_widget_t::tool_state_t::Navigate);
      }
      ImGui::SameLine();
      ImGui::TextColored(ImVec4(1.f, 1.f, 0.f, 1.f), "Click on map to add points...");
    }
    else
    {
      if (ImGui::Button("Draw Path"))
      {
        map.start_drone_path();
      }
    }

    if (ImGui::Button("Clear Path"))
    {
      map.clear_drone_path();
      m_has_results = false;
      m_last_results.clear();
    }

    ImGui::Text("%zu Waypoints", path.size());

    ImGui::Spacing();
    ImGui::Separator();
    ImGui::Spacing();

    // Run Button
    if (!path.empty() && sensors.size() > 0)
    {
      if (ImGui::Button("RUN SIMULATION", ImVec2(-1, 40)))
      {
        // Execute Simulation
        m_last_results = m_sim_engine->run_simulation(path, sensors, elevation_service, building_service, static_cast<double>(m_step_size_m), static_cast<double>(m_drone_tx_power_dbm), static_cast<double>(m_drone_alt_m),
                                                      static_cast<double>(m_frequency_mhz));
        m_has_results = true;

        // Push visual results to map
        std::vector<map_widget_t::drone_path_result_visual_t> visual_res;
        visual_res.reserve(m_last_results.size());
        for (const auto &r : m_last_results)
        {
          visual_res.push_back({r.interpolated_lat, r.interpolated_lon, r.max_signal_dbm, r.los_blocked});
        }
        map.set_drone_path_results(visual_res);
      }
    }
    else if (sensors.empty())
    {
      ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "No sensors defined!");
    }
    else if (path.empty())
    {
      ImGui::TextDisabled("Draw a path to run.");
    }

    // Results Graph
    if (m_has_results && !m_last_results.empty())
    {
      render_results_graph(map);
    }
  }
  ImGui::End();

  return is_open;
}

void SimulationUI::render_results_graph(map_widget_t &map)
{
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Text("Connectivity Profile");

  if (m_last_results.empty())
    return;

  // Prepare data for PlotLines
  // We want Signal vs Distance
  std::vector<float> signal_data;
  signal_data.reserve(m_last_results.size());

  float min_s = 0.0f;
  float max_s = -200.0f;

  for (const auto &r : m_last_results)
  {
    float s = static_cast<float>(r.max_signal_dbm);
    signal_data.push_back(s);
    if (s < min_s)
      min_s = s;
    if (s > max_s)
      max_s = s;
  }

  // Bounds
  if (min_s > -60.0f)
    min_s = -60.0f;
  if (min_s < -120.0f)
    min_s = -120.0f;
  max_s = 0.0f; // Cap at 0 usually

  ImGui::PlotLines("##signalgraph", signal_data.data(), static_cast<int>(signal_data.size()), 0, "Signal (dBm)", -120.0f, 10.0f, ImVec2(-1, 150));

  // Hover Logic
  int hover_idx = -1;
  if (ImGui::IsItemHovered())
  {
    float mouse_x = ImGui::GetMousePos().x;
    float graph_x = ImGui::GetItemRectMin().x;
    float graph_w = ImGui::GetItemRectSize().x;
    float t = (mouse_x - graph_x) / graph_w;
    int idx = static_cast<int>(t * (signal_data.size() - 1));

    if (idx >= 0 && idx < static_cast<int>(signal_data.size()))
    {
      hover_idx = idx;
    }
  }
  map.set_highlight_path_index(hover_idx);

  // Statistics
  // % Connected (> -90 dBm)
  int connected_count = 0;
  for (float s : signal_data)
  {
    if (s > -90.0f)
      connected_count++;
  }
  float pct = 100.0f * (float)connected_count / (float)signal_data.size();

  ImGui::Text("Connected Area (> -90dBm): %.1f%%", pct);
  ImGui::Text("Min Signal: %.1f dBm", min_s);
  ImGui::Text("Max Signal: %.1f dBm", max_s);
}

} // namespace sensor_mapper
