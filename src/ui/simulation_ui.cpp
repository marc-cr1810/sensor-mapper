#include "simulation_ui.hpp"
#include <vector>
#include <portable-file-dialogs.h>

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

  // Update playback
  map.update_playback(ImGui::GetIO().DeltaTime);

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
    ImGui::Text("Visualization Mode:");
    if (ImGui::RadioButton("Signal Strength", m_metric_mode == MetricMode::SignalStrength))
      m_metric_mode = MetricMode::SignalStrength;
    if (ImGui::RadioButton("Position Accuracy", m_metric_mode == MetricMode::PositionAccuracy))
      m_metric_mode = MetricMode::PositionAccuracy;

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

    // Flight Log Import
    ImGui::Spacing();
    ImGui::Separator();
    ImGui::TextDisabled("IMPORT LOG");
    if (ImGui::Button("Load Log File...", ImVec2(-1, 0)))
    {
      auto selection = pfd::open_file("Open Flight Log", ".", {"Flight Logs", "*.gpx *.csv", "All Files", "*"}, pfd::opt::none).result();
      if (!selection.empty())
      {
        std::string path = selection[0];
        if (map.import_drone_path(path))
        {
          // Reset results
          m_has_results = false;
          m_last_results.clear();
        }
      }
    }

    ImGui::Spacing();
    ImGui::Separator();

    // Playback Control
    if (!path.empty())
    {
      ImGui::Spacing();
      ImGui::TextDisabled("PLAYBACK");

      auto &playback = map.get_playback_state();

      // Time Slider
      float t = static_cast<float>(playback.current_time_sec);
      if (ImGui::SliderFloat("Time", &t, 0.0f, static_cast<float>(playback.duration_sec), "%.1fs"))
      {
        map.set_playback_pos(t / playback.duration_sec);
      }

      // Controls
      if (ImGui::Button(playback.is_playing ? "Pause" : "Play", ImVec2(80, 0)))
      {
        playback.is_playing = !playback.is_playing;
      }
      ImGui::SameLine();
      if (ImGui::Button("Stop", ImVec2(60, 0)))
      {
        playback.is_playing = false;
        playback.current_time_sec = 0.0;
        map.set_playback_pos(0.0);
      }
      ImGui::SameLine();
      ImGui::Checkbox("Loop", &playback.loop);

      ImGui::Text("Speed:");
      ImGui::SameLine();
      if (ImGui::Button("1x"))
        playback.speed_multiplier = 1.0f;
      ImGui::SameLine();
      if (ImGui::Button("2x"))
        playback.speed_multiplier = 2.0f;
      ImGui::SameLine();
      if (ImGui::Button("5x"))
        playback.speed_multiplier = 5.0f;
      ImGui::SameLine();
      if (ImGui::Button("10x"))
        playback.speed_multiplier = 10.0f;
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

    // Push visual results to map (every frame or when changed?)
    // Originally it was only on RUN. But if we switch modes, we need to push again.
    // Let's push every frame or check if dirty. For Immediate GUI, doing it here is fine if array isn't massive.
    if (m_has_results)
    {
      std::vector<map_widget_t::drone_path_result_visual_t> visual_res;
      visual_res.reserve(m_last_results.size());
      for (const auto &r : m_last_results)
      {
        double val = (m_metric_mode == MetricMode::SignalStrength) ? r.max_signal_dbm : r.position_error_m;
        visual_res.push_back({r.interpolated_lat, r.interpolated_lon, val, r.los_blocked});
      }
      map.set_drone_path_results(visual_res, (m_metric_mode == MetricMode::SignalStrength) ? 0 : 1);
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
  if (m_metric_mode == MetricMode::SignalStrength)
    ImGui::Text("Connectivity Profile");
  else
    ImGui::Text("TDOA Accuracy Profile");

  if (m_last_results.empty())
    return;

  // Prepare data for PlotLines
  std::vector<float> data;
  data.reserve(m_last_results.size());

  float min_v = 1e9f;
  float max_v = -1e9f;

  for (const auto &r : m_last_results)
  {
    float val = (m_metric_mode == MetricMode::SignalStrength) ? static_cast<float>(r.max_signal_dbm) : static_cast<float>(r.position_error_m);
    data.push_back(val);
    if (val < min_v)
      min_v = val;
    if (val > max_v)
      max_v = val;
  }

  // Bounds
  if (m_metric_mode == MetricMode::SignalStrength)
  {
    if (min_v > -60.0f)
      min_v = -60.0f;
    if (min_v < -120.0f)
      min_v = -120.0f;
    max_v = 0.0f;
  }
  else
  {
    if (min_v < 0.0f)
      min_v = 0.0f;
    if (max_v < 10.0f)
      max_v = 10.0f; // Minimal scale
    if (max_v > 200.0f)
      max_v = 200.0f; // Cap visualization
  }

  ImGui::PlotLines("##results_graph", data.data(), static_cast<int>(data.size()), 0, (m_metric_mode == MetricMode::SignalStrength) ? "Signal (dBm)" : "Error (m)", min_v, max_v, ImVec2(-1, 150));

  // Hover Logic
  int hover_idx = -1;
  if (ImGui::IsItemHovered())
  {
    float mouse_x = ImGui::GetMousePos().x;
    float graph_x = ImGui::GetItemRectMin().x;
    float graph_w = ImGui::GetItemRectSize().x;
    float t = (mouse_x - graph_x) / graph_w;
    int idx = static_cast<int>(t * (data.size() - 1));

    if (idx >= 0 && idx < static_cast<int>(data.size()))
    {
      hover_idx = idx;
    }
  }
  map.set_highlight_path_index(hover_idx);

  // Statistics
  if (m_metric_mode == MetricMode::SignalStrength)
  {
    int connected_count = 0;
    for (float s : data)
    {
      if (s > -90.0f)
        connected_count++;
    }
    float pct = 100.0f * (float)connected_count / (float)data.size();
    ImGui::Text("Connected Area (> -90dBm): %.1f%%", pct);
  }
  else
  {
    float mean_err = 0.0f;
    for (float v : data)
      mean_err += v;
    mean_err /= data.size();
    ImGui::Text("Mean Error: %.1f m", mean_err);
  }

  ImGui::Text("Min: %.1f", min_v);
  ImGui::Text("Max: %.1f", max_v);
}

} // namespace sensor_mapper
