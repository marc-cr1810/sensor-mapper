#pragma once

#include <vector>
#include <set>
#include <functional>
#include "core/sensor.hpp"
#include "core/elevation_service.hpp"
#include "ui/map_widget.hpp"
#include "ui/antenna_pattern_ui.hpp"

namespace sensor_mapper
{

class AppUI
{
public:
  AppUI();
  ~AppUI() = default;

  // Apply custom style/theme
  void setup_style();

  // Main render function
  void render(map_widget_t &map, std::vector<sensor_t> &sensors, std::set<int> &selected_indices, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state, std::function<void()> on_exit);

private:
  void render_main_menu(std::vector<sensor_t> &sensors, std::set<int> &selected_indices, map_widget_t &map, elevation_service_t &elevation_service, std::function<void()> on_exit);
  void render_sensor_config(std::vector<sensor_t> &sensors, std::set<int> &selected_indices, map_widget_t &map, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state);
  void render_map_view_controls(map_widget_t &map);
  void render_elevation_data(elevation_service_t &elevation_service, map_widget_t &map);
  void render_tdoa_analysis(map_widget_t &map, const std::vector<sensor_t> &sensors);

  // UI State
  bool m_show_sensor_config = true;
  bool m_show_map_view = true; // Still keep the window boolean just in case we use it, though we might simplify
  bool m_show_elevation_data = true;
  bool m_show_tdoa_analysis = true;
};

} // namespace sensor_mapper
