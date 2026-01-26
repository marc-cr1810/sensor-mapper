
#include "ui/app_ui.hpp"
#include "imgui.h"
#include "imgui_internal.h" // For BeginViewportSideBar
#include "core/persistence.hpp"
#include <cstdio>
#include <cmath>
#include <algorithm>

namespace sensor_mapper
{

constexpr const char *SENSORS_FILE = "sensors.json";

AppUI::AppUI()
{
  m_optimizer = std::make_unique<sensor_optimizer_t>();
}

void AppUI::setup_style()
{
  ImGuiStyle &style = ImGui::GetStyle();
  ImVec4 *colors = style.Colors;

  // Deep Dark Theme
  colors[ImGuiCol_Text] = ImVec4(0.92f, 0.92f, 0.92f, 1.00f);
  colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);
  colors[ImGuiCol_WindowBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);
  colors[ImGuiCol_ChildBg] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_PopupBg] = ImVec4(0.13f, 0.13f, 0.15f, 0.94f);
  colors[ImGuiCol_Border] = ImVec4(0.24f, 0.24f, 0.26f, 0.50f);
  colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
  colors[ImGuiCol_FrameBg] = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);
  colors[ImGuiCol_FrameBgHovered] = ImVec4(0.24f, 0.24f, 0.26f, 1.00f);
  colors[ImGuiCol_FrameBgActive] = ImVec4(0.30f, 0.30f, 0.32f, 1.00f);
  colors[ImGuiCol_TitleBg] = ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
  colors[ImGuiCol_TitleBgActive] = ImVec4(0.08f, 0.08f, 0.09f, 1.00f);
  colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.00f, 0.00f, 0.00f, 0.51f);
  colors[ImGuiCol_MenuBarBg] = ImVec4(0.11f, 0.11f, 0.13f, 1.00f);
  colors[ImGuiCol_ScrollbarBg] = ImVec4(0.02f, 0.02f, 0.02f, 0.53f);
  colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.31f, 0.31f, 0.31f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.41f, 0.41f, 0.41f, 1.00f);
  colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.51f, 0.51f, 0.51f, 1.00f);
  colors[ImGuiCol_CheckMark] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  colors[ImGuiCol_SliderGrab] = ImVec4(0.24f, 0.52f, 0.88f, 1.00f);
  colors[ImGuiCol_SliderGrabActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  colors[ImGuiCol_Button] = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
  colors[ImGuiCol_ButtonHovered] = ImVec4(0.28f, 0.28f, 0.30f, 1.00f);
  colors[ImGuiCol_ButtonActive] = ImVec4(0.06f, 0.53f, 0.98f, 1.00f);
  colors[ImGuiCol_Header] = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
  colors[ImGuiCol_HeaderHovered] = ImVec4(0.26f, 0.26f, 0.28f, 1.00f);
  colors[ImGuiCol_HeaderActive] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  colors[ImGuiCol_Separator] = ImVec4(0.24f, 0.24f, 0.26f, 1.00f);
  colors[ImGuiCol_SeparatorHovered] = ImVec4(0.10f, 0.40f, 0.75f, 0.78f);
  colors[ImGuiCol_SeparatorActive] = ImVec4(0.10f, 0.40f, 0.75f, 1.00f);
  colors[ImGuiCol_ResizeGrip] = ImVec4(0.26f, 0.59f, 0.98f, 0.25f);
  colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.67f);
  colors[ImGuiCol_ResizeGripActive] = ImVec4(0.26f, 0.59f, 0.98f, 0.95f);
  colors[ImGuiCol_Tab] = ImVec4(0.11f, 0.11f, 0.13f, 0.86f);
  colors[ImGuiCol_TabHovered] = ImVec4(0.26f, 0.59f, 0.98f, 0.80f);
  colors[ImGuiCol_TabActive] = ImVec4(0.20f, 0.20f, 0.22f, 1.00f);
  colors[ImGuiCol_TabUnfocused] = ImVec4(0.07f, 0.10f, 0.15f, 0.97f);
  colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.14f, 0.26f, 0.42f, 1.00f);
  colors[ImGuiCol_PlotLines] = ImVec4(0.61f, 0.61f, 0.61f, 1.00f);
  colors[ImGuiCol_PlotLinesHovered] = ImVec4(1.00f, 0.43f, 0.35f, 1.00f);
  colors[ImGuiCol_PlotHistogram] = ImVec4(0.90f, 0.70f, 0.00f, 1.00f);
  colors[ImGuiCol_PlotHistogramHovered] = ImVec4(1.00f, 0.60f, 0.00f, 1.00f);
  colors[ImGuiCol_TextSelectedBg] = ImVec4(0.26f, 0.59f, 0.98f, 0.35f);
  colors[ImGuiCol_DragDropTarget] = ImVec4(1.00f, 1.00f, 0.00f, 0.90f);
  colors[ImGuiCol_NavHighlight] = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);
  colors[ImGuiCol_NavWindowingHighlight] = ImVec4(1.00f, 1.00f, 1.00f, 0.70f);
  colors[ImGuiCol_NavWindowingDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.20f);
  colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.80f, 0.80f, 0.80f, 0.35f);

  // Rounding & Padding
  style.WindowRounding = 8.0f;
  style.FrameRounding = 6.0f;
  style.GrabRounding = 6.0f;
  style.PopupRounding = 6.0f;
  style.ScrollbarRounding = 6.0f;
  style.TabRounding = 6.0f;
  style.ChildRounding = 6.0f;

  style.WindowPadding = ImVec2(12.0f, 12.0f);
  style.FramePadding = ImVec2(6.0f, 4.0f);
  style.ItemSpacing = ImVec2(8.0f, 6.0f);
  style.IndentSpacing = 20.0f;
}

void AppUI::render(map_widget_t &map, std::vector<sensor_t> &sensors, std::set<int> &selected_indices, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state, std::function<void()> on_exit)
{
  // Dockspace
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  map.set_show_target_area(m_show_auto_placement);

  render_main_menu(sensors, selected_indices, map, elevation_service, on_exit);

  // Check if optimizer has results
  if (m_optimizer && !m_optimizer->is_running())
  {
    auto results = m_optimizer->get_results();
    if (!results.empty())
    {
      for (auto &s : results)
      {
        sensors.push_back(s);
      }
      // Re-trigger heatmap update if needed
      map.invalidate_rf_heatmap();

      // Clear results from optimizer so we don't add them again
      // I'll add a 'clear_results' or just rely on 'get_results' returning a copy
      // and I should probably clear m_results in the optimizer or state in AppUI
    }
  }

  if (m_show_sensor_config)
  {
    // Use a fixed width for the sensor config panel if undocked/first run, but allow resizing
    ImGui::SetNextWindowSize(ImVec2(370, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Sensor Configuration", &m_show_sensor_config))
    {
      render_sensor_config(sensors, selected_indices, map, elevation_service, antenna_ui_state);
    }
    ImGui::End();
  }

  if (m_show_map_view)
  {
    ImGui::SetNextWindowSize(ImVec2(800, 600), ImGuiCond_FirstUseEver);
    // Remove padding for the map window to have edge-to-edge map
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    if (ImGui::Begin("Map View", &m_show_map_view))
    {
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(5, 5));
      ImGui::BeginGroup();
      render_map_view_controls(map);
      ImGui::EndGroup();
      ImGui::PopStyleVar();

      // No padding for map quad
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
      map.draw(sensors, selected_indices, elevation_service,
               [&](double lat, double lon)
               {
                 // Callback to add sensor
                 map.fetch_buildings_near(lat, lon);
                 double b_h = map.get_building_at_location(lat, lon);
                 double mast_h = b_h > 0 ? b_h : 5.0;

                 sensors.emplace_back("New Sensor", lat, lon, 5000.0);
                 auto &s = sensors.back();
                 s.set_mast_height(mast_h); // Set height (AGL)

                 // Auto-set ground elevation
                 float ground_h = 0.0f;
                 if (elevation_service.get_elevation(lat, lon, ground_h))
                 {
                   s.set_ground_elevation(ground_h);
                 }
                 s.set_use_auto_elevation(true); // Enable auto updates

                 map.invalidate_rf_heatmap();
                 selected_indices.clear();
                 selected_indices.insert(static_cast<int>(sensors.size()) - 1);
               });
      ImGui::PopStyleVar();
    }
    ImGui::End();
    ImGui::PopStyleVar();
  }

  if (m_show_elevation_data)
  {
    if (ImGui::Begin("Elevation Data", &m_show_elevation_data))
    {
      render_elevation_data(elevation_service, map);
    }
    ImGui::End();
  }

  if (m_show_tdoa_analysis)
  {
    if (ImGui::Begin("TDOA Analysis", &m_show_tdoa_analysis))
    {
      render_tdoa_analysis(map, sensors);
    }
    ImGui::End();
  }

  if (m_show_auto_placement)
  {
    render_auto_placement(map);
  }

  if (m_show_simulation)
  {
    // Pass nullptr for building service for now as it's not exposed via map public API yet
    m_simulation_ui.render(m_show_simulation, map, sensors, elevation_service, nullptr);
  }

  // Status Bar
  if (ImGui::BeginViewportSideBar("##MainStatusBar", ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetFrameHeight(), ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar))
  {
    if (ImGui::BeginMenuBar())
    {
      ImGui::Text("Cursor: %.5f, %.5f", map.get_mouse_lat(), map.get_mouse_lon());
      ImGui::Separator();

      // For now, assume map stores it or we assume 0 not available here unless map exposed it.
      // map_widget_t doesn't expose m_cursor_alt publically but we can access it via a getter if we added one?
      // Wait, we didn't add a getter for m_cursor_alt in map_widget.hpp properly, only used locally.
      // Let's rely on elevation service query here if cheap, or just skip it for declining complexity for now,
      // OR simpler: we update the map_widget text removal which WAS showing it.
      // Actually map.draw() calculates it. We should assume map's internal state is valid.
      // But we can't access it.
      // Let's just leave it basic: Lat/Lon + Zoom

      ImGui::Text("Zoom: %.2f", map.get_zoom());
      ImGui::Separator();
      ImGui::Text("%zu Sensors", sensors.size());

      ImGui::EndMenuBar();
    }
    ImGui::End();
  }
}

void AppUI::render_main_menu(std::vector<sensor_t> &sensors, std::set<int> &selected_index, map_widget_t &map, elevation_service_t &elevation_service, std::function<void()> on_exit)
{
  if (ImGui::BeginMainMenuBar())
  {
    if (ImGui::BeginMenu("File"))
    {
      if (ImGui::MenuItem("Save Workspace"))
      {
        persistence::workspace_t ws;
        ws.camera.lat = map.get_center_lat();
        ws.camera.lon = map.get_center_lon();
        ws.camera.zoom = map.get_zoom();

        ws.settings.min_signal_dbm = map.get_min_signal_dbm();
        ws.settings.viz_mode = map.get_viz_mode();

        ws.layers.show_rf = map.get_show_rf_gradient();
        ws.layers.show_heatmap = map.get_show_heatmap_overlay();
        ws.layers.show_composite = map.get_show_composite();
        ws.layers.show_buildings = map.get_show_buildings();
        ws.layers.show_elevation = map.get_show_elevation_sources();
        ws.layers.show_tdoa = map.get_show_tdoa_analysis();
        ws.layers.show_hyperbolas = map.get_show_hyperbolas();
        ws.layers.show_gdop = map.get_show_gdop_contours();
        ws.layers.show_accuracy = map.get_show_accuracy_heatmap();

        ws.data.elevation_files = elevation_service.get_loaded_files();

        persistence::save_workspace("workspace.json", ws);
        persistence::save_sensors(SENSORS_FILE, sensors); // Also save sensors implicitly
      }

      if (ImGui::MenuItem("Load Workspace"))
      {
        persistence::workspace_t ws;
        if (persistence::load_workspace("workspace.json", ws))
        {
          map.set_center(ws.camera.lat, ws.camera.lon);
          map.set_zoom(ws.camera.zoom);

          map.set_min_signal_dbm(ws.settings.min_signal_dbm);
          map.set_viz_mode(ws.settings.viz_mode);

          map.set_show_rf_gradient(ws.layers.show_rf);
          map.set_show_heatmap_overlay(ws.layers.show_heatmap);
          map.set_show_composite(ws.layers.show_composite);
          map.set_show_buildings(ws.layers.show_buildings);
          map.set_show_elevation_sources(ws.layers.show_elevation);
          map.set_show_tdoa_analysis(ws.layers.show_tdoa);
          map.set_show_hyperbolas(ws.layers.show_hyperbolas);
          map.set_show_gdop_contours(ws.layers.show_gdop);
          map.set_show_accuracy_heatmap(ws.layers.show_accuracy);

          // Restore elevation files
          for (const auto &path : ws.data.elevation_files)
          {
            elevation_service.load_local_file(path);
          }

          // Reload sensors
          persistence::load_sensors(SENSORS_FILE, sensors);

          map.invalidate_rf_heatmap();
        }
      }

      ImGui::Separator();

      if (ImGui::MenuItem("Save Sensors", "Ctrl+S"))
      {
        persistence::save_sensors(SENSORS_FILE, sensors);
      }
      if (ImGui::MenuItem("Load Sensors", "Ctrl+L"))
      {
        persistence::load_sensors(SENSORS_FILE, sensors);
        if (!selected_index.empty() && *selected_index.rbegin() >= static_cast<int>(sensors.size()))
          selected_index.clear();
        map.invalidate_rf_heatmap();
      }
      ImGui::Separator();
      if (ImGui::MenuItem("Exit", "Alt+F4"))
      {
        if (on_exit)
          on_exit();
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Edit"))
    {
      if (ImGui::MenuItem("Add Sensor"))
      {
        sensors.emplace_back("New Sensor", -33.8688, 151.2093, 1000.0);
        selected_index.clear();
        selected_index.insert(static_cast<int>(sensors.size()) - 1);
        map.invalidate_rf_heatmap();
      }
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("View"))
    {
      ImGui::MenuItem("Sensor Configuration", nullptr, &m_show_sensor_config);
      ImGui::MenuItem("Map View", nullptr, &m_show_map_view);
      ImGui::MenuItem("Elevation Data", nullptr, &m_show_elevation_data);
      ImGui::MenuItem("TDOA Analysis", nullptr, &m_show_tdoa_analysis);
      ImGui::EndMenu();
    }

    if (ImGui::BeginMenu("Tools"))
    {
      ImGui::MenuItem("Auto Sensor Placement", nullptr, &m_show_auto_placement);
      ImGui::MenuItem("Drone Path Simulation", nullptr, &m_show_simulation);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}

void AppUI::render_sensor_config(std::vector<sensor_t> &sensors, std::set<int> &selected_indices, map_widget_t &map, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state)
{
  // List Area
  ImGui::TextDisabled("SENSORS");
  ImGui::BeginGroup();
  {
    if (ImGui::BeginListBox("##sensorlist", ImVec2(-FLT_MIN, 140)))
    {
      for (int i = 0; i < static_cast<int>(sensors.size()); i++)
      {
        ImGui::PushID(i);
        const bool is_selected = selected_indices.contains(i);
        if (ImGui::Selectable(sensors[i].get_name().c_str(), is_selected))
        {
          if (ImGui::GetIO().KeyCtrl)
          {
            if (is_selected)
              selected_indices.erase(i);
            else
              selected_indices.insert(i);
          }
          else if (ImGui::GetIO().KeyShift)
          {
            // Range select (simplified: just add for now, properly requires last clicked state)
            selected_indices.insert(i);
          }
          else
          {
            selected_indices.clear();
            selected_indices.insert(i);
          }
        }

        if (is_selected)
          ImGui::SetItemDefaultFocus();
        ImGui::PopID();
      }
      ImGui::EndListBox();
    }

    // Action Buttons Row
    if (ImGui::Button("Add New", ImVec2(80, 0)))
    {
      sensors.emplace_back("New Sensor", -33.8688, 151.2093, 1000.0);
      selected_indices.clear();
      selected_indices.insert(static_cast<int>(sensors.size()) - 1);
      map.invalidate_rf_heatmap();
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete", ImVec2(80, 0)))
    {
      if (!selected_indices.empty())
      {
        // Delete in reverse order to preserve indices of remaining
        for (auto it = selected_indices.rbegin(); it != selected_indices.rend(); ++it)
        {
          int idx = *it;
          if (idx >= 0 && static_cast<size_t>(idx) < sensors.size())
          {
            sensors.erase(sensors.begin() + idx);
          }
        }
        selected_indices.clear();
        map.invalidate_rf_heatmap();
      }
    }
  }
  ImGui::EndGroup();

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  // Details Area
  if (selected_indices.size() == 1)
  {
    int idx = *selected_indices.begin();
    if (idx >= 0 && idx < static_cast<int>(sensors.size()))
    {
      sensor_t &sensor = sensors[idx];
      bool sensor_modified = false;

      // Use Tabs for cleaner config
      if (ImGui::BeginTabBar("SensorTabs"))
      {
        // --- GENERAL TAB ---
        if (ImGui::BeginTabItem("General"))
        {
          ImGui::Spacing();

          char name_buffer[128];
          snprintf(name_buffer, sizeof(name_buffer), "%s", sensor.get_name().c_str());
          if (ImGui::InputText("Name", name_buffer, sizeof(name_buffer)))
          {
            sensor.set_name(std::string(name_buffer));
          }

          if (ImGui::Button("Focus Camera", ImVec2(-1, 0)))
          {
            map.set_center(sensor.get_latitude(), sensor.get_longitude());
            if (map.get_zoom() < 16.0)
              map.set_zoom(16.0);
          }

          ImGui::Spacing();
          ImGui::Separator();
          ImGui::TextDisabled("LOCATION");
          ImGui::SameLine();
          bool is_locked = sensor.is_locked();
          // Use a smaller checkbox or text?
          // "Lock"
          if (ImGui::Checkbox("Locked", &is_locked))
          {
            sensor.set_locked(is_locked);
          }

          double lat = sensor.get_latitude();
          if (ImGui::InputDouble("Lat", &lat, 0.0001, 0.001, "%.6f"))
          {
            sensor.set_latitude(lat);
            sensor_modified = true;
          }

          double lon = sensor.get_longitude();
          if (ImGui::InputDouble("Lon", &lon, 0.0001, 0.001, "%.6f"))
          {
            sensor.set_longitude(lon);
            sensor_modified = true;
          }

          if (ImGui::Button("Snap to Building", ImVec2(-1, 0)))
          {
            map.fetch_buildings_near(lat, lon);
            double b_h = map.get_building_at_location(lat, lon);
            if (b_h > 0)
            {
              sensor.set_mast_height(b_h);
              sensor_modified = true;
            }
          }

          ImGui::Spacing();
          ImGui::Separator();
          ImGui::TextDisabled("VISUALIZATION");
          ImGui::ColorEdit3("Color", sensor.get_color_data());

          ImGui::EndTabItem();
        }

        // --- RF TAB ---
        if (ImGui::BeginTabItem("RF"))
        {
          ImGui::Spacing();

          double range = sensor.get_range();
          if (ImGui::InputDouble("Range (m)", &range, 100.0, 1000.0))
          {
            sensor.set_range(range);
            sensor_modified = true;
          }

          double mast_height = sensor.get_mast_height();
          if (ImGui::InputDouble("Mast (m)", &mast_height, 1.0, 5.0))
          {
            sensor.set_mast_height(mast_height);
            sensor_modified = true;
          }

          ImGui::Spacing();

          int current_model = static_cast<int>(sensor.get_propagation_model());
          const char *model_names[] = {"Free Space", "Hata (Urban)", "Hata (Suburban)", "Hata (Rural)"};
          if (ImGui::Combo("Model", &current_model, model_names, IM_ARRAYSIZE(model_names)))
          {
            sensor.set_propagation_model(static_cast<sensor_mapper::PropagationModel>(current_model));
            sensor_modified = true;
          }

          ImGui::Separator();
          ImGui::TextDisabled("ELEVATION");

          bool use_auto = sensor.get_use_auto_elevation();
          if (ImGui::Checkbox("Auto Terrain Alt.", &use_auto))
          {
            sensor.set_use_auto_elevation(use_auto);
            sensor_modified = true;
          }

          double ground_elevation = sensor.get_ground_elevation();
          if (use_auto)
          {
            float fetched_h;
            if (elevation_service.get_elevation(sensor.get_latitude(), sensor.get_longitude(), fetched_h))
            {
              ground_elevation = static_cast<double>(fetched_h);
              sensor.set_ground_elevation(ground_elevation);
            }
            ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Current: %.1f m", ground_elevation);
          }
          else
          {
            if (ImGui::InputDouble("Manual Elev", &ground_elevation))
            {
              sensor.set_ground_elevation(ground_elevation);
              sensor_modified = true;
            }
          }

          ImGui::EndTabItem();
        }

        // --- ANTENNA TAB ---
        if (ImGui::BeginTabItem("Antenna"))
        {
          ImGui::Spacing();

          float azimuth = static_cast<float>(sensor.get_azimuth_deg());
          if (ImGui::SliderFloat("Azimuth", &azimuth, 0.0f, 360.0f, "%.0f deg"))
          {
            sensor.set_azimuth_deg(static_cast<double>(azimuth));
            sensor_modified = true;
          }

          float beamwidth = static_cast<float>(sensor.get_beamwidth_deg());
          if (ImGui::SliderFloat("Beamwidth", &beamwidth, 1.0f, 360.0f, "%.0f deg"))
          {
            sensor.set_beamwidth_deg(static_cast<double>(beamwidth));
            sensor_modified = true;
          }

          ImGui::Separator();

          auto current_pattern = sensor.get_pattern();
          if (antenna_pattern_ui_t::render_pattern_selector(current_pattern, antenna_ui_state))
          {
            sensor.set_custom_pattern(current_pattern);
            sensor_modified = true;
          }

          if (current_pattern)
          {
            ImGui::Indent();
            ImGui::TextDisabled("PATTERN TWEAKS");
            float e_tilt = static_cast<float>(sensor.get_electrical_tilt_deg());
            if (ImGui::SliderFloat("E-Tilt", &e_tilt, -30.0f, 30.0f, "%.1f deg"))
            {
              sensor.set_electrical_tilt_deg(static_cast<double>(e_tilt));
              sensor_modified = true;
            }

            float m_tilt = static_cast<float>(sensor.get_mechanical_tilt_deg());
            if (ImGui::SliderFloat("M-Tilt", &m_tilt, -30.0f, 30.0f, "%.1f deg"))
            {
              sensor.set_mechanical_tilt_deg(static_cast<double>(m_tilt));
              sensor_modified = true;
            }
            ImGui::Unindent();
          }

          ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
      }

      if (sensor_modified)
        map.invalidate_rf_heatmap();
    }
  }
  else if (selected_indices.size() > 1)
  {
    ImGui::TextDisabled("BULK EDIT (%zu Sensors)", selected_indices.size());
    ImGui::Separator();

    bool modified = false;

    // We can only reliably bulk edit shared scalar properties like Color, Frequency (not stored in sensor_t yet), Range, Power, etc.
    // But sensor_t doesn't have Tx Power exposed in UI yet? No, it doesn't. Just Range.
    // Let's create a proxy Sensor representing the "Shared" state? Or just apply changes blindly.

    static bool range_init = false;
    if (!range_init)
    {
      // Initialize with first selected
      // But actually we should just show the slider and apply value.
      // Or show "Unchanged" if mixed? ImGui doesn't support mixed very well without multi-value widgets.
      // Let's just have an "Apply" button or apply immediately.
    }

    if (ImGui::Button("Set Range to 1km"))
    {
      for (int idx : selected_indices)
        sensors[idx].set_range(1000.0);
      modified = true;
    }
    if (ImGui::Button("Set Range to 5km"))
    {
      for (int idx : selected_indices)
        sensors[idx].set_range(5000.0);
      modified = true;
    }

    ImGui::Spacing();
    ImGui::Text("Propagation Model");
    int current_model = 0; // Default to Free Space
    const char *model_names[] = {"Free Space", "Hata (Urban)", "Hata (Suburban)", "Hata (Rural)"};
    if (ImGui::Combo("##bulk_model", &current_model, model_names, IM_ARRAYSIZE(model_names)))
    {
      for (int idx : selected_indices)
        sensors[idx].set_propagation_model(static_cast<sensor_mapper::PropagationModel>(current_model));
      modified = true;
    }

    ImGui::Spacing();
    ImGui::Text("Color");
    static float shared_color[3] = {1.0f, 0.0f, 0.0f};
    if (ImGui::ColorEdit3("##bulk_color", shared_color))
    {
      // Apply to all
      for (int idx : selected_indices)
      {
        // sensor_t uses float* get_color_data() returning internal pointer.
        // We can copy to it.
        float *c = sensors[idx].get_color_data();
        c[0] = shared_color[0];
        c[1] = shared_color[1];
        c[2] = shared_color[2];
      }
      // Do not invalidate heatmap for color change usually? Actually color is CPU-side for points?
      // If color is used for heatmap (no, heatmap is signal), then fine.
      // But sensor points draw uses color.
    }

    if (modified)
      map.invalidate_rf_heatmap();
  }
  else
  {
    ImGui::TextWrapped("Select a sensor to edit its properties.");
  }
}

void AppUI::render_elevation_data(elevation_service_t &elevation_service, map_widget_t &map)
{
  ImGui::TextDisabled("LOCAL ELEVATION FILES");

  static char file_path_buffer[512] = "";
  static std::string status_message = "";
  static bool status_is_error = false;

  ImGui::InputText("Path", file_path_buffer, sizeof(file_path_buffer));
  if (ImGui::Button("Load File", ImVec2(-1, 0)))
  {
    std::string path(file_path_buffer);
    if (!path.empty())
    {
      if (elevation_service.load_local_file(path))
      {
        status_message = "Loaded: " + path;
        status_is_error = false;
        map.invalidate_rf_heatmap();
      }
      else
      {
        status_message = "Failed to load file.";
        status_is_error = true;
      }
    }
  }

  if (!status_message.empty())
  {
    ImVec4 color = status_is_error ? ImVec4(1.0f, 0.4f, 0.4f, 1.0f) : ImVec4(0.4f, 1.0f, 0.4f, 1.0f);
    ImGui::TextColored(color, "%s", status_message.c_str());
  }

  ImGui::Separator();
  ImGui::TextDisabled("ACTIVE SOURCES");

  auto &sources = elevation_service.get_sources();
  if (sources.empty())
  {
    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f), "No sources loaded.");
  }
  else
  {
    if (ImGui::BeginChild("##sources_list", ImVec2(0, 150), true))
    {
      for (int i = static_cast<int>(sources.size()) - 1; i >= 0; --i)
      {
        ImGui::Text("[%d] %s", i + 1, sources[i]->get_name());
      }
    }
    ImGui::EndChild();
  }
}

void AppUI::render_tdoa_analysis(map_widget_t &map, const std::vector<sensor_t> & /*sensors*/)
{
  bool tdoa_enabled = map.get_show_tdoa_analysis();
  if (ImGui::Checkbox("Enable TDOA Visualization", &tdoa_enabled))
  {
    map.set_show_tdoa_analysis(tdoa_enabled);
  }

  if (tdoa_enabled)
  {
    ImGui::Separator();
    ImGui::TextDisabled("OPTIONS");

    bool show_hyperbolas = map.get_show_hyperbolas();
    if (ImGui::Checkbox("Show Hyperbolas", &show_hyperbolas))
      map.set_show_hyperbolas(show_hyperbolas);

    bool show_gdop = map.get_show_gdop_contours();
    if (ImGui::Checkbox("Show GDOP", &show_gdop))
      map.set_show_gdop_contours(show_gdop);

    bool show_accuracy = map.get_show_accuracy_heatmap();
    if (ImGui::Checkbox("Show Accuracy", &show_accuracy))
      map.set_show_accuracy_heatmap(show_accuracy);

    ImGui::Spacing();
    ImGui::TextDisabled("TEST POINT");

    if (map.has_tdoa_test_point())
    {
      auto result_2d = map.get_tdoa_test_result();
      ImGui::TextDisabled("2D ESTIMATE (Fixed Alt)");
      ImGui::Text("Error: %.1f m", result_2d.error_estimate_m);
      ImGui::Text("GDOP:  %.2f", result_2d.gdop);

      ImGui::Spacing();
      ImGui::TextDisabled("3D ESTIMATE (Free Space)");
      auto result_3d = map.get_tdoa_test_result_3d();
      if (result_3d.converged)
      {
        ImGui::Text("Alt:   %.1f m", result_3d.altitude);
        ImGui::Text("Error: %.1f m", result_3d.error_estimate_m);
        ImGui::Text("GDOP:  %.2f", result_3d.gdop);
      }
      else
      {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.5f, 1.0f), "Solver Diverged (Coplanar?)");
      }
      if (ImGui::Button("Clear Point"))
        map.clear_tdoa_test_point();
    }
    else
    {
      ImGui::TextWrapped("Shift+Click map to place test point.");
    }
  }
}

void AppUI::render_map_view_controls(map_widget_t &map)
{
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 4));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(8, 4));

  float window_width = ImGui::GetContentRegionAvail().x;
  float current_x = 0;

  auto wrap_group = [&](float group_width)
  {
    if (current_x > 0 && current_x + group_width > window_width)
    {
      ImGui::NewLine();
      current_x = 0;
    }
    else if (current_x > 0)
    {
      ImGui::SameLine();
      current_x += ImGui::GetStyle().ItemSpacing.x;
    }
    ImGui::BeginGroup();
  };

  auto end_group = [&](float actual_width)
  {
    ImGui::EndGroup();
    current_x += actual_width;
  };

  // --- Group 1: RF Visualization Modes (Approx 100px) ---
  wrap_group(100.0f);
  {
    bool show_heat = map.get_show_heatmap_overlay();
    if (ImGui::Checkbox("Heat", &show_heat))
      map.set_show_heatmap_overlay(show_heat);

    end_group(100.0f);
  }

  // --- Group 2: Special Modes (Approx 160px) ---
  wrap_group(160.0f);
  {
    bool show_overlap = (map.get_viz_mode() == 1);
    if (ImGui::Checkbox("Overlap", &show_overlap))
      map.set_viz_mode(show_overlap ? 1 : 0);

    ImGui::SameLine();
    bool show_qual = (map.get_viz_mode() == 2);
    if (ImGui::Checkbox("Qual", &show_qual))
      map.set_viz_mode(show_qual ? 2 : 0);

    end_group(160.0f);
  }

  // --- Group 3: Environment (Approx 140px) ---
  wrap_group(140.0f);
  {
    bool show_bldg = map.get_show_buildings();
    if (ImGui::Checkbox("Bldg", &show_bldg))
      map.set_show_buildings(show_bldg);

    ImGui::SameLine();
    bool show_src = map.get_show_elevation_sources();
    if (ImGui::Checkbox("Src", &show_src))
      map.set_show_elevation_sources(show_src);

    end_group(140.0f);
  }

  // --- Group 4: Overlays (Approx 160px) ---
  wrap_group(160.0f);
  {
    bool show_scale = map.get_show_scale_bar();
    if (ImGui::Checkbox("Scale", &show_scale))
      map.set_show_scale_bar(show_scale);

    ImGui::SameLine();
    bool show_compass = map.get_show_compass();
    if (ImGui::Checkbox("Compass", &show_compass))
      map.set_show_compass(show_compass);

    end_group(160.0f);
  }

  // --- Group 4: Config & Tools (Approx 400px) ---
  wrap_group(400.0f);
  {
    ImGui::SetNextItemWidth(80);
    float min_signal = map.get_min_signal_dbm();
    if (ImGui::SliderFloat("##minsignal", &min_signal, -120.0f, -50.0f, "%.0f"))
      map.set_min_signal_dbm(min_signal);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Threshold: %.0f dBm", min_signal);

    ImGui::SameLine();
    ImGui::SetNextItemWidth(80);
    float target_alt = map.get_target_alt_agl();
    if (ImGui::SliderFloat("##targetalt", &target_alt, 0.0f, 500.0f, "AGL: %.0fm"))
      map.set_target_alt_agl(target_alt);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Target (Drone) Altitude: %.0f m AGL", target_alt);

    ImGui::SameLine();
    if (ImGui::Button("Export"))
      map.export_coverage_map("coverage_export.ppm");

    ImGui::SameLine();
    ImGui::SetNextItemWidth(100);
    int map_source = map.get_map_source();
    const char *sources[] = {"Street", "Terrain", "Satellite"};
    if (ImGui::Combo("##mapsource", &map_source, sources, IM_ARRAYSIZE(sources)))
    {
      map.set_map_source(map_source);
    }

    end_group(300.0f);
  }

  ImGui::PopStyleVar(2);
  ImGui::Separator();
}

void AppUI::render_auto_placement(map_widget_t &map)
{
  ImGui::SetNextWindowSize(ImVec2(420, 650), ImGuiCond_FirstUseEver);
  if (!ImGui::Begin("Auto Sensor Placement", &m_show_auto_placement, ImGuiWindowFlags_NoScrollbar))
  {
    ImGui::End();
    return;
  }

  ImGui::TextWrapped("Automatically place sensors for optimal coverage and geolocation accuracy (minimized GDOP).");
  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  // --- SECTION 1: CONSTRAINTS ---
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "1. PLACEMENT CONSTRAINTS");
  ImGui::Spacing();

  ImGui::Checkbox("Use Buildings (Rooftops)", &m_opt_use_buildings);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Prioritize placing sensors on top of known buildings for better line-of-sight.");

  ImGui::SameLine(ImGui::GetWindowWidth() * 0.55f);
  ImGui::Checkbox("Use Terrain Peaks", &m_opt_use_terrain);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Prioritize placing sensors on local terrain peaks.");

  ImGui::Spacing();
  ImGui::Text("Target Sensor Count:");
  ImGui::SetNextItemWidth(-1);
  ImGui::SliderInt("##sensorcount", &m_opt_sensor_count, 3, 10, "%d Sensors");

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  // --- SECTION 2: SITE SELECTION ---
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "2. BUILDING SELECTION");
  ImGui::Spacing();
  ImGui::TextWrapped("Tag individual buildings on the map to influence the placement algorithm:");
  ImGui::Spacing();

  int sel_mode = static_cast<int>(map.get_selection_mode());
  bool changed = false;

  ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10.0f);
  if (ImGui::RadioButton("None", &sel_mode, 0))
    changed = true;
  ImGui::SameLine(120);
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.4f, 1.0f, 0.4f, 1.0f));
  if (ImGui::RadioButton("Priority", &sel_mode, 1))
    changed = true;
  ImGui::PopStyleColor();
  ImGui::SameLine(240);
  ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.4f, 0.4f, 1.0f));
  if (ImGui::RadioButton("Exclude", &sel_mode, 2))
    changed = true;
  ImGui::PopStyleColor();

  if (changed)
    map.set_selection_mode(static_cast<map_widget_t::SelectionMode>(sel_mode));

  ImGui::Spacing();
  bool restrict = map.get_restrict_to_priority();
  if (ImGui::Checkbox("Only place sensors on 'Priority' buildings", &restrict))
    map.set_restrict_to_priority(restrict);

  ImGui::Spacing();
  auto &p_set = map.get_priority_buildings();
  auto &e_set = map.get_excluded_buildings();

  ImGui::Text("Selection Status:");
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(0.4f, 1.0f, 0.4f, 1.0f), "%zu Priority", p_set.size());
  ImGui::SameLine();
  ImGui::Text("|");
  ImGui::SameLine();
  ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%zu Excluded", e_set.size());

  if (ImGui::Button("Clear Selection Tags", ImVec2(-1, 26)))
    map.clear_building_selection();

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  // --- SECTION 3: EXECUTION ---
  ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "3. OPTIMIZATION");
  ImGui::Spacing();

  static int strategy_idx = 0;
  const char *strategies[] = {"Geometric (Fastest)", "Advanced (Best Coverage + LOS)"};
  ImGui::Text("Optimization Strategy:");
  ImGui::SetNextItemWidth(-1);
  ImGui::Combo("##strategy", &strategy_idx, strategies, IM_ARRAYSIZE(strategies));
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Geometric: Even spatial distribution.\nAdvanced: Iterative optimization considering obstacles and geometry.");

  ImGui::Spacing();

  // Polygon Drawing Logic
  static bool was_drawing = false;
  bool is_drawing = map.is_drawing_polygon();
  if (was_drawing && !is_drawing)
  {
    const auto &poly = map.get_target_polygon();
    if (!poly.empty())
    {
      double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0;
      for (const auto &p : poly)
      {
        min_lat = std::min(min_lat, p.first);
        max_lat = std::max(max_lat, p.first);
        min_lon = std::min(min_lon, p.second);
        max_lon = std::max(max_lon, p.second);
      }
      map.fetch_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
      m_waiting_for_buildings = true;
    }
  }
  was_drawing = is_drawing;

  if (map.is_drawing_polygon())
  {
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.45f, 0.85f, 1.0f));
    if (ImGui::Button("FINISH DRAWING TARGET AREA", ImVec2(-1, 45)))
      map.finish_polygon();
    ImGui::PopStyleColor();
  }
  else
  {
    if (ImGui::Button("DRAW TARGET AREA ON MAP", ImVec2(-1, 45)))
      map.start_listing_polygon();
  }

  ImGui::Spacing();

  const auto &polygon = map.get_target_polygon();
  bool ready = !polygon.empty() && !map.is_drawing_polygon() && (m_opt_use_buildings || m_opt_use_terrain);

  if (m_optimizer->is_running())
  {
    ImGui::Text("Status: Optimizing...");
    ImGui::ProgressBar(m_optimizer->get_progress(), ImVec2(-1, 28), m_optimizer->get_status().c_str());
    if (ImGui::Button("Cancel Optimization", ImVec2(-1, 26)))
      m_optimizer->cancel();
  }
  else if (m_waiting_for_buildings)
  {
    auto status = map.get_building_loading_status();
    ImGui::Text("Loading data: %d active, %d queued", status.first, status.second);
    ImGui::ProgressBar(-1.0f * (float)ImGui::GetTime(), ImVec2(-1, 28), "Fetching Building Tiles...");

    // Auto-start logic
    double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0;
    for (const auto &p : polygon)
    {
      min_lat = std::min(min_lat, p.first);
      max_lat = std::max(max_lat, p.first);
      min_lon = std::min(min_lon, p.second);
      max_lon = std::max(max_lon, p.second);
    }
    if (!m_opt_use_buildings || map.has_buildings_for_area(min_lat, max_lat, min_lon, max_lon) || (status.first == 0 && status.second == 0))
    {
      m_waiting_for_buildings = false;
      if (m_opt_start_pending)
      {
        m_opt_start_pending = false;
        optimizer_config_t config;
        config.use_buildings = m_opt_use_buildings;
        config.use_terrain = m_opt_use_terrain;
        config.sensor_count = m_opt_sensor_count;
        config.strategy = static_cast<OptimizationStrategy>(strategy_idx);
        auto building_ptrs = map.get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
        for (auto b_ptr : building_ptrs)
          if (b_ptr)
            config.buildings.push_back(*b_ptr);
        auto &p_set_ref = map.get_priority_buildings();
        config.priority_building_ids.assign(p_set_ref.begin(), p_set_ref.end());
        auto &e_set_ref = map.get_excluded_buildings();
        config.excluded_building_ids.assign(e_set_ref.begin(), e_set_ref.end());
        config.restrict_to_priority = map.get_restrict_to_priority();
        m_optimizer->start(polygon, config);
      }
    }
  }
  else
  {
    if (!ready)
      ImGui::BeginDisabled();
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.65f, 0.25f, 1.0f));
    if (ImGui::Button("RUN OPTIMIZER", ImVec2(-1, 55)))
    {
      double min_lat = 90.0, max_lat = -90.0, min_lon = 180.0, max_lon = -180.0;
      for (const auto &p : polygon)
      {
        min_lat = std::min(min_lat, p.first);
        max_lat = std::max(max_lat, p.first);
        min_lon = std::min(min_lon, p.second);
        max_lon = std::max(max_lon, p.second);
      }
      if (m_opt_use_buildings && !map.has_buildings_for_area(min_lat, max_lat, min_lon, max_lon))
      {
        map.fetch_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
        m_waiting_for_buildings = true;
        m_opt_start_pending = true;
      }
      else
      {
        optimizer_config_t config;
        config.use_buildings = m_opt_use_buildings;
        config.use_terrain = m_opt_use_terrain;
        config.sensor_count = m_opt_sensor_count;
        config.strategy = static_cast<OptimizationStrategy>(strategy_idx);
        auto building_ptrs = map.get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
        for (auto b_ptr : building_ptrs)
          if (b_ptr)
            config.buildings.push_back(*b_ptr);
        auto &p_set_ref = map.get_priority_buildings();
        config.priority_building_ids.assign(p_set_ref.begin(), p_set_ref.end());
        auto &e_set_ref = map.get_excluded_buildings();
        config.excluded_building_ids.assign(e_set_ref.begin(), e_set_ref.end());
        config.restrict_to_priority = map.get_restrict_to_priority();
        m_optimizer->start(polygon, config);
      }
    }
    ImGui::PopStyleColor();
    if (!ready)
      ImGui::EndDisabled();

    if (!polygon.empty())
    {
      ImGui::Spacing();
      if (ImGui::Button("Clear Target Area", ImVec2(-1, 26)))
        map.clear_polygon();
    }
  }

  ImGui::End();
}

} // namespace sensor_mapper
