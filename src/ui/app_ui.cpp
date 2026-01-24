
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

void AppUI::render(map_widget_t &map, std::vector<sensor_t> &sensors, int &selected_sensor_index, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state, std::function<void()> on_exit)
{
  // Dockspace
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  render_main_menu(sensors, selected_sensor_index, map, on_exit);

  if (m_show_sensor_config)
  {
    // Use a fixed width for the sensor config panel if undocked/first run, but allow resizing
    ImGui::SetNextWindowSize(ImVec2(370, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Sensor Configuration", &m_show_sensor_config))
    {
      render_sensor_config(sensors, selected_sensor_index, map, elevation_service, antenna_ui_state);
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
      render_map_view_controls(map);

      // Draw map
      map.draw(sensors, selected_sensor_index, elevation_service,
               [&](double lat, double lon)
               {
                 // Callback to add sensor
                 map.fetch_buildings_near(lat, lon);
                 double b_h = map.get_building_at_location(lat, lon);
                 double mast_h = b_h > 0 ? b_h : 5.0;

                 sensors.emplace_back("New Sensor", lat, lon, 5000.0);
                 auto &s = sensors.back();
                 s.set_mast_height(mast_h); // Set height (AGL)
                 map.invalidate_rf_heatmap();
                 selected_sensor_index = static_cast<int>(sensors.size()) - 1;
               });
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

  // Status Bar
  if (ImGui::BeginViewportSideBar("##MainStatusBar", ImGui::GetMainViewport(), ImGuiDir_Down, ImGui::GetFrameHeight(), ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_MenuBar))
  {
    if (ImGui::BeginMenuBar())
    {
      ImGui::Text("Cursor: %.5f, %.5f", map.get_mouse_lat(), map.get_mouse_lon());
      ImGui::Separator();

      // Re-calculate cursor altitude for status bar manually or use cached
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

void AppUI::render_main_menu(std::vector<sensor_t> &sensors, int &selected_index, map_widget_t &map, std::function<void()> on_exit)
{
  if (ImGui::BeginMainMenuBar())
  {
    if (ImGui::BeginMenu("File"))
    {
      if (ImGui::MenuItem("Save Sensors", "Ctrl+S"))
      {
        persistence::save_sensors(SENSORS_FILE, sensors);
      }
      if (ImGui::MenuItem("Load Sensors", "Ctrl+L"))
      {
        persistence::load_sensors(SENSORS_FILE, sensors);
        if (selected_index >= static_cast<int>(sensors.size()))
          selected_index = -1;
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
        selected_index = static_cast<int>(sensors.size()) - 1;
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
    ImGui::EndMainMenuBar();
  }
}

void AppUI::render_sensor_config(std::vector<sensor_t> &sensors, int &selected_index, map_widget_t &map, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state)
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
        const bool is_selected = (selected_index == i);
        if (ImGui::Selectable(sensors[i].get_name().c_str(), is_selected))
          selected_index = i;

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
      selected_index = static_cast<int>(sensors.size()) - 1;
      map.invalidate_rf_heatmap();
    }
    ImGui::SameLine();
    if (ImGui::Button("Delete", ImVec2(80, 0)))
    {
      if (selected_index >= 0 && selected_index < static_cast<int>(sensors.size()))
      {
        sensors.erase(sensors.begin() + selected_index);
        if (selected_index >= static_cast<int>(sensors.size()))
          selected_index = static_cast<int>(sensors.size()) - 1;
        map.invalidate_rf_heatmap();
      }
    }
  }
  ImGui::EndGroup();

  ImGui::Spacing();
  ImGui::Separator();
  ImGui::Spacing();

  // Details Area
  if (selected_index >= 0 && selected_index < static_cast<int>(sensors.size()))
  {
    sensor_t &sensor = sensors[selected_index];
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

void AppUI::render_tdoa_analysis(map_widget_t &map, const std::vector<sensor_t> &sensors)
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
      auto result = map.get_tdoa_test_result();
      ImGui::Text("Error: %.1f m", result.error_estimate_m);
      ImGui::Text("GDOP: %.2f", result.gdop);
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
  // Toolbar style
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(5, 5));

  // -- Toggles --
  bool show_rf = map.get_show_rf_gradient();
  if (ImGui::Checkbox("RF", &show_rf))
    map.set_show_rf_gradient(show_rf);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Show RF signal strength gradient");

  ImGui::SameLine();
  bool show_comp = map.get_show_composite();
  if (ImGui::Checkbox("Comp", &show_comp))
    map.set_show_composite(show_comp);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Show composite coverage");

  ImGui::SameLine();
  bool show_heat = map.get_show_heatmap_overlay();
  if (ImGui::Checkbox("Heat", &show_heat))
    map.set_show_heatmap_overlay(show_heat);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Show coverage heatmap");

  ImGui::SameLine();
  bool show_bldg = map.get_show_buildings();
  if (ImGui::Checkbox("Bldg", &show_bldg))
    map.set_show_buildings(show_bldg);

  ImGui::SameLine();
  bool show_elev = map.get_show_elevation_sources();
  if (ImGui::Checkbox("Src", &show_elev))
    map.set_show_elevation_sources(show_elev);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Show elevation source bounds");

  ImGui::SameLine();
  bool show_rast = map.get_show_raster_visual();
  if (ImGui::Checkbox("Rast", &show_rast))
    map.set_show_raster_visual(show_rast);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Show elevation raster visual");

  // -- Signal Slider --
  ImGui::SameLine();
  ImGui::SetNextItemWidth(100);
  float min_signal = map.get_min_signal_dbm();
  if (ImGui::SliderFloat("##minsignal", &min_signal, -120.0f, -50.0f, "%.0f dBm"))
  {
    map.set_min_signal_dbm(min_signal);
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Min Signal Threshold");

  ImGui::PopStyleVar();
  ImGui::Separator();
}

} // namespace sensor_mapper
