#include <glad/glad.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <vector>

#include "core/persistence.hpp"
#include "core/sensor.hpp"
#include "ui/map_widget.hpp"
#include "ui/antenna_pattern_ui.hpp"

// Forward declaration
namespace sensor_mapper
{
void render_ui(map_widget_t &map, std::vector<sensor_t> &sensors, int &selected_index, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state, std::function<void()> on_exit);
}

constexpr const char *SENSORS_FILE = "sensors.json";

// Main code
int main(int, char **)
{
  // Setup window
  glfwSetErrorCallback([](int error, const char *description) { std::cerr << "Glfw Error " << error << ": " << description << std::endl; });

  if (!glfwInit())
    return 1;

  // GL 3.0 + GLSL 130
  const char *glsl_version = "#version 130";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  // Create window with graphics context
  GLFWwindow *window = glfwCreateWindow(1280, 720, "Sensor Mapper", NULL, NULL);
  if (window == NULL)
    return 1;
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  // Initialize GLAD
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
  {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return 1;
  }

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;     // Enable Docking

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Application State
  sensor_mapper::map_widget_t map_widget;
  sensor_mapper::elevation_service_t elevation_service;
  std::vector<sensor_mapper::sensor_t> sensors;

  // Try loading
  sensor_mapper::persistence::load_sensors(SENSORS_FILE, sensors);
  int selected_sensor_index = 0;

  // Antenna Pattern UI State
  sensor_mapper::antenna_pattern_ui_state_t antenna_ui_state;

  // Main loop
  while (!glfwWindowShouldClose(window))
  {
    // Poll and handle events
    glfwPollEvents();

    // Update services
    elevation_service.update();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    struct ExitContext
    {
      GLFWwindow *win;
    } exit_ctx = {window};

    // Core UI Logic
    sensor_mapper::render_ui(map_widget, sensors, selected_sensor_index, elevation_service, antenna_ui_state, [&exit_ctx]() { glfwSetWindowShouldClose(exit_ctx.win, 1); });

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.45f, 0.55f, 0.60f, 1.00f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}

namespace sensor_mapper
{
void render_ui(map_widget_t &map, std::vector<sensor_t> &sensors, int &selected_index, elevation_service_t &elevation_service, antenna_pattern_ui_state_t &antenna_ui_state, std::function<void()> on_exit)
{
  // Dockspace
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  // Window Visibility State
  static bool show_sensor_config = true;
  static bool show_map_view = true;
  static bool show_elevation_data = true;
  static bool show_tdoa_analysis = true;

  // Main Menu Bar
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
      ImGui::MenuItem("Sensor Configuration", nullptr, &show_sensor_config);
      ImGui::MenuItem("Map View", nullptr, &show_map_view);
      ImGui::MenuItem("Elevation Data", nullptr, &show_elevation_data);
      ImGui::MenuItem("TDOA Analysis", nullptr, &show_tdoa_analysis);
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }

  if (show_sensor_config)
  {
    if (ImGui::Begin("Sensor Configuration", &show_sensor_config))
    {
      // Management Buttons
      if (ImGui::Button("Add Sensor"))
      {
        sensors.emplace_back("New Sensor", -33.8688, 151.2093, 1000.0);
        selected_index = static_cast<int>(sensors.size()) - 1;
        map.invalidate_rf_heatmap();
      }
      ImGui::SameLine();
      bool delete_pressed = ImGui::Button("Delete Sensor");

      // List Box
      if (ImGui::BeginListBox("##sensorlist", ImVec2(-FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing())))
      {
        for (int i = 0; i < static_cast<int>(sensors.size()); i++)
        {
          ImGui::PushID(i);
          const bool is_selected = (selected_index == i);
          if (ImGui::Selectable(sensors[i].get_name().c_str(), is_selected))
            selected_index = i;

          // Set the initial focus when opening the combo (scrolling + keyboard
          // navigation focus)
          if (is_selected)
            ImGui::SetItemDefaultFocus();
          ImGui::PopID();
        }
        ImGui::EndListBox();
      }

      if (delete_pressed && selected_index >= 0 && selected_index < static_cast<int>(sensors.size()))
      {
        sensors.erase(sensors.begin() + selected_index);
        if (selected_index >= static_cast<int>(sensors.size()))
          selected_index = static_cast<int>(sensors.size()) - 1;
        map.invalidate_rf_heatmap();
      }

      ImGui::Separator();

      // Selected Sensor Properties
      if (selected_index >= 0 && selected_index < static_cast<int>(sensors.size()))
      {
        sensor_t &sensor = sensors[selected_index];
        bool sensor_modified = false;
        ImGui::Spacing();
        ImGui::Separator();
        ImGui::TextDisabled("PROPERTIES");

        char name_buffer[128];
        snprintf(name_buffer, sizeof(name_buffer), "%s", sensor.get_name().c_str());
        if (ImGui::InputText("Name", name_buffer, sizeof(name_buffer)))
        {
          sensor.set_name(std::string(name_buffer));
        }

        if (ImGui::Button("Focus Camera"))
        {
          map.set_center(sensor.get_latitude(), sensor.get_longitude());
          if (map.get_zoom() < 15.0)
            map.set_zoom(16.0);
        }

        if (ImGui::CollapsingHeader("Location", ImGuiTreeNodeFlags_DefaultOpen))
        {
          double lat = sensor.get_latitude();
          if (ImGui::InputDouble("Latitude", &lat, 0.0001, 0.001, "%.6f"))
          {
            sensor.set_latitude(lat);
            sensor_modified = true;
          }

          double lon = sensor.get_longitude();
          if (ImGui::InputDouble("Longitude", &lon, 0.0001, 0.001, "%.6f"))
          {
            sensor.set_longitude(lon);
            sensor_modified = true;
          }

          if (ImGui::Button("Snap to Building Height"))
          {
            // Manual trigger
            map.fetch_buildings_near(lat, lon);
            double b_h = map.get_building_at_location(lat, lon);
            if (b_h > 0)
            {
              sensor.set_mast_height(b_h);
              sensor_modified = true;
            }
          }
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Sets mast height to building roof if available.");
        }

        if (ImGui::CollapsingHeader("RF Parameters", ImGuiTreeNodeFlags_DefaultOpen))
        {
          double range = sensor.get_range();
          if (ImGui::InputDouble("Range (m)", &range, 100.0, 1000.0))
          {
            sensor.set_range(range);
            sensor_modified = true;
          }

          double mast_height = sensor.get_mast_height();
          if (ImGui::InputDouble("Mast Height (m)", &mast_height, 1.0, 5.0))
          {
            sensor.set_mast_height(mast_height);
            sensor_modified = true;
          }

          // Directional Parameters
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

          // Antenna Pattern Selection
          ImGui::Separator();
          auto current_pattern = sensor.get_pattern();
          if (antenna_pattern_ui_t::render_pattern_selector(current_pattern, antenna_ui_state))
          {
            sensor.set_custom_pattern(current_pattern);
            sensor_modified = true;
          }

          // Display current pattern info
          if (current_pattern)
          {
            ImGui::Indent();
            ImGui::TextColored(ImVec4(0.7f, 1.0f, 0.7f, 1.0f), "Pattern: %s", current_pattern->name.c_str());
            ImGui::Text("Max Gain: %.1f dBi", current_pattern->max_gain_dbi);
            ImGui::Text("H. Beamwidth: %.1f°", current_pattern->horizontal_beamwidth_deg);
            if (current_pattern->is_3d())
            {
              ImGui::Text("V. Beamwidth: %.1f°", current_pattern->vertical_beamwidth_deg);
            }

            // Tilt controls if pattern supports it
            float e_tilt = static_cast<float>(sensor.get_electrical_tilt_deg());
            if (ImGui::SliderFloat("E-Tilt", &e_tilt, -30.0f, 30.0f, "%.1f°"))
            {
              sensor.set_electrical_tilt_deg(static_cast<double>(e_tilt));
              sensor_modified = true;
            }

            float m_tilt = static_cast<float>(sensor.get_mechanical_tilt_deg());
            if (ImGui::SliderFloat("M-Tilt", &m_tilt, -30.0f, 30.0f, "%.1f°"))
            {
              sensor.set_mechanical_tilt_deg(static_cast<double>(m_tilt));
              sensor_modified = true;
            }

            ImGui::Unindent();
          }
          ImGui::Separator();

          // Propagation Model
          int current_model = static_cast<int>(sensor.get_propagation_model());
          const char *model_names[] = {"Free Space", "Hata (Urban)", "Hata (Suburban)", "Hata (Rural)"};
          if (ImGui::Combo("Propagation Model", &current_model, model_names, IM_ARRAYSIZE(model_names)))
          {
            sensor.set_propagation_model(static_cast<sensor_mapper::PropagationModel>(current_model));
            sensor_modified = true;
          }

          // Display effective height
          float ground_h = 0.0f;
          float elev = 0.0f;
          if (elevation_service.get_elevation(sensor.get_latitude(), sensor.get_longitude(), elev))
          {
            ground_h = elev;
          }
          ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Ground Elevation: %.1f m", ground_h);
          ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f), "Total Height (AMSL): %.1f m", ground_h + mast_height);
        }

        bool use_auto = sensor.get_use_auto_elevation();
        double ground_elevation = sensor.get_ground_elevation();

        if (ImGui::CollapsingHeader("Advanced"))
        {
          if (ImGui::Checkbox("Auto Terrain Altitude", &use_auto))
          {
            sensor.set_use_auto_elevation(use_auto);
            sensor_modified = true;
          }

          if (!use_auto)
          {
            if (ImGui::InputDouble("Manual Ground Elev", &ground_elevation))
            {
              sensor.set_ground_elevation(ground_elevation);
              sensor_modified = true;
            }
          }
        }

        // Invalidate RF heatmap if any sensor property was modified
        if (sensor_modified)
        {
          map.invalidate_rf_heatmap();
        }

        // If Auto is enabled, try to display the actual fetched elevation
        if (use_auto)
        {
          float fetched_h;
          if (elevation_service.get_elevation(sensor.get_latitude(), sensor.get_longitude(), fetched_h))
          {
            ground_elevation = static_cast<double>(fetched_h);
            // Auto-update the sensor's ground elevation too so RF model sees it
            sensor.set_ground_elevation(ground_elevation);
          }
        }

        ImGui::BeginDisabled(true);
        ImGui::InputDouble("Ground Altitude (m) (AMSL)", &ground_elevation, 0.0, 0.0, "%.1f");
        ImGui::EndDisabled();

        ImGui::ColorEdit3("Color", sensor.get_color_data());
      }
      else
      {
        ImGui::Text("No sensor selected.");
      }
    }
    ImGui::End();
  }

  if (show_map_view)
  {
    if (ImGui::Begin("Map View", &show_map_view))
    {
      // RF Gradient Toggle
      // RF Gradient Toggle
      bool show_rf = map.get_show_rf_gradient();
      if (ImGui::Checkbox("Show RF Signal Gradient", &show_rf))
      {
        map.set_show_rf_gradient(show_rf);
      }

      bool show_composite = map.get_show_composite();
      if (ImGui::Checkbox("Show Composite Coverage", &show_composite))
      {
        map.set_show_composite(show_composite);
      }

      bool show_heatmap = map.get_show_heatmap_overlay();
      if (ImGui::Checkbox("Show Coverage Heatmap", &show_heatmap))
      {
        map.set_show_heatmap_overlay(show_heatmap);
      }

      // Min Signal Threshold Slider
      float min_signal = map.get_min_signal_dbm();
      ImGui::SetNextItemWidth(200.0f);
      if (ImGui::SliderFloat("Min Signal (dBm)", &min_signal, -120.0f, -50.0f, "%.0f dBm"))
      {
        map.set_min_signal_dbm(min_signal);
      }
      if (ImGui::IsItemHovered())
      {
        ImGui::SetTooltip("Minimum signal strength to display.\nLower values = more coverage (slower)\nHigher values = less coverage (faster)");
      }

      bool show_buildings = map.get_show_buildings();
      if (ImGui::Checkbox("Show 3D Buildings", &show_buildings))
      {
        map.set_show_buildings(show_buildings);
      }

      map.draw(sensors, selected_index, elevation_service,
               [&](double lat, double lon)
               {
                 // Callback for adding sensor from map

                 // 1. Trigger building fetch for this area (for next time /
                 // background)
                 map.fetch_buildings_near(lat, lon);

                 // 2. Check if we already have building data
                 double building_height = map.get_building_at_location(lat, lon);

                 // 3. Create sensor
                 // Default height 5m, or use building height if found
                 double mast_h = building_height > 0 ? building_height : 5.0;

                 sensors.emplace_back("New Sensor", lat, lon, 5000.0);
                 auto &s = sensors.back();
                 s.set_mast_height(mast_h); // Set height (AGL)
                 map.invalidate_rf_heatmap();

                 // If on building, user probably wants it ON the roof, not
                 // floating 5m above it? Wait, mast_height is AGL (Above Ground
                 // Level). If we are on a building, ground elevation (terrain) is
                 // used + mast height. So if building is 20m, setting
                 // mast_height=20m puts it on the roof. (Assuming "ground" means
                 // "terrain"). Yes, implementation usually adds mast_height to
                 // terrain elevation.

                 selected_index = static_cast<int>(sensors.size()) - 1;
               });
    }
    ImGui::End();
  }

  // Elevation Data Panel
  if (show_elevation_data)
  {
    if (ImGui::Begin("Elevation Data", &show_elevation_data))
    {
      ImGui::TextDisabled("LOCAL ELEVATION FILES");
      ImGui::Separator();

      // File path input
      static char file_path_buffer[512] = "";
      static std::string status_message = "";
      static bool status_is_error = false;

      ImGui::Text("Load GeoTIFF (.tif, .tiff) or LIDAR (.las, .laz):");
      ImGui::SetNextItemWidth(-100.0f);
      ImGui::InputText("##filepath", file_path_buffer, sizeof(file_path_buffer));
      ImGui::SameLine();

      if (ImGui::Button("Load File"))
      {
        std::string path(file_path_buffer);
        if (!path.empty())
        {
          if (elevation_service.load_local_file(path))
          {
            status_message = "✓ Successfully loaded: " + path;
            status_is_error = false;
            map.invalidate_rf_heatmap(); // Invalidate heatmap to use new elevation data
          }
          else
          {
            status_message = "✗ Failed to load file (unsupported format or file not found)";
            status_is_error = true;
          }
        }
        else
        {
          status_message = "✗ Please enter a file path";
          status_is_error = true;
        }
      }

      // Display status message
      if (!status_message.empty())
      {
        ImVec4 color = status_is_error ? ImVec4(1.0f, 0.3f, 0.3f, 1.0f) : ImVec4(0.3f, 1.0f, 0.3f, 1.0f);
        ImGui::TextColored(color, "%s", status_message.c_str());
      }

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::TextDisabled("ACTIVE SOURCES");

      // List active elevation sources
      auto &sources = elevation_service.get_sources();
      if (sources.empty())
      {
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "No elevation sources loaded");
      }
      else
      {
        ImGui::Text("Total sources: %zu", sources.size());
        ImGui::Spacing();

        if (ImGui::BeginChild("##sources_list", ImVec2(0, 100), true))
        {
          for (size_t i = 0; i < sources.size(); ++i)
          {
            if (sources[i])
            {
              // Show index (in reverse order since LIFO priority)
              size_t priority = sources.size() - i;
              ImGui::Text("[%zu] %s", priority, sources[i]->get_name());
            }
          }
        }
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::TextColored(ImVec4(0.7f, 0.9f, 1.0f, 1.0f), "ℹ Sources are queried in reverse order (last loaded = highest priority)");
      }

      ImGui::Spacing();
      ImGui::Separator();
      ImGui::TextDisabled("CAPABILITIES");
      ImGui::TextWrapped("• Supports GeoTIFF (.tif, .tiff)");
      ImGui::TextWrapped("• Supports LIDAR (.las, .laz) with automatic coordinate reprojection (UTM/MGA -> Lat/Lon) via PROJ");
    }
    ImGui::End();
  }

  // TDOA Analysis Panel
  if (show_tdoa_analysis)
  {
    if (ImGui::Begin("TDOA Analysis", &show_tdoa_analysis))
    {
      bool tdoa_enabled = map.get_show_tdoa_analysis();
      if (ImGui::Checkbox("Enable TDOA Visualization", &tdoa_enabled))
      {
        map.set_show_tdoa_analysis(tdoa_enabled);
      }

      if (tdoa_enabled)
      {
        ImGui::Separator();
        ImGui::TextDisabled("VISUALIZATION OPTIONS");

        bool show_hyperbolas = map.get_show_hyperbolas();
        if (ImGui::Checkbox("Show Hyperbolas", &show_hyperbolas))
        {
          map.set_show_hyperbolas(show_hyperbolas);
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Display time-difference curves between sensor pairs");

        bool show_gdop = map.get_show_gdop_contours();
        if (ImGui::Checkbox("Show GDOP Contours", &show_gdop))
        {
          map.set_show_gdop_contours(show_gdop);
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Show geometric dilution of precision (lower is better)");

        bool show_accuracy = map.get_show_accuracy_heatmap();
        if (ImGui::Checkbox("Show Accuracy Heatmap", &show_accuracy))
        {
          map.set_show_accuracy_heatmap(show_accuracy);
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Visualize expected positioning accuracy across coverage area");

        ImGui::Separator();
        ImGui::TextDisabled("TEST POINT");

        if (map.has_tdoa_test_point())
        {
          auto result = map.get_tdoa_test_result();

          ImGui::Text("Estimated Position:");
          ImGui::Text("  Lat: %.6f°", result.latitude);
          ImGui::Text("  Lon: %.6f°", result.longitude);
          ImGui::Text("Error Estimate: %.1f m", result.error_estimate_m);
          ImGui::Text("GDOP: %.2f", result.gdop);
          ImGui::Text("Converged: %s", result.converged ? "Yes" : "No");

          if (ImGui::Button("Clear Test Point"))
          {
            map.clear_tdoa_test_point();
          }
        }
        else
        {
          ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "Shift+Click on map to place test point");
        }

        ImGui::Separator();
        ImGui::TextDisabled("CONFIGURATION");

        // Timing jitter slider
        float timing_jitter = map.get_timing_jitter_ns();
        if (ImGui::SliderFloat("Timing Jitter (ns)", &timing_jitter, 0.0f, 100.0f, "%.1f ns"))
        {
          map.set_timing_jitter_ns(timing_jitter);
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Clock synchronization error\nGPS-disciplined: ~10ns\nTypical: ~50ns");

        // Display sensor count
        ImGui::Text("Active Sensors: %zu", sensors.size());

        // Warn if insufficient sensors
        if (sensors.size() < 3)
        {
          ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "⚠ Need at least 3 sensors for TDOA");
        }
        else
        {
          ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "✓ %zu sensors ready for TDOA", sensors.size());
        }
      }
    }
    ImGui::End();
  }
}
} // namespace sensor_mapper
