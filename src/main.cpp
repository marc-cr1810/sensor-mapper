#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <vector>

#include "core/persistence.hpp"
#include "core/sensor.hpp"
#include "ui/map_widget.hpp"

// Forward declaration
namespace sensor_mapper {
void render_ui(map_widget_t &map, std::vector<sensor_t> &sensors,
               int &selected_index, elevation_service_t &elevation_service);
}

constexpr const char *SENSORS_FILE = "sensors.json";

// Main code
int main(int, char **) {
  // Setup window
  glfwSetErrorCallback([](int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
  });

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

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |=
      ImGuiConfigFlags_NavEnableKeyboard;           // Enable Keyboard Controls
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable; // Enable Docking

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

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // Poll and handle events
    glfwPollEvents();

    // Update services
    elevation_service.update();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Core UI Logic
    sensor_mapper::render_ui(map_widget, sensors, selected_sensor_index,
                             elevation_service);

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

namespace sensor_mapper {
void render_ui(map_widget_t &map, std::vector<sensor_t> &sensors,
               int &selected_index, elevation_service_t &elevation_service) {
  // Dockspace
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  if (ImGui::Begin("Sensor Configuration")) {

    // Persistence
    if (ImGui::Button("Save Sensors")) {
      persistence::save_sensors(SENSORS_FILE, sensors);
    }
    ImGui::SameLine();
    if (ImGui::Button("Load Sensors")) {
      persistence::load_sensors(SENSORS_FILE, sensors);
      if (selected_index >= static_cast<int>(sensors.size()))
        selected_index = -1;
    }
    ImGui::Separator();

    // Management Buttons
    if (ImGui::Button("Add Sensor")) {
      sensors.emplace_back("New Sensor", -33.8688, 151.2093, 1000.0);
      selected_index = static_cast<int>(sensors.size()) - 1;
    }
    ImGui::SameLine();
    bool delete_pressed = ImGui::Button("Delete Sensor");

    // List Box
    if (ImGui::BeginListBox(
            "##sensorlist",
            ImVec2(-FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing()))) {
      for (int i = 0; i < static_cast<int>(sensors.size()); i++) {
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

    if (delete_pressed && selected_index >= 0 &&
        selected_index < sensors.size()) {
      sensors.erase(sensors.begin() + selected_index);
      if (selected_index >= sensors.size())
        selected_index = static_cast<int>(sensors.size()) - 1;
    }

    ImGui::Separator();

    // Selected Sensor Properties
    if (selected_index >= 0 && selected_index < sensors.size()) {
      sensor_t &sensor = sensors[selected_index];
      ImGui::Spacing();
      ImGui::Separator();
      ImGui::TextDisabled("PROPERTIES");

      char name_buffer[128];
      snprintf(name_buffer, sizeof(name_buffer), "%s",
               sensor.get_name().c_str());
      if (ImGui::InputText("Name", name_buffer, sizeof(name_buffer))) {
        sensor.set_name(std::string(name_buffer));
      }

      if (ImGui::Button("Focus Camera")) {
        map.set_center(sensor.get_latitude(), sensor.get_longitude());
        if (map.get_zoom() < 15.0)
          map.set_zoom(16.0);
      }

      if (ImGui::CollapsingHeader("Location", ImGuiTreeNodeFlags_DefaultOpen)) {
        double lat = sensor.get_latitude();
        if (ImGui::InputDouble("Latitude", &lat, 0.0001, 0.001, "%.6f")) {
          sensor.set_latitude(lat);
        }

        double lon = sensor.get_longitude();
        if (ImGui::InputDouble("Longitude", &lon, 0.0001, 0.001, "%.6f")) {
          sensor.set_longitude(lon);
        }

        if (ImGui::Button("Snap to Building Height")) {
          // Manual trigger
          map.fetch_buildings_near(lat, lon);
          double b_h = map.get_building_at_location(lat, lon);
          if (b_h > 0) {
            sensor.set_mast_height(b_h);
          }
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("Sets mast height to building roof if available.");
      }

      if (ImGui::CollapsingHeader("RF Parameters",
                                  ImGuiTreeNodeFlags_DefaultOpen)) {
        double range = sensor.get_range();
        if (ImGui::InputDouble("Range (m)", &range, 100.0, 1000.0)) {
          sensor.set_range(range);
        }

        double mast_height = sensor.get_mast_height();
        if (ImGui::InputDouble("Mast Height (m)", &mast_height, 1.0, 5.0)) {
          sensor.set_mast_height(mast_height);
        }

        // Display effective height
        float ground_h = 0.0f;
        float elev = 0.0f;
        if (elevation_service.get_elevation(sensor.get_latitude(),
                                            sensor.get_longitude(), elev)) {
          ground_h = elev;
        }
        ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f),
                           "Ground Elevation: %.1f m", ground_h);
        ImGui::TextColored(ImVec4(0.5f, 1.0f, 0.5f, 1.0f),
                           "Total Height (AMSL): %.1f m",
                           ground_h + mast_height);
      }

      bool use_auto = sensor.get_use_auto_elevation();
      double ground_elevation = sensor.get_ground_elevation();

      if (ImGui::CollapsingHeader("Advanced")) {
        if (ImGui::Checkbox("Auto Terrain Altitude", &use_auto)) {
          sensor.set_use_auto_elevation(use_auto);
        }

        if (!use_auto) {
          if (ImGui::InputDouble("Manual Ground Elev", &ground_elevation)) {
            sensor.set_ground_elevation(ground_elevation);
          }
        }
      }

      // If Auto is enabled, try to display the actual fetched elevation
      if (use_auto) {
        float fetched_h;
        if (elevation_service.get_elevation(
                sensor.get_latitude(), sensor.get_longitude(), fetched_h)) {
          ground_elevation = static_cast<double>(fetched_h);
          // Auto-update the sensor's ground elevation too so RF model sees it
          sensor.set_ground_elevation(ground_elevation);
        }
      }

      ImGui::BeginDisabled(true);
      ImGui::InputDouble("Ground Altitude (m) (AMSL)", &ground_elevation, 0.0,
                         0.0, "%.1f");
      ImGui::EndDisabled();

      ImGui::ColorEdit3("Color", sensor.get_color_data());
    } else {
      ImGui::Text("No sensor selected.");
    }
  }
  ImGui::End();

  if (ImGui::Begin("Map View")) {
    // RF Gradient Toggle
    ImGui::Checkbox("Show RF Signal Gradient", &map.m_show_rf_gradient);
    ImGui::Checkbox("Show 3D Buildings", &map.m_show_buildings);

    map.draw(sensors, selected_index, elevation_service,
             [&](double lat, double lon) {
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
} // namespace sensor_mapper
