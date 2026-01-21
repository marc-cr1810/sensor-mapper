#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <stdio.h>

#include "core/sensor.hpp"
#include "ui/map_widget.hpp"

// Forward declaration
namespace sensor_mapper {
void render_ui(map_widget_t &map, sensor_t &sensor);
}

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
  sensor_mapper::sensor_t main_sensor("Primary Sensor", 0.0, 0.0, 100.0);

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    // Poll and handle events
    glfwPollEvents();

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Core UI Logic
    sensor_mapper::render_ui(map_widget, main_sensor);

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
void render_ui(map_widget_t &map, sensor_t &sensor) {
  // Dockspace
  ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport());

  if (ImGui::Begin("Sensor Configuration")) {
    // Sensor Controls
    char name_buffer[128];
    snprintf(name_buffer, sizeof(name_buffer), "%s", sensor.get_name().c_str());
    if (ImGui::InputText("Name", name_buffer, sizeof(name_buffer))) {
      sensor.set_name(std::string(name_buffer));
    }

    double lat = sensor.get_latitude();
    if (ImGui::InputDouble("Latitude", &lat)) {
      sensor.set_latitude(lat);
    }

    double lon = sensor.get_longitude();
    if (ImGui::InputDouble("Longitude", &lon)) {
      sensor.set_longitude(lon);
    }

    double range = sensor.get_range();
    if (ImGui::InputDouble("Range (m)", &range)) {
      sensor.set_range(range);
    }
  }
  ImGui::End();

  if (ImGui::Begin("Map View")) {
    map.draw(sensor);
  }
  ImGui::End();
}
} // namespace sensor_mapper
