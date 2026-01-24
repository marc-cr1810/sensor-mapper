#include <glad/glad.h>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GLFW/glfw3.h>
#include <iostream>
// #include <vector> // Removed (transitive)
#include "core/persistence.hpp"
// #include "core/sensor.hpp" // Removed (transitive)
#include "ui/app_ui.hpp"

// Forward declaration
namespace sensor_mapper
{
// void render_ui(...); // Removed in favor of AppUI class
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

  // UI Manager
  sensor_mapper::AppUI app_ui;
  app_ui.setup_style();

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
    app_ui.render(map_widget, sensors, selected_sensor_index, elevation_service, antenna_ui_state, [&exit_ctx]() { glfwSetWindowShouldClose(exit_ctx.win, 1); });

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.12f, 1.00f);
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
