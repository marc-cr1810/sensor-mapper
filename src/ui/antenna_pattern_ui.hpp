#pragma once

#include "../core/antenna_pattern.hpp"
#include "../core/antenna_pattern_io.hpp"
#include "../core/antenna_pattern_library.hpp"
#include "../core/antenna_pattern_viz.hpp"
#include "antenna_pattern_3d_viewer.hpp"
#include <imgui.h>
#include <memory>
#include <string>
#include <vector>

namespace sensor_mapper
{

// UI state for antenna pattern management
struct antenna_pattern_ui_state_t
{
  // Pattern library browser
  bool show_pattern_library = false;
  int selected_pattern_category = 0;
  int selected_pattern_type = 0;

  // Pattern editor
  bool show_pattern_editor = false;
  std::shared_ptr<antenna_pattern_t> editing_pattern;

  // Pattern visualizer
  bool show_pattern_visualizer = false;
  std::shared_ptr<antenna_pattern_t> visualizing_pattern;
  float viz_elevation_angle = 0.0f;
  int viz_show_3d = 0;
  viz_params_t viz_params;

  // 3D viewer instance
  std::unique_ptr<antenna_pattern_3d_viewer_t> viewer_3d;

  // File I/O
  bool show_load_dialog = false;
  bool show_save_dialog = false;
  char filepath_buffer[512] = "";
  antenna_file_format_t selected_format = antenna_file_format_t::CSV;

  // Pattern comparison
  bool show_comparison_mode = false;
  std::vector<std::shared_ptr<antenna_pattern_t>> comparison_patterns;

  // Recently used patterns
  std::vector<std::shared_ptr<antenna_pattern_t>> recent_patterns;
  size_t max_recent_patterns = 10;

  // Generation parameters
  float param_beamwidth_h = 65.0f;
  float param_beamwidth_v = 15.0f;
  float param_gain_dbi = 10.0f;
  float param_frequency_mhz = 2400.0f;
  float param_diameter_m = 1.0f;
  int param_num_elements = 8;
  float param_electrical_tilt = 0.0f;
  float param_mechanical_tilt = 0.0f;

  antenna_pattern_ui_state_t()
  {
    viz_params.min_gain_db = -40.0f;
    viz_params.max_gain_db = 0.0f;
    viz_params.normalize_to_peak = true;
    viz_params.show_3db_contour = true;
  }
};

class antenna_pattern_ui_t
{
public:
  // Main UI entry point - shows pattern selector/manager
  static auto render_pattern_selector(std::shared_ptr<antenna_pattern_t> &current_pattern, antenna_pattern_ui_state_t &state) -> bool
  {
    bool pattern_changed = false;

    ImGui::Text("Current Pattern:");

    const char *pattern_name = current_pattern ? current_pattern->name.c_str() : "None";

    if (ImGui::BeginCombo("##pattern_combo", pattern_name))
    {
      if (ImGui::Selectable("None", !current_pattern))
      {
        current_pattern = nullptr;
        pattern_changed = true;
      }

      // Recent patterns
      if (!state.recent_patterns.empty())
      {
        ImGui::Separator();
        ImGui::Text("Recent:");
        for (const auto &pattern : state.recent_patterns)
        {
          if (pattern && ImGui::Selectable(pattern->name.c_str(), current_pattern == pattern))
          {
            current_pattern = pattern;
            pattern_changed = true;
          }
        }
      }

      ImGui::EndCombo();
    }

    if (ImGui::Button("Library..."))
    {
      state.show_pattern_library = true;
    }

    ImGui::SameLine();
    if (ImGui::Button("Load..."))
    {
      state.show_load_dialog = true;
    }

    if (current_pattern)
    {
      if (ImGui::Button("Edit"))
      {
        state.editing_pattern = current_pattern;
        state.show_pattern_editor = true;
      }

      ImGui::SameLine();
      if (ImGui::Button("Visualize"))
      {
        state.visualizing_pattern = current_pattern;
        state.show_pattern_visualizer = true;
      }

      ImGui::SameLine();
      if (ImGui::Button("Save..."))
      {
        state.show_save_dialog = true;
      }
    }

    // Show dialogs
    if (state.show_pattern_library)
    {
      auto selected = render_pattern_library_dialog(state);
      if (selected)
      {
        current_pattern = selected;
        add_to_recent(state, selected);
        pattern_changed = true;
        state.show_pattern_library = false;
      }
    }

    if (state.show_load_dialog)
    {
      auto loaded = render_load_dialog(state);
      if (loaded)
      {
        current_pattern = loaded;
        add_to_recent(state, loaded);
        pattern_changed = true;
        state.show_load_dialog = false;
      }
    }

    if (state.show_save_dialog && current_pattern)
    {
      if (render_save_dialog(state, *current_pattern))
      {
        state.show_save_dialog = false;
      }
    }

    if (state.show_pattern_editor && state.editing_pattern)
    {
      render_pattern_editor(state.editing_pattern, state);
    }

    if (state.show_pattern_visualizer && state.visualizing_pattern)
    {
      render_pattern_visualizer(state.visualizing_pattern, state);
    }

    return pattern_changed;
  }

  // Pattern library browser dialog
  static auto render_pattern_library_dialog(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> selected_pattern;

    ImGui::SetNextWindowSize(ImVec2(700, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Antenna Pattern Library", &state.show_pattern_library))
    {

      // Category selection
      ImGui::BeginChild("Categories", ImVec2(200, 0), true);
      ImGui::Text("Categories:");
      ImGui::Separator();

      const char *categories[] = {"Basic Patterns", "Cellular / 5G", "WiFi / WLAN", "IoT / LoRa", "Satellite", "Point-to-Point", "Radar / Phased Array"};

      for (int i = 0; i < 7; ++i)
      {
        if (ImGui::Selectable(categories[i], state.selected_pattern_category == i))
        {
          state.selected_pattern_category = i;
          state.selected_pattern_type = 0;
        }
      }

      ImGui::EndChild();

      ImGui::SameLine();

      // Pattern types and parameters
      ImGui::BeginChild("PatternTypes", ImVec2(0, -35), true);

      switch (state.selected_pattern_category)
      {
      case 0: // Basic
        selected_pattern = render_basic_patterns(state);
        break;
      case 1: // Cellular
        selected_pattern = render_cellular_patterns(state);
        break;
      case 2: // WiFi
        selected_pattern = render_wifi_patterns(state);
        break;
      case 3: // IoT
        selected_pattern = render_iot_patterns(state);
        break;
      case 4: // Satellite
        selected_pattern = render_satellite_patterns(state);
        break;
      case 5: // Point-to-Point
        selected_pattern = render_p2p_patterns(state);
        break;
      case 6: // Radar
        selected_pattern = render_radar_patterns(state);
        break;
      }

      ImGui::EndChild();

      // Live preview area
      ImGui::Separator();
      ImGui::Text("Preview:");
      if (selected_pattern)
      {
        render_pattern_mini_preview(selected_pattern);
      }

      // Bottom buttons
      if (ImGui::Button("Close"))
      {
        state.show_pattern_library = false;
      }
    }
    ImGui::End();

    return selected_pattern;
  }

  // Pattern editor window
  static auto render_pattern_editor(std::shared_ptr<antenna_pattern_t> &pattern, antenna_pattern_ui_state_t &state) -> void
  {
    if (!pattern)
      return;

    ImGui::SetNextWindowSize(ImVec2(500, 600), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Pattern Editor", &state.show_pattern_editor))
    {

      // Basic information
      ImGui::SeparatorText("Basic Information");

      char name_buf[256];
      std::strncpy(name_buf, pattern->name.c_str(), 255);
      name_buf[255] = '\0';
      if (ImGui::InputText("Name", name_buf, 256))
      {
        pattern->name = name_buf;
      }

      char mfr_buf[256];
      std::strncpy(mfr_buf, pattern->manufacturer.c_str(), 255);
      mfr_buf[255] = '\0';
      if (ImGui::InputText("Manufacturer", mfr_buf, 256))
      {
        pattern->manufacturer = mfr_buf;
      }

      char model_buf[256];
      std::strncpy(model_buf, pattern->model.c_str(), 255);
      model_buf[255] = '\0';
      if (ImGui::InputText("Model", model_buf, 256))
      {
        pattern->model = model_buf;
      }

      ImGui::InputFloat("Frequency (MHz)", &pattern->frequency_mhz, 1.0f, 100.0f, "%.2f");

      // Polarization
      const char *pol_types[] = {"Vertical", "Horizontal", "Circular Left", "Circular Right", "Dual Slant", "Unknown"};
      int pol_idx = static_cast<int>(pattern->polarization);
      if (ImGui::Combo("Polarization", &pol_idx, pol_types, 6))
      {
        pattern->polarization = static_cast<polarization_t>(pol_idx);
      }

      // Antenna parameters
      ImGui::SeparatorText("Antenna Parameters");

      ImGui::InputFloat("Max Gain (dBi)", &pattern->max_gain_dbi, 0.1f, 1.0f, "%.2f");
      ImGui::InputFloat("H. Beamwidth (°)", &pattern->horizontal_beamwidth_deg, 1.0f, 10.0f, "%.1f");
      ImGui::InputFloat("V. Beamwidth (°)", &pattern->vertical_beamwidth_deg, 1.0f, 10.0f, "%.1f");
      ImGui::InputFloat("F/B Ratio (dB)", &pattern->front_to_back_ratio_db, 1.0f, 5.0f, "%.1f");

      // Tilt parameters
      ImGui::SeparatorText("Tilt");

      ImGui::SliderFloat("Electrical Tilt (°)", &pattern->electrical_tilt_deg, -30.0f, 30.0f, "%.1f");
      ImGui::SliderFloat("Mechanical Tilt (°)", &pattern->mechanical_tilt_deg, -30.0f, 30.0f, "%.1f");

      // Pattern data info
      ImGui::SeparatorText("Pattern Data");

      ImGui::Text("Azimuth samples: %zu", pattern->azimuth_angles.size());
      ImGui::Text("Elevation samples: %zu", pattern->elevation_angles.size());
      ImGui::Text("Pattern type: %s", pattern->is_3d() ? "3D (Full Sphere)" : "2D (Azimuth Only)");

      if (ImGui::Button("Calculate Beamwidths from Pattern"))
      {
        pattern->calculate_beamwidths();
      }

      // Preview
      ImGui::SeparatorText("Quick Preview");
      render_pattern_mini_preview(pattern);
    }
    ImGui::End();
  }

  // Pattern visualizer window
  static auto render_pattern_visualizer(std::shared_ptr<antenna_pattern_t> &pattern, antenna_pattern_ui_state_t &state) -> void
  {
    if (!pattern)
      return;

    ImGui::SetNextWindowSize(ImVec2(800, 700), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Pattern Visualizer", &state.show_pattern_visualizer))
    {

      // Visualization controls
      ImGui::BeginChild("Controls", ImVec2(250, 0), true);

      ImGui::Text("Visualization Mode:");
      ImGui::RadioButton("2D Polar Plot", &state.viz_show_3d, 0);
      ImGui::RadioButton("3D Pattern", &state.viz_show_3d, 1);

      ImGui::Separator();

      if (!state.viz_show_3d)
      {
        ImGui::SliderFloat("Elevation Cut (°)", &state.viz_elevation_angle, -90.0f, 90.0f, "%.1f");
      }

      ImGui::Checkbox("Normalize to Peak", &state.viz_params.normalize_to_peak);
      ImGui::Checkbox("Show 3dB Contour", &state.viz_params.show_3db_contour);
      ImGui::Checkbox("Show in Linear", &state.viz_params.show_in_linear);

      ImGui::SliderFloat("Min Gain (dB)", &state.viz_params.min_gain_db, -60.0f, -10.0f, "%.0f");
      ImGui::SliderFloat("Max Gain (dB)", &state.viz_params.max_gain_db, -10.0f, 30.0f, "%.0f");

      ImGui::SliderInt("Resolution (°)", &state.viz_params.resolution_deg, 1, 10);

      ImGui::Separator();

      ImGui::Text("Pattern Info:");
      ImGui::Text("Name: %s", pattern->name.c_str());
      ImGui::Text("Max Gain: %.2f dBi", pattern->max_gain_dbi);
      ImGui::Text("H. BW: %.1f°", pattern->horizontal_beamwidth_deg);
      ImGui::Text("V. BW: %.1f°", pattern->vertical_beamwidth_deg);
      ImGui::Text("F/B: %.1f dB", pattern->front_to_back_ratio_db);

      ImGui::EndChild();

      ImGui::SameLine();

      // Visualization display
      ImGui::BeginChild("Display", ImVec2(0, 0), true);

      if (!state.viz_show_3d)
      {
        // 2D polar plot
        render_polar_plot(*pattern, state.viz_elevation_angle, state.viz_params);
      }
      else
      {
        // 3D visualization with OpenGL
        if (!state.viewer_3d)
        {
          state.viewer_3d = std::make_unique<antenna_pattern_3d_viewer_t>();
        }
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
        state.viewer_3d->render(*pattern, state.viz_params, canvas_size);
      }

      ImGui::EndChild();
    }
    ImGui::End();
  }

  // File load dialog
  static auto render_load_dialog(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> loaded_pattern;

    ImGui::SetNextWindowSize(ImVec2(500, 200), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Load Antenna Pattern", &state.show_load_dialog))
    {

      ImGui::Text("File Path:");
      ImGui::InputText("##filepath", state.filepath_buffer, 512);

      ImGui::Text("Format:");
      const char *formats[] = {"Auto-Detect", "CSV", "JSON", "MSI Planet", "NSMA", "XML"};
      int format_idx = static_cast<int>(state.selected_format);
      ImGui::Combo("##format", &format_idx, formats, 6);
      state.selected_format = static_cast<antenna_file_format_t>(format_idx);

      if (ImGui::Button("Load"))
      {
        auto result = antenna_pattern_io_t::load_from_file(state.filepath_buffer, state.selected_format);
        if (result.success)
        {
          loaded_pattern = result.pattern;
        }
        else
        {
          // Show error (in a real app, use a proper message box)
          ImGui::OpenPopup("Load Error");
        }
      }

      ImGui::SameLine();
      if (ImGui::Button("Cancel"))
      {
        state.show_load_dialog = false;
      }

      // Error popup
      if (ImGui::BeginPopupModal("Load Error", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
      {
        ImGui::Text("Failed to load pattern file.");
        if (ImGui::Button("OK"))
        {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
    }
    ImGui::End();

    return loaded_pattern;
  }

  // File save dialog
  static auto render_save_dialog(antenna_pattern_ui_state_t &state, const antenna_pattern_t &pattern) -> bool
  {
    bool saved = false;

    ImGui::SetNextWindowSize(ImVec2(500, 200), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Save Antenna Pattern", &state.show_save_dialog))
    {

      ImGui::Text("File Path:");
      ImGui::InputText("##filepath", state.filepath_buffer, 512);

      ImGui::Text("Format:");
      const char *formats[] = {"CSV", "JSON"};
      int format_idx = (state.selected_format == antenna_file_format_t::JSON) ? 1 : 0;
      ImGui::Combo("##format", &format_idx, formats, 2);
      state.selected_format = (format_idx == 1) ? antenna_file_format_t::JSON : antenna_file_format_t::CSV;

      if (ImGui::Button("Save"))
      {
        auto result = antenna_pattern_io_t::save_to_file(pattern, state.filepath_buffer, state.selected_format);
        if (result.success)
        {
          saved = true;
        }
        else
        {
          ImGui::OpenPopup("Save Error");
        }
      }

      ImGui::SameLine();
      if (ImGui::Button("Cancel"))
      {
        saved = true; // Close without saving
      }

      // Error popup
      if (ImGui::BeginPopupModal("Save Error", nullptr, ImGuiWindowFlags_AlwaysAutoResize))
      {
        ImGui::Text("Failed to save pattern file.");
        if (ImGui::Button("OK"))
        {
          ImGui::CloseCurrentPopup();
        }
        ImGui::EndPopup();
      }
    }
    ImGui::End();

    return saved;
  }

private:
  // Helper: Add pattern to recent list
  static auto add_to_recent(antenna_pattern_ui_state_t &state, std::shared_ptr<antenna_pattern_t> pattern) -> void
  {
    // Remove if already in list
    auto it = std::find(state.recent_patterns.begin(), state.recent_patterns.end(), pattern);
    if (it != state.recent_patterns.end())
    {
      state.recent_patterns.erase(it);
    }

    // Add to front
    state.recent_patterns.insert(state.recent_patterns.begin(), pattern);

    // Trim to max size
    if (state.recent_patterns.size() > state.max_recent_patterns)
    {
      state.recent_patterns.resize(state.max_recent_patterns);
    }
  }

  // Pattern generation helpers for each category
  static auto render_basic_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"Omnidirectional", "Isotropic", "Dipole", "Sector", "Patch", "Yagi", "Horn", "Parabolic", "Bi-directional"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 9, 10);

    ImGui::Separator();
    ImGui::Text("Parameters:");

    // Live preview temporary pattern
    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // Omni
      preview_pattern = antenna_library_t::generate_omni();
      if (ImGui::Button("Generate Omnidirectional"))
      {
        pattern = antenna_library_t::generate_omni();
      }
      break;
    case 1: // Isotropic
      preview_pattern = antenna_library_t::generate_isotropic();
      if (ImGui::Button("Generate Isotropic"))
      {
        pattern = antenna_library_t::generate_isotropic();
      }
      break;
    case 2: // Dipole
      preview_pattern = antenna_library_t::generate_dipole();
      if (ImGui::Button("Generate Dipole"))
      {
        pattern = antenna_library_t::generate_dipole();
      }
      break;
    case 3: // Sector
      ImGui::SliderFloat("Beamwidth (°)", &state.param_beamwidth_h, 30.0f, 180.0f);
      preview_pattern = antenna_library_t::generate_sector(state.param_beamwidth_h);
      if (ImGui::Button("Generate Sector"))
      {
        pattern = antenna_library_t::generate_sector(state.param_beamwidth_h);
      }
      break;
    case 4: // Patch
      ImGui::SliderFloat("H. Beamwidth (°)", &state.param_beamwidth_h, 30.0f, 120.0f);
      ImGui::SliderFloat("V. Beamwidth (°)", &state.param_beamwidth_v, 10.0f, 90.0f);
      ImGui::SliderFloat("Gain (dBi)", &state.param_gain_dbi, 2.0f, 15.0f);
      preview_pattern = antenna_library_t::generate_patch(state.param_beamwidth_h, state.param_beamwidth_v, state.param_gain_dbi);
      if (ImGui::Button("Generate Patch"))
      {
        pattern = antenna_library_t::generate_patch(state.param_beamwidth_h, state.param_beamwidth_v, state.param_gain_dbi);
      }
      break;
    case 5: // Yagi
      ImGui::SliderFloat("Beamwidth (°)", &state.param_beamwidth_h, 20.0f, 90.0f);
      ImGui::SliderInt("Elements", &state.param_num_elements, 3, 12);
      preview_pattern = antenna_library_t::generate_yagi(state.param_beamwidth_h, state.param_num_elements);
      if (ImGui::Button("Generate Yagi"))
      {
        pattern = antenna_library_t::generate_yagi(state.param_beamwidth_h, state.param_num_elements);
      }
      break;
    case 6: // Horn
      ImGui::SliderFloat("Beamwidth (°)", &state.param_beamwidth_h, 10.0f, 60.0f);
      ImGui::SliderFloat("Gain (dBi)", &state.param_gain_dbi, 10.0f, 25.0f);
      preview_pattern = antenna_library_t::generate_horn(state.param_beamwidth_h, state.param_gain_dbi);
      if (ImGui::Button("Generate Horn"))
      {
        pattern = antenna_library_t::generate_horn(state.param_beamwidth_h, state.param_gain_dbi);
      }
      break;
    case 7: // Parabolic
      ImGui::SliderFloat("Diameter (m)", &state.param_diameter_m, 0.3f, 5.0f);
      ImGui::InputFloat("Frequency (MHz)", &state.param_frequency_mhz, 100.0f, 1000.0f);
      preview_pattern = antenna_library_t::generate_parabolic(state.param_diameter_m, state.param_frequency_mhz);
      if (ImGui::Button("Generate Parabolic"))
      {
        pattern = antenna_library_t::generate_parabolic(state.param_diameter_m, state.param_frequency_mhz);
      }
      break;
    case 8: // Bi-directional
      ImGui::SliderFloat("Beamwidth (°)", &state.param_beamwidth_h, 20.0f, 90.0f);
      preview_pattern = antenna_library_t::generate_bidirectional(state.param_beamwidth_h);
      if (ImGui::Button("Generate Bi-directional"))
      {
        pattern = antenna_library_t::generate_bidirectional(state.param_beamwidth_h);
      }
      break;
    }

    // Show live preview
    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_cellular_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"Sector Antenna", "Small Cell Omni", "Massive MIMO"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 3, 5);

    ImGui::Separator();
    ImGui::Text("Parameters:");
    ImGui::InputFloat("Frequency (MHz)", &state.param_frequency_mhz, 100.0f, 1000.0f);

    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // Sector
      ImGui::SliderFloat("E-Tilt (°)", &state.param_electrical_tilt, 0.0f, 15.0f);
      preview_pattern = antenna_pattern_library_advanced_t::generate_cellular_sector_antenna(state.param_frequency_mhz, state.param_electrical_tilt);
      if (ImGui::Button("Generate Cellular Sector"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_cellular_sector_antenna(state.param_frequency_mhz, state.param_electrical_tilt);
      }
      break;
    case 1: // Small Cell
      preview_pattern = antenna_pattern_library_advanced_t::generate_small_cell_omni(state.param_frequency_mhz);
      if (ImGui::Button("Generate Small Cell Omni"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_small_cell_omni(state.param_frequency_mhz);
      }
      break;
    case 2: // MIMO
      ImGui::SliderInt("H. Elements", &state.param_num_elements, 4, 16);
      int v_elements = state.param_num_elements / 2;
      ImGui::SliderInt("V. Elements", &v_elements, 2, 8);
      preview_pattern = antenna_pattern_library_advanced_t::generate_massive_mimo_array(state.param_num_elements, v_elements, state.param_frequency_mhz);
      if (ImGui::Button("Generate Massive MIMO"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_massive_mimo_array(state.param_num_elements, v_elements, state.param_frequency_mhz);
      }
      break;
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_wifi_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"WiFi Dipole", "WiFi Patch Panel"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 2, 4);

    ImGui::Separator();

    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // Dipole
      preview_pattern = antenna_pattern_library_advanced_t::generate_wifi_dipole();
      if (ImGui::Button("Generate WiFi Dipole"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_wifi_dipole();
      }
      break;
    case 1: // Patch
      ImGui::SliderFloat("Beamwidth (°)", &state.param_beamwidth_h, 60.0f, 120.0f);
      preview_pattern = antenna_pattern_library_advanced_t::generate_wifi_patch_panel(state.param_beamwidth_h);
      if (ImGui::Button("Generate WiFi Patch"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_wifi_patch_panel(state.param_beamwidth_h);
      }
      break;
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_iot_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"LoRa Gateway Collinear", "IoT Chip Antenna"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 2, 4);

    ImGui::Separator();

    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // LoRa
      ImGui::SliderInt("Sections", &state.param_num_elements, 2, 8);
      preview_pattern = antenna_pattern_library_advanced_t::generate_lora_gateway_collinear(state.param_num_elements);
      if (ImGui::Button("Generate LoRa Gateway"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_lora_gateway_collinear(state.param_num_elements);
      }
      break;
    case 1: // Chip
      preview_pattern = antenna_pattern_library_advanced_t::generate_iot_chip_antenna();
      if (ImGui::Button("Generate IoT Chip Antenna"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_iot_chip_antenna();
      }
      break;
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_satellite_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"GNSS/GPS Antenna", "Satellite Dish"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 2, 4);

    ImGui::Separator();

    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // GNSS
      preview_pattern = antenna_pattern_library_advanced_t::generate_gnss_antenna();
      if (ImGui::Button("Generate GNSS Antenna"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_gnss_antenna();
      }
      break;
    case 1: // Dish
      ImGui::SliderFloat("Diameter (m)", &state.param_diameter_m, 0.3f, 3.0f);
      ImGui::InputFloat("Frequency (GHz)", &state.param_frequency_mhz, 1.0f, 10.0f);
      preview_pattern = antenna_pattern_library_advanced_t::generate_satellite_dish(state.param_diameter_m, state.param_frequency_mhz / 1000.0f);
      if (ImGui::Button("Generate Satellite Dish"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_satellite_dish(state.param_diameter_m, state.param_frequency_mhz / 1000.0f);
      }
      break;
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_p2p_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    ImGui::Text("Microwave Backhaul");
    ImGui::SliderFloat("Diameter (m)", &state.param_diameter_m, 0.3f, 1.5f);
    ImGui::InputFloat("Frequency (GHz)", &state.param_frequency_mhz, 1.0f, 10.0f);

    auto preview_pattern = antenna_pattern_library_advanced_t::generate_microwave_backhaul(state.param_diameter_m, state.param_frequency_mhz / 1000.0f);

    if (ImGui::Button("Generate Microwave Backhaul"))
    {
      pattern = antenna_pattern_library_advanced_t::generate_microwave_backhaul(state.param_diameter_m, state.param_frequency_mhz / 1000.0f);
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  static auto render_radar_patterns(antenna_pattern_ui_state_t &state) -> std::shared_ptr<antenna_pattern_t>
  {
    std::shared_ptr<antenna_pattern_t> pattern;

    const char *types[] = {"Phased Array Radar", "Beamformed Pattern"};
    ImGui::ListBox("##pattern_types", &state.selected_pattern_type, types, 2, 4);

    ImGui::Separator();

    std::shared_ptr<antenna_pattern_t> preview_pattern;

    switch (state.selected_pattern_type)
    {
    case 0: // Phased Array
      ImGui::SliderInt("Elements", &state.param_num_elements, 16, 128);
      ImGui::InputFloat("Frequency (GHz)", &state.param_frequency_mhz, 1.0f, 10.0f);
      preview_pattern = antenna_pattern_library_advanced_t::generate_phased_array_radar(state.param_num_elements, state.param_frequency_mhz / 1000.0f, 0.0f);
      if (ImGui::Button("Generate Phased Array"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_phased_array_radar(state.param_num_elements, state.param_frequency_mhz / 1000.0f, 0.0f);
      }
      break;
    case 1: // Beamformed
      ImGui::SliderFloat("Beam Azimuth (°)", &state.param_beamwidth_h, 0.0f, 360.0f);
      ImGui::SliderFloat("Beam Elevation (°)", &state.param_beamwidth_v, -30.0f, 30.0f);
      ImGui::SliderInt("Elements", &state.param_num_elements, 8, 64);
      preview_pattern = antenna_pattern_library_advanced_t::generate_beamformed_pattern(state.param_beamwidth_h, state.param_beamwidth_v, 15.0f, state.param_num_elements);
      if (ImGui::Button("Generate Beamformed Pattern"))
      {
        pattern = antenna_pattern_library_advanced_t::generate_beamformed_pattern(state.param_beamwidth_h, state.param_beamwidth_v, 15.0f, state.param_num_elements);
      }
      break;
    }

    if (preview_pattern)
    {
      ImGui::Separator();
      ImGui::Text("Live Preview:");
      render_pattern_mini_preview(preview_pattern);
    }

    return pattern;
  }

  // Mini preview for pattern editor
  static auto render_pattern_mini_preview(std::shared_ptr<antenna_pattern_t> &pattern) -> void
  {
    if (!pattern)
      return;

    ImVec2 canvas_size = ImVec2(200, 200);
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // Draw background
    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(20, 20, 20, 255));

    // Draw polar grid
    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.5f);
    float radius = canvas_size.x * 0.4f;

    // Grid circles
    for (int i = 1; i <= 4; ++i)
    {
      float r = radius * i / 4.0f;
      draw_list->AddCircle(center, r, IM_COL32(60, 60, 60, 255), 32, 1.0f);
    }

    // Draw pattern
    std::vector<ImVec2> points;
    for (int angle = 0; angle < 360; ++angle)
    {
      float gain_db = pattern->get_gain(static_cast<float>(angle), 0.0f, true);
      float normalized = (gain_db - (-40.0f)) / 40.0f;
      normalized = std::clamp(normalized, 0.0f, 1.0f);

      float angle_rad = angle * M_PI / 180.0f;
      float x = center.x + std::sin(angle_rad) * radius * normalized;
      float y = center.y - std::cos(angle_rad) * radius * normalized;
      points.push_back(ImVec2(x, y));
    }

    // Draw pattern shape
    for (size_t i = 0; i < points.size(); ++i)
    {
      size_t next = (i + 1) % points.size();
      draw_list->AddLine(points[i], points[next], IM_COL32(0, 200, 255, 255), 2.0f);
    }

    ImGui::Dummy(canvas_size);
  }

  // Render polar plot
  static auto render_polar_plot(const antenna_pattern_t &pattern, float elevation_deg, const viz_params_t &params) -> void
  {
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    canvas_size.y = canvas_size.x; // Make it square
    if (canvas_size.x < 100.0f)
      canvas_size.x = 400.0f;
    if (canvas_size.y < 100.0f)
      canvas_size.y = 400.0f;

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // Background
    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(15, 15, 15, 255));

    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.5f);
    float radius = std::min(canvas_size.x, canvas_size.y) * 0.4f;

    // Draw polar grid
    for (int i = 1; i <= 4; ++i)
    {
      float r = radius * i / 4.0f;
      draw_list->AddCircle(center, r, IM_COL32(50, 50, 50, 255), 64, 1.5f);
    }

    // Draw angle lines
    for (int angle = 0; angle < 360; angle += 30)
    {
      float angle_rad = static_cast<float>(angle * M_PI / 180.0);
      float x = center.x + std::sin(angle_rad) * radius;
      float y = center.y - std::cos(angle_rad) * radius;
      draw_list->AddLine(center, ImVec2(x, y), IM_COL32(50, 50, 50, 255), 1.0f);
    }

    // Generate pattern data
    auto plot_data = antenna_pattern_viz_t::generate_polar_plot(pattern, elevation_deg, params);

    // Draw pattern
    std::vector<ImVec2> points;
    for (const auto &point : plot_data.points)
    {
      float angle_rad = static_cast<float>(point.angle_deg * M_PI / 180.0);
      float r = point.radius * radius;
      float x = center.x + std::sin(angle_rad) * r;
      float y = center.y - std::cos(angle_rad) * r;
      points.push_back(ImVec2(x, y));
    }

    // Fill pattern area
    if (!points.empty())
    {
      draw_list->AddConvexPolyFilled(points.data(), static_cast<int>(points.size()), IM_COL32(0, 150, 255, 80));

      // Draw outline
      for (size_t i = 0; i < points.size(); ++i)
      {
        size_t next = (i + 1) % points.size();
        draw_list->AddLine(points[i], points[next], IM_COL32(0, 200, 255, 255), 2.5f);
      }
    }

    // Draw 3dB beamwidth indicator
    if (params.show_3db_contour)
    {
      float bw = pattern.horizontal_beamwidth_deg / 2.0f;
      float angle1_rad = static_cast<float>(-bw * M_PI / 180.0);
      float angle2_rad = static_cast<float>(bw * M_PI / 180.0);

      float x1 = center.x + std::sin(angle1_rad) * radius;
      float y1 = center.y - std::cos(angle1_rad) * radius;
      float x2 = center.x + std::sin(angle2_rad) * radius;
      float y2 = center.y - std::cos(angle2_rad) * radius;

      draw_list->AddLine(center, ImVec2(x1, y1), IM_COL32(255, 255, 0, 180), 2.0f);
      draw_list->AddLine(center, ImVec2(x2, y2), IM_COL32(255, 255, 0, 180), 2.0f);
    }

    ImGui::Dummy(canvas_size);
  }

  // Render 3D pattern view (stacked elevation cuts)
  static auto render_3d_pattern_view(const antenna_pattern_t &pattern, const viz_params_t &params) -> void
  {
    ImVec2 canvas_size = ImGui::GetContentRegionAvail();
    if (canvas_size.x < 100.0f)
      canvas_size.x = 500.0f;
    if (canvas_size.y < 100.0f)
      canvas_size.y = 500.0f;

    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // Background
    draw_list->AddRectFilled(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), IM_COL32(15, 15, 15, 255));

    ImVec2 center = ImVec2(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.6f);
    float base_radius = std::min(canvas_size.x, canvas_size.y) * 0.35f;

    // Draw multiple elevation cuts stacked in 3D perspective
    int num_elevation_cuts = 7;
    for (int elev_idx = 0; elev_idx < num_elevation_cuts; ++elev_idx)
    {
      float elev = -60.0f + (elev_idx * 120.0f / (num_elevation_cuts - 1));

      // 3D perspective: lower elevations appear smaller and higher
      float depth_factor = 0.3f + 0.7f * (elev_idx / (float)(num_elevation_cuts - 1));
      float y_offset = -100.0f * (1.0f - depth_factor);
      float radius = base_radius * depth_factor;

      ImVec2 slice_center = ImVec2(center.x, center.y + y_offset);

      // Draw reference circle
      draw_list->AddCircle(slice_center, radius, IM_COL32(50, 50, 50, 150), 64, 1.0f);

      // Generate pattern points for this elevation
      std::vector<ImVec2> points;
      for (int angle = 0; angle <= 360; angle += 3)
      {
        float gain_db = pattern.get_gain(static_cast<float>(angle % 360), elev, true);

        if (params.normalize_to_peak)
        {
          gain_db -= pattern.max_gain_dbi;
        }

        float normalized = (gain_db - params.min_gain_db) / (params.max_gain_db - params.min_gain_db);
        normalized = std::clamp(normalized, 0.0f, 1.0f);

        float angle_rad = static_cast<float>(angle * M_PI / 180.0);
        float r = radius * normalized;
        float x = slice_center.x + std::sin(angle_rad) * r;
        float y = slice_center.y - std::cos(angle_rad) * r;
        points.push_back(ImVec2(x, y));
      }

      // Color based on elevation
      float color_t = (elev + 60.0f) / 120.0f;
      ImU32 color = IM_COL32(static_cast<int>(100 + 155 * color_t), static_cast<int>(150 + 105 * (1.0f - std::abs(color_t - 0.5f) * 2.0f)), static_cast<int>(255 - 155 * color_t), 200);

      // Fill the pattern shape
      if (!points.empty())
      {
        draw_list->AddConvexPolyFilled(points.data(), static_cast<int>(points.size()), IM_COL32((color >> 0) & 0xFF, (color >> 8) & 0xFF, (color >> 16) & 0xFF, 100));

        // Draw outline
        for (size_t i = 0; i < points.size() - 1; ++i)
        {
          draw_list->AddLine(points[i], points[i + 1], color, 2.0f);
        }
      }

      // Label elevation angle
      char label[32];
      snprintf(label, sizeof(label), "%+.0f°", elev);
      draw_list->AddText(ImVec2(slice_center.x + radius + 5, slice_center.y - 10), IM_COL32(200, 200, 200, 255), label);
    }

    // Title
    draw_list->AddText(ImVec2(canvas_pos.x + 10, canvas_pos.y + 10), IM_COL32(255, 255, 255, 255), "3D Pattern (Stacked Elevation Cuts)");

    ImGui::Dummy(canvas_size);
  }
};

} // namespace sensor_mapper