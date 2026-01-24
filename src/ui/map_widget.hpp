#pragma once

#include "core/building_service.hpp"
#include "core/elevation_service.hpp"
#include "core/geo_math.hpp"
#include "core/sensor.hpp"
#include "core/tdoa_solver.hpp"
#include "core/tile_service.hpp"
#include "imgui.h"
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <vector>

namespace sensor_mapper
{

class map_widget_t
{
public:
  map_widget_t();
  ~map_widget_t();

  auto update() -> void;
  auto draw(const std::vector<sensor_t> &sensors, int &selected_index, elevation_service_t &elevation_service, std::function<void(double, double)> on_add_sensor) -> void;

  auto get_building_at_location(double lat, double lon) const -> double; // Returns height or 0
  auto fetch_buildings_near(double lat, double lon) -> void;

  auto set_center(double lat, double lon) -> void;
  auto set_zoom(double zoom) -> void;

  auto set_map_source(int source_index) -> void;

  auto get_zoom() const -> double;

  auto get_mouse_lat() const -> double
  {
    return m_mouse_lat;
  }
  auto get_mouse_lon() const -> double
  {
    return m_mouse_lon;
  }

  auto get_show_rf_gradient() const -> bool
  {
    return m_show_rf_gradient;
  }
  auto set_show_rf_gradient(bool show) -> void
  {
    m_show_rf_gradient = show;
  }

  auto get_show_composite() const -> bool
  {
    return m_show_composite;
  }
  auto set_show_composite(bool show) -> void
  {
    m_show_composite = show;
  }

  auto get_show_buildings() const -> bool
  {
    return m_show_buildings;
  }
  auto set_show_buildings(bool show) -> void
  {
    m_show_buildings = show;
  }

  auto get_show_elevation_sources() const -> bool
  {
    return m_show_elevation_sources;
  }
  auto set_show_elevation_sources(bool show) -> void
  {
    m_show_elevation_sources = show;
  }

  auto get_show_raster_visual() const -> bool
  {
    return m_show_raster_visual;
  }
  auto set_show_raster_visual(bool show) -> void
  {
    m_show_raster_visual = show;
  }

  auto get_show_heatmap_overlay() const -> bool
  {
    return m_show_heatmap_overlay;
  }
  auto set_show_heatmap_overlay(bool show) -> void
  {
    m_show_heatmap_overlay = show;
  }

  auto get_min_signal_dbm() const -> float
  {
    return m_min_signal_dbm;
  }
  auto set_min_signal_dbm(float dbm) -> void
  {
    m_min_signal_dbm = dbm;
    m_heatmap_dirty = true; // Trigger re-render with new threshold
  }

  // Mark the RF heatmap as dirty to trigger re-render
  auto invalidate_rf_heatmap() -> void
  {
    m_heatmap_dirty = true;
  }

  // TDOA visualization controls
  auto get_show_tdoa_analysis() const -> bool
  {
    return m_show_tdoa_analysis;
  }
  auto set_show_tdoa_analysis(bool show) -> void
  {
    m_show_tdoa_analysis = show;
  }

  auto get_show_hyperbolas() const -> bool
  {
    return m_show_hyperbolas;
  }
  auto set_show_hyperbolas(bool show) -> void
  {
    m_show_hyperbolas = show;
  }

  auto get_show_gdop_contours() const -> bool
  {
    return m_show_gdop_contours;
  }
  auto set_show_gdop_contours(bool show) -> void
  {
    m_show_gdop_contours = show;
  }

  auto get_show_accuracy_heatmap() const -> bool
  {
    return m_show_accuracy_heatmap;
  }
  auto set_show_accuracy_heatmap(bool show) -> void
  {
    m_show_accuracy_heatmap = show;
  }

  // Test point management
  auto set_tdoa_test_point(double lat, double lon) -> void;
  auto get_tdoa_test_result() const -> const tdoa_result_t &
  {
    return m_test_result;
  }
  auto has_tdoa_test_point() const -> bool
  {
    return m_has_test_point;
  }
  auto clear_tdoa_test_point() -> void
  {
    m_has_test_point = false;
  }

  auto get_timing_jitter_ns() const -> float
  {
    return m_timing_jitter_ns;
  }
  auto set_timing_jitter_ns(float jitter) -> void
  {
    m_timing_jitter_ns = jitter;
  }

  // Tool State
  enum class tool_state_t
  {
    Navigate,
    PathProfile_SelectA,
    PathProfile_SelectB
  };

  auto get_tool_state() const -> tool_state_t
  {
    return m_tool_state;
  }
  auto set_tool_state(tool_state_t state)
  {
    m_tool_state = state;
  }

  auto get_show_profile_window() const -> bool
  {
    return m_show_profile_window;
  }
  auto set_show_profile_window(bool show)
  {
    m_show_profile_window = show;
  }

private:
  double m_center_lat;
  double m_center_lon;
  double m_zoom; // Double for smooth zoom

  double m_mouse_lat = 0.0;
  double m_mouse_lon = 0.0;

  // Tool State
  tool_state_t m_tool_state = tool_state_t::Navigate;

  // Path Profile State
  bool m_show_profile_window = false;
  geo_point_t m_profile_a = {0.0, 0.0};
  bool m_has_profile_a = false;
  geo_point_t m_profile_b = {0.0, 0.0};
  bool m_has_profile_b = false;

  bool m_show_rf_gradient; // Show RF signal strength gradient (default: off)
  bool m_show_composite = false;
  bool m_show_buildings = false;
  bool m_show_elevation_sources = false;
  bool m_show_raster_visual = true;
  bool m_show_heatmap_overlay = true; // Show GPU-rendered coverage overlay

  float m_min_signal_dbm = -90.0f; // Minimum signal strength to display (dBm)

  // Smooth zoom/pan state (placeholder for now, using direct integers)

  struct cached_view_t
  {
    std::string hash_key;
    std::vector<ImVec2> points;     // Lat/Lon coordinates
    std::vector<double> signal_dbm; // Pre-calculated signal strength (dBm)
  };
  std::map<std::string, cached_view_t> m_view_cache;

  std::unique_ptr<tile_service_t> m_tile_service;
  std::unique_ptr<building_service_t> m_building_service;

  // RF Engine
  std::unique_ptr<class gpu_rf_engine_t> m_rf_engine;
  // We don't need async futures anymore, GPU is immediate(ish)
  unsigned int m_heatmap_texture_id = 0;
  bool m_heatmap_dirty = true;

  // TDOA visualization state
  bool m_show_tdoa_analysis = false;
  bool m_show_hyperbolas = true;
  bool m_show_gdop_contours = false;
  bool m_show_accuracy_heatmap = false;
  float m_timing_jitter_ns = 10.0f; // Default GPS-disciplined timing

  // Test point state
  bool m_has_test_point = false;
  double m_test_point_lat = 0.0;
  double m_test_point_lon = 0.0;
  tdoa_result_t m_test_result;

  std::unique_ptr<tdoa_solver_t> m_tdoa_solver;

  // Elevation Visualization Textures
  struct source_texture_t
  {
    unsigned int id = 0;
    int w = 0, h = 0;
  };
  std::map<const class elevation_source_t *, source_texture_t> m_source_textures;

  // Rendering helpers
  auto render_hyperbolas(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void;
  auto render_gdop_contours(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void;
  auto render_accuracy_heatmap(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void;
  auto render_test_point(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void;

  auto draw_path_profile_window(elevation_service_t &elevation_service) -> void;

  // View State (Refactored from statics)
  double m_prev_min_lat = 0.0;
  double m_prev_max_lat = 0.0;
  double m_prev_min_lon = 0.0;
  double m_prev_max_lon = 0.0;
  size_t m_prev_sensor_count = 0;
  double m_last_render_time = 0.0;
  double m_cached_render_min_lat = 0.0;
  double m_cached_render_max_lat = 0.0;
  double m_cached_render_min_lon = 0.0;
  double m_cached_render_max_lon = 0.0;

  double m_cursor_alt = 0.0;    // Cached cursor altitude for display
  int m_current_map_source = 0; // Current map source selection

  // Context Menu State
  double m_ctx_lat = 0.0;
  double m_ctx_lon = 0.0;
};

} // namespace sensor_mapper
