#pragma once

#include "core/building_service.hpp"
#include "core/elevation_service.hpp"
#include "core/sensor.hpp"
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

private:
  double m_center_lat;
  double m_center_lon;
  double m_zoom; // Double for smooth zoom

  double m_mouse_lat = 0.0;
  double m_mouse_lon = 0.0;

  bool m_show_rf_gradient; // Show RF signal strength gradient (default: off)
  bool m_show_composite = false;
  bool m_show_buildings = false;
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
};

} // namespace sensor_mapper
