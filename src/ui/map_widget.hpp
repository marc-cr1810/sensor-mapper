#pragma once

#include "core/building_service.hpp"
#include "core/elevation_service.hpp"
#include "core/sensor.hpp"
#include "core/tile_service.hpp"
#include "imgui.h"
#include <functional>
#include <map>
#include <memory>
#include <vector>

namespace sensor_mapper {

class map_widget_t {
public:
  map_widget_t();
  ~map_widget_t();

  auto update() -> void;
  auto draw(const std::vector<sensor_t> &sensors, int &selected_index,
            elevation_service_t &elevation_service,
            std::function<void(double, double)> on_add_sensor) -> void;

  auto get_building_at_location(double lat, double lon) const
      -> double; // Returns height or 0
  auto fetch_buildings_near(double lat, double lon) -> void;

  auto set_center(double lat, double lon) -> void;
  auto set_zoom(double zoom) -> void;

  auto set_map_source(int source_index) -> void;

  auto get_zoom() const -> double;

  auto get_mouse_lat() const -> double { return m_mouse_lat; }
  auto get_mouse_lon() const -> double { return m_mouse_lon; }

  bool m_show_rf_gradient; // Show RF signal strength gradient (default: off)
  bool m_show_buildings = false;

private:
  double m_center_lat;
  double m_center_lon;
  double m_zoom; // Double for smooth zoom

  double m_mouse_lat = 0.0;
  double m_mouse_lon = 0.0;

  // Smooth zoom/pan state (placeholder for now, using direct integers)

  struct cached_view_t {
    std::string hash_key;
    std::vector<ImVec2> points;     // Lat/Lon coordinates
    std::vector<double> signal_dbm; // Pre-calculated signal strength (dBm)
  };
  std::map<std::string, cached_view_t> m_view_cache;

  std::unique_ptr<tile_service_t> m_tile_service;
  std::unique_ptr<building_service_t> m_building_service;
};

} // namespace sensor_mapper
