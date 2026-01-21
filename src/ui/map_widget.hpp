#pragma once

#include "core/sensor.hpp"
#include "core/tile_service.hpp"
#include "imgui.h"
#include <memory>

namespace sensor_mapper {

class map_widget_t {
public:
  map_widget_t();
  ~map_widget_t();

  auto update() -> void;
  auto draw(const sensor_t &sensor) -> void;

  auto set_center(double lat, double lon) -> void;
  auto set_zoom(double zoom) -> void;

  auto get_zoom() const -> double;

private:
  double m_center_lat;
  double m_center_lon;
  double m_zoom; // Double for smooth zoom

  // Smooth zoom/pan state (placeholder for now, using direct integers)

  std::unique_ptr<tile_service_t> m_tile_service;
};

} // namespace sensor_mapper
