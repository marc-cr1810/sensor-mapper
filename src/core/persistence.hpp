#pragma once

#include "core/sensor.hpp"
#include <string>
#include <vector>

namespace sensor_mapper
{
namespace persistence
{

auto save_sensors(const std::string &filename, const std::vector<sensor_t> &sensors) -> bool;
auto load_sensors(const std::string &filename, std::vector<sensor_t> &sensors) -> bool;

struct workspace_t
{
  struct camera_t
  {
    double lat = 0.0;
    double lon = 0.0;
    double zoom = 0.0;
  } camera;

  struct settings_t
  {
    float min_signal_dbm = -90.0f;
    int viz_mode = 0; // 0=Heatmap, 1=Overlap
  } settings;

  struct layers_t
  {
    bool show_rf = true;
    bool show_heatmap = true;
    bool show_composite = false;
    bool show_buildings = false;
    bool show_elevation = false;
    bool show_tdoa = false;
    bool show_hyperbolas = true;
    bool show_gdop = false;
    bool show_accuracy = false;
  } layers;

  struct data_t
  {
    std::string sensor_file = "sensors.json";
    std::vector<std::string> elevation_files;
  } data;
};

auto save_workspace(const std::string &filename, const workspace_t &workspace) -> bool;
auto load_workspace(const std::string &filename, workspace_t &workspace) -> bool;

} // namespace persistence
} // namespace sensor_mapper
