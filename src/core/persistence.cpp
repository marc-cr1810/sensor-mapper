#include "core/persistence.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace sensor_mapper {
namespace persistence {

auto save_sensors(const std::string &filename,
                  const std::vector<sensor_t> &sensors) -> bool {
  json j;
  for (const auto &sensor : sensors) {
    auto color = sensor.get_color();
    j.push_back({{"name", sensor.get_name()},
                 {"latitude", sensor.get_latitude()},
                 {"longitude", sensor.get_longitude()},
                 {"range", sensor.get_range()},
                 {"altitude", sensor.get_altitude()},
                 {"color", {color[0], color[1], color[2]}}});
  }

  std::ofstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  file << j.dump(4);
  return true;
}

auto load_sensors(const std::string &filename, std::vector<sensor_t> &sensors)
    -> bool {
  std::ifstream file(filename);
  if (!file.is_open()) {
    return false;
  }

  json j;
  try {
    file >> j;
  } catch (const json::parse_error &e) {
    std::cerr << "JSON Parse Error: " << e.what() << std::endl;
    return false;
  }

  sensors.clear();
  for (const auto &item : j) {
    std::string name = item.value("name", "Unknown Sensor");
    double lat = item.value("latitude", 0.0);
    double lon = item.value("longitude", 0.0);
    double range = item.value("range", 1000.0);
    double altitude = item.value("altitude", 10.0);

    // Create sensor
    sensors.emplace_back(name, lat, lon, range);
    sensors.back().set_altitude(altitude);

    // Restore color if present
    if (item.contains("color") && item["color"].is_array() &&
        item["color"].size() == 3) {
      std::array<float, 3> color;
      color[0] = item["color"][0];
      color[1] = item["color"][1];
      color[2] = item["color"][2];

      // Need a way to set color - assuming we can access it or need a setter?
      // Wait, sensor_t only exposed get_color_data() and get_color().
      // We need a set_color(std::array<float, 3>) or use get_color_data() to
      // copy.

      float *color_ptr = sensors.back().get_color_data();
      color_ptr[0] = color[0];
      color_ptr[1] = color[1];
      color_ptr[2] = color[2];
    }
  }

  return true;
}

} // namespace persistence
} // namespace sensor_mapper
