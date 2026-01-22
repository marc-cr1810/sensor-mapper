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

} // namespace persistence
} // namespace sensor_mapper
