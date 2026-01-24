#pragma once
#include <string>

namespace sensor_mapper
{
class image_exporter_t
{
public:
  static auto save_texture_to_png(unsigned int texture_id, int width, int height, const std::string &path) -> bool;
};
} // namespace sensor_mapper
