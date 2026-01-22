#pragma once

#include <string>
#include <vector>

namespace sensor_mapper
{

class texture_t
{
public:
  texture_t();
  ~texture_t();

  // Load from raw image data (png/jpg)
  auto load_from_memory(const unsigned char *data, size_t size) -> bool;

  // Check if valid
  auto is_valid() const -> bool;

  // Get OpenGL ID
  auto get_id() const -> unsigned int; // returns GLuint
  auto get_width() const -> int;
  auto get_height() const -> int;

private:
  unsigned int m_renderer_id;
  int m_width;
  int m_height;
  int m_channels;
};

} // namespace sensor_mapper
