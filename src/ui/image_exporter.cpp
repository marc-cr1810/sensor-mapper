#include "image_exporter.hpp"
#include <vector>
#include <iostream>
#include <algorithm>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <glad/glad.h>

namespace sensor_mapper
{

auto image_exporter_t::save_texture_to_png(unsigned int texture_id, int width, int height, const std::string &path) -> bool
{
  if (texture_id == 0 || width <= 0 || height <= 0)
    return false;

  std::vector<unsigned char> pixels(width * height * 4);

  glBindTexture(GL_TEXTURE_2D, texture_id);
  glGetTexImage(GL_TEXTURE_2D, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  glBindTexture(GL_TEXTURE_2D, 0);

  // Flip Y because OpenGL is bottom-left, PNG is top-left
  std::vector<unsigned char> flipped(width * height * 4);
  int stride = width * 4;
  for (int y = 0; y < height; ++y)
  {
    unsigned char *src_row = pixels.data() + y * stride;
    unsigned char *dst_row = flipped.data() + (height - 1 - y) * stride;
    std::copy(src_row, src_row + stride, dst_row);
  }

  // Use stbi_write_png with alpha
  int result = stbi_write_png(path.c_str(), width, height, 4, flipped.data(), width * 4);
  return result != 0;
}

} // namespace sensor_mapper
