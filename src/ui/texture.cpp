#include "ui/texture.hpp"
#include <iostream>

#include "GLFW/glfw3.h" // For GL types

// STB Image implementation
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

namespace sensor_mapper {

texture_t::texture_t()
    : m_renderer_id(0), m_width(0), m_height(0), m_channels(0) {}

texture_t::~texture_t() {
  if (m_renderer_id) {
    glDeleteTextures(1, &m_renderer_id);
  }
}

auto texture_t::load_from_memory(const unsigned char *data, size_t size)
    -> bool {
  // Flip vertically because OpenGL expects 0,0 at bottom left,
  // BUT map tiles are usually 0,0 top left.
  // Let's not flip for map tiles, usually we want them Top-Down.
  stbi_set_flip_vertically_on_load(0);

  unsigned char *pixel_data = stbi_load_from_memory(
      data, static_cast<int>(size), &m_width, &m_height, &m_channels,
      4 // Force RGBA
  );

  if (!pixel_data) {
    std::cerr << "Failed to load texture from memory" << std::endl;
    return false;
  }

  if (m_renderer_id) {
    glDeleteTextures(1, &m_renderer_id);
  }

  glGenTextures(1, &m_renderer_id);
  glBindTexture(GL_TEXTURE_2D, m_renderer_id);

  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, m_width, m_height, 0, GL_RGBA,
               GL_UNSIGNED_BYTE, pixel_data);

  stbi_image_free(pixel_data);
  return true;
}

auto texture_t::is_valid() const -> bool { return m_renderer_id != 0; }

auto texture_t::get_id() const -> unsigned int { return m_renderer_id; }

auto texture_t::get_width() const -> int { return m_width; }

auto texture_t::get_height() const -> int { return m_height; }

} // namespace sensor_mapper
