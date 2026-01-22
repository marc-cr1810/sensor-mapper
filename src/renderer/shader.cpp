#include "shader.hpp"
#include <iostream>
#include <vector>

namespace sensor_mapper {

shader_t::shader_t(const std::string &vertex_src,
                   const std::string &fragment_src) {
  // Compile
  unsigned int vs = compile_shader(GL_VERTEX_SHADER, vertex_src);
  unsigned int fs = compile_shader(GL_FRAGMENT_SHADER, fragment_src);

  // Link
  m_renderer_id = glCreateProgram();
  glAttachShader(m_renderer_id, vs);
  glAttachShader(m_renderer_id, fs);
  glLinkProgram(m_renderer_id);
  glValidateProgram(m_renderer_id);

  // Check Link Errors (optional but good)
  int result;
  glGetProgramiv(m_renderer_id, GL_LINK_STATUS, &result);
  if (result == GL_FALSE) {
    int length;
    glGetProgramiv(m_renderer_id, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> message(length);
    glGetProgramInfoLog(m_renderer_id, length, &length, message.data());
    std::cerr << "Failed to link shader program!" << std::endl;
    std::cerr << message.data() << std::endl;
    glDeleteProgram(m_renderer_id);
    m_renderer_id = 0;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);
}

shader_t::~shader_t() {
  if (m_renderer_id != 0)
    glDeleteProgram(m_renderer_id);
}

void shader_t::bind() const { glUseProgram(m_renderer_id); }

void shader_t::unbind() const { glUseProgram(0); }

void shader_t::set_int(const std::string &name, int value) {
  glUniform1i(get_uniform_location(name), value);
}

void shader_t::set_float(const std::string &name, float value) {
  glUniform1f(get_uniform_location(name), value);
}

void shader_t::set_vec2(const std::string &name, float x, float y) {
  glUniform2f(get_uniform_location(name), x, y);
}

void shader_t::set_vec3(const std::string &name, float x, float y, float z) {
  glUniform3f(get_uniform_location(name), x, y, z);
}

void shader_t::set_vec4(const std::string &name, float x, float y, float z,
                        float w) {
  glUniform4f(get_uniform_location(name), x, y, z, w);
}

void shader_t::set_float_array(const std::string &name, int count,
                               const float *values) {
  glUniform1fv(get_uniform_location(name), count, values);
}

void shader_t::set_vec4_array(const std::string &name, int count,
                              const float *values) {
  glUniform4fv(get_uniform_location(name), count, values);
}

int shader_t::get_uniform_location(const std::string &name) {
  if (m_uniform_cache.find(name) != m_uniform_cache.end())
    return m_uniform_cache[name];

  int location = glGetUniformLocation(m_renderer_id, name.c_str());
  if (location == -1)
    std::cout << "Warning: uniform '" << name << "' doesn't exist!"
              << std::endl;

  m_uniform_cache[name] = location;
  return location;
}

unsigned int shader_t::compile_shader(unsigned int type,
                                      const std::string &source) {
  unsigned int id = glCreateShader(type);
  const char *src = source.c_str();
  glShaderSource(id, 1, &src, nullptr);
  glCompileShader(id);

  // Error handling
  int result;
  glGetShaderiv(id, GL_COMPILE_STATUS, &result);
  if (result == GL_FALSE) {
    int length;
    glGetShaderiv(id, GL_INFO_LOG_LENGTH, &length);
    std::vector<char> message(length);
    glGetShaderInfoLog(id, length, &length, message.data());
    std::cerr << "Failed to compile "
              << (type == GL_VERTEX_SHADER ? "vertex" : "fragment")
              << " shader!" << std::endl;
    std::cerr << message.data() << std::endl;
    glDeleteShader(id);
    return 0;
  }

  return id;
}

} // namespace sensor_mapper
