#pragma once

#include <glad/glad.h>
#include <string>
#include <unordered_map>

namespace sensor_mapper
{

class shader_t
{
public:
  shader_t(const std::string &vertex_src, const std::string &fragment_src);
  ~shader_t();

  void bind() const;
  void unbind() const;

  void set_int(const std::string &name, int value);
  void set_float(const std::string &name, float value);
  void set_vec2(const std::string &name, float x, float y);
  void set_vec3(const std::string &name, float x, float y, float z);
  void set_vec4(const std::string &name, float x, float y, float z, float w);

  // Array setters
  void set_float_array(const std::string &name, int count, const float *values);
  void set_vec4_array(const std::string &name, int count, const float *values);

  bool is_valid() const
  {
    return m_renderer_id != 0;
  }

private:
  unsigned int m_renderer_id;
  std::unordered_map<std::string, int> m_uniform_cache;

  int get_uniform_location(const std::string &name);
  unsigned int compile_shader(unsigned int type, const std::string &source);
};

} // namespace sensor_mapper