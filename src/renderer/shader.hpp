#pragma once

#include <glad/glad.h> // Or whatever loader you use, we use GLFW/GL directly usually?
#include <string>
#include <unordered_map>

// build system uses ImGui's loader which is ... let's check.
// map_widget used imgui_impl_opengl3.h which usually includes a loader or we
// need one. The project is linking opengl32/GL. Usually for modern GL we need a
// loader like GLAD or GLEW. Check CMakeLists.txt... it links `glfw`, `cpr`,
// `opengl32`. imgui_impl_opengl3 uses its own loader mechanism if not provided?
// Actually, `imgui_impl_opengl3_loader.h` exists in recent ImGui.
// But safely, we should stick to basic GL 3.0 functions or use the loader if
// available. For now, let's assume standard headers or minimal loader. Wait,
// `map_widget.cpp` includes `<GLFW/glfw3.h>`. Windows requires a loader for
// >GL1.1. Creating a shader requires `glCreateShader` which is >1.1. We should
// check if we have GLAD. CMakeLists doesn't show GLAD. ImGui backend usually
// handles this. `imgui_impl_opengl3.h` defines `IMGL3W_IMPL` or similar? Let's
// use `imgui_impl_opengl3_loader.h` if possible, or just add `glad` to CMake?
// Simplest: The user's project likely relies on ImGui's internal loading or we
// need to add one. Let's try to include `imgui_impl_opengl3.h` and see if it
// exposes GL functions or we need to look closer. Actually, the
// `map_widget.cpp` uses `glGenTextures`... that's GL 1.1? No, 1.1 has it.
// `glCreateShader` is 2.0.
// Let's try to use `imgui_impl_opengl3_loader.h` which is embedded in ImGui's
// backends in newer versions. If not, we might need to add `glad` via
// FetchContent. Checking CMakeLists again... It fetches ImGui 'docking' branch.
// High chance it has the loader. Let's rely on `imgui_impl_opengl3.h` providing
// headers or include the loader explicitly if needed.

#include "imgui_impl_opengl3.h"
// Note: imgui_impl_opengl3 might not export the GL symbols globally if we don't
// enable the loader option. But let's assume we can access them or standard
// system headers. If on Windows, we need `GL/glcorearb.h` or `glad` or `glew`.
// Let's look at `map_widget.cpp` includes: `<GLFW/glfw3.h>`.
// GLFW includes `GL/gl.h`. On Windows `gl.h` is old (1.1).
// We DEFINITELY need a loader for Shaders.
// I'll add a simple GL loader or use `imgui_impl_opengl3_loader.h` (file is
// `backends/imgui_impl_opengl3_loader.h`).

#if defined(IMGUI_IMPL_OPENGL_LOADER_CUSTOM)
// ...
#else
#include "backends/imgui_impl_opengl3_loader.h"
#endif

namespace sensor_mapper {

class shader_t {
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

  bool is_valid() const { return m_renderer_id != 0; }

private:
  unsigned int m_renderer_id;
  std::unordered_map<std::string, int> m_uniform_cache;

  int get_uniform_location(const std::string &name);
  unsigned int compile_shader(unsigned int type, const std::string &source);
};

} // namespace sensor_mapper
