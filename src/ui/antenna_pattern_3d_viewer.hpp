#pragma once

#include "../core/antenna_pattern.hpp"
#include "../core/antenna_pattern_viz.hpp"
#include <glad/glad.h>
#include <imgui.h>
#include <memory>
#include <vector>
#include <cmath>

namespace sensor_mapper {

// 3D Camera for orbital viewing
struct orbital_camera_t {
  float distance = 3.0f;
  float azimuth = 45.0f;    // degrees
  float elevation = 30.0f;  // degrees
  float pan_x = 0.0f;
  float pan_y = 0.0f;
  
  // Mouse interaction state
  bool is_rotating = false;
  bool is_panning = false;
  ImVec2 last_mouse_pos;
  
  auto get_view_matrix() const -> std::array<float, 16> {
    std::array<float, 16> mat = {
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1
    };
    
    float az_rad = azimuth * M_PI / 180.0f;
    float el_rad = elevation * M_PI / 180.0f;
    
    // Camera position in spherical coordinates
    float x = distance * std::cos(el_rad) * std::sin(az_rad);
    float y = distance * std::sin(el_rad);
    float z = distance * std::cos(el_rad) * std::cos(az_rad);
    
    // Apply pan offset
    x += pan_x;
    y += pan_y;
    
    // Simple look-at calculation
    float length = std::sqrt(x*x + y*y + z*z);
    if (length > 0.001f) {
      x /= length; y /= length; z /= length;
    }
    
    // Right vector
    float rx = -std::sin(az_rad);
    float rz = std::cos(az_rad);
    
    // Up vector
    float ux = -std::cos(az_rad) * std::sin(el_rad);
    float uy = std::cos(el_rad);
    float uz = std::sin(az_rad) * std::sin(el_rad);
    
    mat[0] = rx;  mat[4] = ux;  mat[8] = x;   mat[12] = 0;
    mat[1] = 0;   mat[5] = uy;  mat[9] = y;   mat[13] = 0;
    mat[2] = rz;  mat[6] = uz;  mat[10] = z;  mat[14] = -distance;
    mat[3] = 0;   mat[7] = 0;   mat[11] = 0;  mat[15] = 1;
    
    return mat;
  }
  
  auto get_projection_matrix(float aspect) const -> std::array<float, 16> {
    std::array<float, 16> mat = {0};
    float fov = 45.0f * M_PI / 180.0f;
    float near = 0.1f;
    float far = 100.0f;
    
    float f = 1.0f / std::tan(fov / 2.0f);
    mat[0] = f / aspect;
    mat[5] = f;
    mat[10] = (far + near) / (near - far);
    mat[11] = -1.0f;
    mat[14] = (2.0f * far * near) / (near - far);
    
    return mat;
  }
  
  void handle_mouse_input(ImVec2 mouse_pos, ImVec2 mouse_delta, bool left_down, bool right_down, float wheel) {
    // Rotation with left mouse
    if (left_down) {
      if (!is_rotating) {
        is_rotating = true;
        last_mouse_pos = mouse_pos;
      } else {
        azimuth += mouse_delta.x * 0.5f;
        elevation += mouse_delta.y * 0.5f;
        elevation = std::clamp(elevation, -89.0f, 89.0f);
      }
    } else {
      is_rotating = false;
    }
    
    // Pan with right mouse
    if (right_down) {
      if (!is_panning) {
        is_panning = true;
        last_mouse_pos = mouse_pos;
      } else {
        pan_x -= mouse_delta.x * 0.01f;
        pan_y += mouse_delta.y * 0.01f;
      }
    } else {
      is_panning = false;
    }
    
    // Zoom with mouse wheel
    if (wheel != 0) {
      distance *= (1.0f - wheel * 0.1f);
      distance = std::clamp(distance, 0.5f, 10.0f);
    }
  }
  
  void reset() {
    distance = 3.0f;
    azimuth = 45.0f;
    elevation = 30.0f;
    pan_x = 0.0f;
    pan_y = 0.0f;
  }
};

// 3D Pattern Viewer with OpenGL rendering
class antenna_pattern_3d_viewer_t {
public:
  antenna_pattern_3d_viewer_t() {
    init_gl_resources();
  }
  
  ~antenna_pattern_3d_viewer_t() {
    cleanup_gl_resources();
  }
  
  void render(const antenna_pattern_t& pattern, const viz_params_t& params, 
              ImVec2 canvas_size) {
    if (canvas_size.x < 100 || canvas_size.y < 100) return;
    
    // Handle window resize
    if (m_fbo_width != (int)canvas_size.x || m_fbo_height != (int)canvas_size.y) {
      resize_fbo((int)canvas_size.x, (int)canvas_size.y);
    }
    
    // Handle mouse input
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
    ImVec2 mouse_pos = io.MousePos;
    ImVec2 mouse_delta = io.MouseDelta;
    bool in_canvas = mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
                     mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y;
    
    if (in_canvas) {
      m_camera.handle_mouse_input(mouse_pos, mouse_delta, 
                                   io.MouseDown[0], io.MouseDown[1], 
                                   io.MouseWheel);
    }
    
    // Generate/update mesh if pattern changed
    update_mesh(pattern, params);
    
    // Render to FBO
    render_to_fbo();
    
    // Display the result
    ImGui::Image((void*)(intptr_t)m_result_texture, canvas_size, 
                 ImVec2(0, 1), ImVec2(1, 0));
    
    // Overlay controls help
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_list->AddText(ImVec2(canvas_pos.x + 10, canvas_pos.y + 10),
                      IM_COL32(255, 255, 255, 200),
                      "Left Mouse: Rotate | Right Mouse: Pan | Wheel: Zoom");
    
    // Reset button
    ImGui::SetCursorScreenPos(ImVec2(canvas_pos.x + canvas_size.x - 80, canvas_pos.y + 10));
    if (ImGui::Button("Reset View")) {
      m_camera.reset();
    }
  }

private:
  orbital_camera_t m_camera;
  
  // OpenGL resources
  GLuint m_fbo = 0;
  GLuint m_result_texture = 0;
  GLuint m_depth_buffer = 0;
  int m_fbo_width = 512;
  int m_fbo_height = 512;
  
  GLuint m_shader_program = 0;
  GLuint m_vao = 0;
  GLuint m_vbo = 0;
  GLuint m_ebo = 0;
  
  std::vector<float> m_vertices;
  std::vector<unsigned int> m_indices;
  bool m_mesh_dirty = true;
  
  void init_gl_resources() {
    // Create shader program
    const char* vertex_shader_src = R"(
      #version 130
      in vec3 a_position;
      in vec3 a_normal;
      in float a_gain;
      
      out vec3 v_normal;
      out float v_gain;
      out vec3 v_position;
      
      uniform mat4 u_projection;
      uniform mat4 u_view;
      
      void main() {
        v_normal = a_normal;
        v_gain = a_gain;
        v_position = a_position;
        gl_Position = u_projection * u_view * vec4(a_position, 1.0);
      }
    )";
    
    const char* fragment_shader_src = R"(
      #version 130
      in vec3 v_normal;
      in float v_gain;
      in vec3 v_position;
      
      out vec4 FragColor;
      
      uniform float u_min_gain;
      uniform float u_max_gain;
      
      vec3 gain_to_color(float gain) {
        float t = (gain - u_min_gain) / (u_max_gain - u_min_gain);
        t = clamp(t, 0.0, 1.0);
        
        // Blue -> Cyan -> Green -> Yellow -> Red heatmap
        if (t < 0.25) {
          float s = t / 0.25;
          return vec3(0.0, s, 1.0);
        } else if (t < 0.5) {
          float s = (t - 0.25) / 0.25;
          return vec3(0.0, 1.0, 1.0 - s);
        } else if (t < 0.75) {
          float s = (t - 0.5) / 0.25;
          return vec3(s, 1.0, 0.0);
        } else {
          float s = (t - 0.75) / 0.25;
          return vec3(1.0, 1.0 - s, 0.0);
        }
      }
      
      void main() {
        vec3 light_dir = normalize(vec3(1.0, 1.0, 1.0));
        float diffuse = max(dot(normalize(v_normal), light_dir), 0.0) * 0.6 + 0.4;
        
        vec3 color = gain_to_color(v_gain);
        FragColor = vec4(color * diffuse, 0.9);
      }
    )";
    
    GLuint vs = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vs, 1, &vertex_shader_src, nullptr);
    glCompileShader(vs);
    
    GLuint fs = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fs, 1, &fragment_shader_src, nullptr);
    glCompileShader(fs);
    
    m_shader_program = glCreateProgram();
    glAttachShader(m_shader_program, vs);
    glAttachShader(m_shader_program, fs);
    glLinkProgram(m_shader_program);
    
    glDeleteShader(vs);
    glDeleteShader(fs);
    
    // Create VAO/VBO/EBO
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
    
    // Create FBO
    glGenFramebuffers(1, &m_fbo);
    glGenTextures(1, &m_result_texture);
    glGenRenderbuffers(1, &m_depth_buffer);
    
    resize_fbo(m_fbo_width, m_fbo_height);
  }
  
  void cleanup_gl_resources() {
    if (m_shader_program) glDeleteProgram(m_shader_program);
    if (m_vao) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo) glDeleteBuffers(1, &m_vbo);
    if (m_ebo) glDeleteBuffers(1, &m_ebo);
    if (m_fbo) glDeleteFramebuffers(1, &m_fbo);
    if (m_result_texture) glDeleteTextures(1, &m_result_texture);
    if (m_depth_buffer) glDeleteRenderbuffers(1, &m_depth_buffer);
  }
  
  void resize_fbo(int width, int height) {
    m_fbo_width = width;
    m_fbo_height = height;
    
    glBindTexture(GL_TEXTURE_2D, m_result_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    
    glBindRenderbuffer(GL_RENDERBUFFER, m_depth_buffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height);
    
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_result_texture, 0);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_depth_buffer);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
  }
  
  void update_mesh(const antenna_pattern_t& pattern, const viz_params_t& params) {
    if (!m_mesh_dirty) return;
    
    // Force better resolution for 3D mesh
    viz_params_t mesh_params = params;
    mesh_params.resolution_deg = 5; // 5 degree resolution for smooth mesh
    mesh_params.scale_factor = 1.0f;
    
    auto mesh = antenna_pattern_viz_t::generate_3d_mesh(pattern, mesh_params);
    
    // Build vertex data: position(3) + normal(3) + gain(1)
    m_vertices.clear();
    m_indices.clear();
    
    // Validate mesh has data
    if (mesh.vertices.empty() || mesh.indices.empty()) {
      m_mesh_dirty = false;
      return;
    }
    
    for (const auto& vertex : mesh.vertices) {
      // Position
      m_vertices.push_back(vertex.x);
      m_vertices.push_back(vertex.z);  // Swap Y and Z for proper orientation
      m_vertices.push_back(-vertex.y);
      
      // Normal (computed from position for sphere-like surface)
      float len = std::sqrt(vertex.x*vertex.x + vertex.y*vertex.y + vertex.z*vertex.z);
      if (len > 0.001f) {
        m_vertices.push_back(vertex.x / len);
        m_vertices.push_back(vertex.z / len);
        m_vertices.push_back(-vertex.y / len);
      } else {
        m_vertices.push_back(0.0f);
        m_vertices.push_back(1.0f);
        m_vertices.push_back(0.0f);
      }
      
      // Gain
      m_vertices.push_back(vertex.gain_db);
    }
    
    m_indices = mesh.indices;
    
    // Upload to GPU
    glBindVertexArray(m_vao);
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(float), 
                 m_vertices.data(), GL_STATIC_DRAW);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int),
                 m_indices.data(), GL_STATIC_DRAW);
    
    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
    
    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
    
    // Gain
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(6 * sizeof(float)));
    
    glBindVertexArray(0);
    
    m_mesh_dirty = false;
  }
  
  void render_to_fbo() {
    glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
    glViewport(0, 0, m_fbo_width, m_fbo_height);
    
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glUseProgram(m_shader_program);
    
    // Set uniforms
    float aspect = (float)m_fbo_width / (float)m_fbo_height;
    auto proj = m_camera.get_projection_matrix(aspect);
    auto view = m_camera.get_view_matrix();
    
    GLint proj_loc = glGetUniformLocation(m_shader_program, "u_projection");
    GLint view_loc = glGetUniformLocation(m_shader_program, "u_view");
    GLint min_gain_loc = glGetUniformLocation(m_shader_program, "u_min_gain");
    GLint max_gain_loc = glGetUniformLocation(m_shader_program, "u_max_gain");
    
    glUniformMatrix4fv(proj_loc, 1, GL_FALSE, proj.data());
    glUniformMatrix4fv(view_loc, 1, GL_FALSE, view.data());
    glUniform1f(min_gain_loc, -40.0f);
    glUniform1f(max_gain_loc, 0.0f);
    
    // Draw mesh
    glBindVertexArray(m_vao);
    glDrawElements(GL_TRIANGLES, m_indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
    
    // Draw coordinate axes
    draw_axes();
    
    glUseProgram(0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDisable(GL_DEPTH_TEST);
  }
  
  void draw_axes() {
    // Simple line rendering for X, Y, Z axes
    // This is a simplified version - you could make it more elaborate
  }
};

} // namespace sensor_mapper