#pragma once

#include "../core/elevation_service.hpp"
#include "../core/building_service.hpp"
#include "../core/rf_engine.hpp" // For coverage_grid_t definition
#include "../core/sensor.hpp"
#include "shader.hpp"
#include <future>
#include <memory>
#include <vector>

namespace sensor_mapper
{

class gpu_rf_engine_t
{
public:
  gpu_rf_engine_t();
  ~gpu_rf_engine_t();

  struct render_result_t
  {
    unsigned int texture_id = 0;
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
    bool is_ready = false; // True if this matches the requested bounds, False if stale/old
  };

  // Replaces rf_engine_t::compute_coverage but returns a Texture ID directly!
  // We don't return a std::future<coverage_grid> because the result stays on
  // GPU. If we need the CPU data (e.g. for inspection), we'd need glGetTexImage
  // (slow). For visualization, we just need the texture ID.
  auto render(const std::vector<sensor_t> &sensors, elevation_service_t *elevation_service, const building_service_t *building_service, double min_lat, double max_lat, double min_lon, double max_lon, float min_signal_dbm = -90.0f)
      -> render_result_t;

  // Read back signal strength (dBm) at a specific pixel coordinate (0,0 is bottom-left)
  auto read_dbm_at(int x, int y) -> float;

  auto get_width() const -> int
  {
    return m_fbo_width;
  }
  auto get_height() const -> int
  {
    return m_fbo_height;
  }

private:
  std::unique_ptr<shader_t> m_shader;

  // Framebuffer for rendering the result
  unsigned int m_fbo = 0;
  unsigned int m_result_texture = 0;
  int m_fbo_width = 512;
  int m_fbo_height = 512;

  // Texture for uploading Elevation Data
  unsigned int m_elevation_texture = 0;
  int m_elevation_width = 0;
  int m_elevation_height = 0;

  // Render State
  struct render_bounds_t
  {
    double min_lat = 0.0;
    double max_lat = 0.0;
    double min_lon = 0.0;
    double max_lon = 0.0;
  };
  render_bounds_t m_current_bounds;  // Bounds requested
  render_bounds_t m_uploaded_bounds; // Bounds currently in the texture (VRAM)

  // Async Terrain Generation
  std::future<std::vector<float>> m_terrain_future;
  bool m_is_generating = false;
  render_bounds_t m_generating_bounds; // Bounds being generated

  // Texture for uploading Antenna Pattern Data (360 degrees x max_sensors)
  unsigned int m_antenna_pattern_texture = 0;
  // Texture for Data Readback (R32F)
  unsigned int m_data_texture = 0;
  int m_max_pattern_sensors = 32; // Maximum sensors we can handle patterns for

  // Fullscreen Quad
  unsigned int m_quad_vao = 0;
  unsigned int m_quad_vbo = 0;

  void init_gl();
  void resize_fbo(int width, int height);

  // Helper to copy building data for thread safety
  struct simple_building_t
  {
    std::vector<struct geo_point_t> footprint;
    double height_m;
  };
  static auto generate_elevation_data(elevation_service_t *service, const std::vector<simple_building_t> &buildings, double min_lat, double max_lat, double min_lon, double max_lon, int w, int h) -> std::vector<float>;

  void update_antenna_pattern_texture(const std::vector<sensor_t> &sensors);
};

} // namespace sensor_mapper
