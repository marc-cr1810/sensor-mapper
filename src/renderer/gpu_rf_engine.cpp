#include "gpu_rf_engine.hpp"
#include "../core/geo_math.hpp"
#include "../core/rasterizer_utils.hpp"
#include <glad/glad.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace sensor_mapper
{

static const char *VERTEX_SHADER = R"(
#version 330 core
layout(location = 0) in vec2 a_pos;
layout(location = 1) in vec2 a_texcoord;
out vec2 v_texcoord;
void main() {
    v_texcoord = a_texcoord;
    gl_Position = vec4(a_pos, 0.0, 1.0);
}
)";

gpu_rf_engine_t::gpu_rf_engine_t()
{
  init_gl();
}

gpu_rf_engine_t::~gpu_rf_engine_t()
{
  if (m_fbo)
    glDeleteFramebuffers(1, &m_fbo);
  if (m_result_texture)
    glDeleteTextures(1, &m_result_texture);
  if (m_data_texture)
    glDeleteTextures(1, &m_data_texture);
  if (m_elevation_texture)
    glDeleteTextures(1, &m_elevation_texture);
  if (m_antenna_pattern_texture)
    glDeleteTextures(1, &m_antenna_pattern_texture);
  if (m_quad_vao)
    glDeleteVertexArrays(1, &m_quad_vao);
  if (m_quad_vbo)
    glDeleteBuffers(1, &m_quad_vbo);
}

void gpu_rf_engine_t::init_gl()
{
  // Quad
  float vertices[] = {// Pos        // Tex
                      -1.0f, -1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, 1.0f, 1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f};

  glGenVertexArrays(1, &m_quad_vao);
  glBindVertexArray(m_quad_vao);

  glGenBuffers(1, &m_quad_vbo);
  glBindBuffer(GL_ARRAY_BUFFER, m_quad_vbo);
  glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

  // Attribs
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (void *)(2 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glBindVertexArray(0);

  // Texture for Elevation
  glGenTextures(1, &m_elevation_texture);
  glBindTexture(GL_TEXTURE_2D, m_elevation_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void gpu_rf_engine_t::resize_fbo(int width, int height)
{
  if (m_fbo_width == width && m_fbo_height == height && m_fbo != 0)
    return;

  m_fbo_width = width;
  m_fbo_height = height;

  if (m_fbo == 0)
    glGenFramebuffers(1, &m_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);

  // Attachment 0: Color (Visualization)
  if (m_result_texture == 0)
    glGenTextures(1, &m_result_texture);
  glBindTexture(GL_TEXTURE_2D, m_result_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_result_texture, 0);

  // Attachment 1: Data (Signal dBm, R32F)
  if (m_data_texture == 0)
    glGenTextures(1, &m_data_texture);
  glBindTexture(GL_TEXTURE_2D, m_data_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, width, height, 0, GL_RED, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST); // Nearest because it's data
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, m_data_texture, 0);

  // Set Draw Buffers
  GLenum buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glDrawBuffers(2, buffers);

  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::cerr << "GPU RF Engine: Framebuffer not complete!" << std::endl;

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void gpu_rf_engine_t::update_elevation_texture(elevation_service_t *service, const building_service_t *building_service, double min_lat, double max_lat, double min_lon, double max_lon)
{
  // Use lower resolution for elevation texture to reduce memory and sampling cost
  int w = 256;
  int h = 256;

  std::vector<float> data(w * h, 0.0f);

  // 1. Load Terrain
  for (int y = 0; y < h; ++y)
  {
    double t = (double)y / (h - 1);
    double lat = min_lat + t * (max_lat - min_lat);
    for (int x = 0; x < w; ++x)
    {
      double u = (double)x / (w - 1);
      double lon = min_lon + u * (max_lon - min_lon);

      float elev = 0.0f;
      if (service)
        service->get_elevation(lat, lon, elev);

      // Store in non-flipped buffer first
      data[y * w + x] = elev;
    }
  }

  // 2. Rasterize Buildings
  if (building_service)
  {
    auto buildings = building_service->get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
    rasterize_buildings(data, w, h, min_lat, max_lat, min_lon, max_lon, buildings, true); // add_mode = true
  }

  // 3. Flip Y for OpenGL Upload
  std::vector<float> flipped_data(w * h);
  for (int y = 0; y < h; ++y)
  {
    for (int x = 0; x < w; ++x)
    {
      flipped_data[((h - 1) - y) * w + x] = data[y * w + x];
    }
  }

  glBindTexture(GL_TEXTURE_2D, m_elevation_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, w, h, 0, GL_RED, GL_FLOAT, flipped_data.data());
}

auto gpu_rf_engine_t::render(const std::vector<sensor_t> &sensors, elevation_service_t *service, const building_service_t *building_service, double min_lat, double max_lat, double min_lon, double max_lon, float min_signal_dbm)
    -> unsigned int
{

  // Lazy load shader
  if (!m_shader)
  {
    const char *FRAG_SRC = R"(
#version 330 core
precision mediump float;

layout(location = 0) out vec4 FragColor;
layout(location = 1) out float FragData;

in vec2 v_texcoord;

uniform sampler2D u_elevation_tex;
uniform sampler2D u_antenna_pattern_tex;

struct Sensor {
    vec2 pos_uv;      // 0-1
    float range_m;    // Range in meters
    float power_dbm;
    float freq_mhz;
    float azimuth_deg;
    float beamwidth_deg;
    float gain_dbi;
    float mast_height;
    float ground_elev;
    int model;
};

// Max sensors
const int MAX_SENSORS = 32;
uniform int u_sensor_count;
// Arrays
uniform vec4 u_sensor_pos_range[MAX_SENSORS]; // xy=uv, z=range_uv, w=unused
uniform vec4 u_sensor_params_1[MAX_SENSORS];  // x=pwr, y=freq, z=azi, w=beam
uniform vec4 u_sensor_params_2[MAX_SENSORS];  // x=gain, y=mast, z=ground, w=model

uniform vec2 u_bounds_meters; // Width/Height of map in meters
uniform float u_min_signal_dbm; // Minimum signal threshold to display

float get_elevation(vec2 uv) {
    return texture(u_elevation_tex, uv).r;
}

float calculate_fspl(float d_km, float f_mhz) {
    if (d_km <= 0.001) return 0.0;
    return 20.0 * log(d_km)/log(10.0) + 20.0 * log(f_mhz)/log(10.0) + 32.44;
}

float calculate_hata(float d_km, float f_mhz, float h_tx, float h_rx) {
   if (d_km <= 0.001) return 0.0;
   float log_f = log(f_mhz)/log(10.0);
   float log_h = log(max(1.0, h_tx))/log(10.0);
   float log_d = log(d_km)/log(10.0);

   float a_h_rx = (1.1 * log_f - 0.7)*h_rx - (1.56*log_f - 0.8);
   return 69.55 + 26.16*log_f - 13.82*log_h - a_h_rx + (44.9 - 6.55*log_h)*log_d;
}

// Turbo Colormap (Copyright 2019 Google LLC. Apache License 2.0)
float turbo_r(float x) {
    const vec4 kRedVec4 = vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234);
    const vec2 kRedVec2 = vec2(-152.94239396, 59.28637943);
    x = clamp(x, 0.0, 1.0);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z; // x^4, x^5
    return dot(v4, kRedVec4) + dot(v2, kRedVec2);
}
float turbo_g(float x) {
    const vec4 kGreenVec4 = vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333);
    const vec2 kGreenVec2 = vec2(4.27729857, 2.82956604);
    x = clamp(x, 0.0, 1.0);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z;
    return dot(v4, kGreenVec4) + dot(v2, kGreenVec2);
}
float turbo_b(float x) {
    const vec4 kBlueVec4 = vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771);
    const vec2 kBlueVec2 = vec2(-89.90310912, 27.34824973);
    x = clamp(x, 0.0, 1.0);
    vec4 v4 = vec4(1.0, x, x * x, x * x * x);
    vec2 v2 = v4.zw * v4.z;
    return dot(v4, kBlueVec4) + dot(v2, kBlueVec2);
}

void main() {
    float my_elev = get_elevation(v_texcoord);
    float h_rx = 2.0 + my_elev;

    float max_dbm = -200.0;

    for(int i=0; i<MAX_SENSORS; ++i) {
        if (i >= u_sensor_count) break;
        
        if (max_dbm >= -50.0) break;

        vec2 s_uv = u_sensor_pos_range[i].xy;
        float s_range = u_sensor_pos_range[i].z;

        vec2 diff = (v_texcoord - s_uv);
        vec2 diff_m = diff * u_bounds_meters;
        float dist_m = length(diff_m);

        if (dist_m > s_range) continue;

        float tx_pwr = u_sensor_params_1[i].x;
        float freq   = u_sensor_params_1[i].y;
        float azi    = u_sensor_params_1[i].z;
        float beam   = u_sensor_params_1[i].w;
        float gain   = u_sensor_params_2[i].x;
        float mast   = u_sensor_params_2[i].y;
        float ground = u_sensor_params_2[i].z;
        int   model  = int(u_sensor_params_2[i].w);

        float angle_rad = atan(diff_m.y, diff_m.x);
        float angle_deg = degrees(angle_rad);
        angle_deg = 90.0 - angle_deg;
        if (angle_deg < 0.0) angle_deg += 360.0;

        float rel_angle = angle_deg - azi;
        if (rel_angle < 0.0) rel_angle += 360.0;
        if (rel_angle >= 360.0) rel_angle -= 360.0;
        
        float pattern_u = rel_angle / 360.0;
        float pattern_v = (float(i) + 0.5) / 32.0; 
        float pattern_gain = texture(u_antenna_pattern_tex, vec2(pattern_u, pattern_v)).r;
        float pattern_loss = -pattern_gain; 
        
        float d_km = dist_m / 1000.0;
        float quick_loss = 0.0;
        float h_tx = mast + ground;
        if (model == 0) {
            quick_loss = calculate_fspl(d_km, freq);
        } else {
            quick_loss = calculate_hata(d_km, freq, h_tx, 2.0);
        }
        
        float estimated_rx = tx_pwr + gain - quick_loss - pattern_loss - 10.0; 
        if (estimated_rx < u_min_signal_dbm - 20.0) continue; 

        float diffraction_loss = 0.0;
        float dist_km = dist_m / 1000.0;
        float R_eff = 6378137.0 * 1.333333;
        int steps = int(clamp(dist_m / 50.0, 20.0, 100.0));
        float step_increment = 1.0 / float(steps);
        float max_v = -10.0;
        
        for(int s=1; s<steps; ++s) {
            float t = float(s) * step_increment;
            vec2 p_uv = s_uv + diff * t;
            float p_h = texture(u_elevation_tex, p_uv).r;
            float d1 = dist_m * t;
            float d2 = dist_m * (1.0 - t);
            float h_los = h_tx + t * (h_rx - h_tx);
            float bulge = (d1 * d2) / (2.0 * R_eff);
            float h_obs = p_h + bulge;
            float clearance = h_los - h_obs;
            float lambda = 299.79 / freq;
            float v = -clearance * sqrt( (2.0 * (d1+d2)) / (lambda * d1 * d2) );
            if (v > max_v) max_v = v;
        }
        
        if (max_v > -0.7) {
            diffraction_loss = 6.9 + 20.0 * log(sqrt(max_v*max_v + 1.0) + max_v) / log(10.0);
        }

        float rx = tx_pwr + gain - quick_loss - pattern_loss - diffraction_loss;
        max_dbm = max(max_dbm, rx);
    }
    
    // Output Data
    FragData = max_dbm;

    // Output Visualization
    vec4 col = vec4(0.0);
    if (max_dbm > u_min_signal_dbm) {
        float val = (max_dbm - u_min_signal_dbm) / 100.0;
        val = clamp(val, 0.0, 1.0);
        
        float r = turbo_r(val);
        float g = turbo_g(val);
        float b = turbo_b(val);
        float a = 0.4 + val * 0.6;
        
        col = vec4(r, g, b, a);
    }
    FragColor = col;
}
)";
    m_shader = std::make_unique<shader_t>(VERTEX_SHADER, FRAG_SRC);
  }

  // Adaptive resolution
  double area_deg = (max_lat - min_lat) * (max_lon - min_lon);
  int resolution = 512;
  if (area_deg > 1.0)
    resolution = 256;
  else if (area_deg > 0.1)
    resolution = 384;
  else if (area_deg < 0.01)
    resolution = 768;

  resize_fbo(resolution, resolution);
  update_elevation_texture(service, building_service, min_lat, max_lat, min_lon, max_lon);

  m_shader->bind();

  double mid_lat = (min_lat + max_lat) * 0.5;
  double height_m = (max_lat - min_lat) * 111000.0;
  double width_m = (max_lon - min_lon) * 111000.0 * std::cos(mid_lat * 3.14159 / 180.0);
  m_shader->set_vec2("u_bounds_meters", (float)width_m, (float)height_m);
  m_shader->set_float("u_min_signal_dbm", min_signal_dbm);

  std::vector<size_t> sensor_indices(sensors.size());
  for (size_t i = 0; i < sensors.size(); ++i)
    sensor_indices[i] = i;
  std::sort(sensor_indices.begin(), sensor_indices.end(),
            [&sensors](size_t a, size_t b)
            {
              double power_a = sensors[a].get_tx_power_dbm() + sensors[a].get_tx_antenna_gain_dbi();
              double power_b = sensors[b].get_tx_power_dbm() + sensors[b].get_tx_antenna_gain_dbi();
              return power_a > power_b;
            });

  int count = 0;
  std::vector<float> u_pos_range;
  std::vector<float> u_p1;
  std::vector<float> u_p2;

  for (size_t idx : sensor_indices)
  {
    if (count >= 32)
      break;
    const auto &s = sensors[idx];
    float u = (float)((s.get_longitude() - min_lon) / (max_lon - min_lon));
    float v = 1.0f - (float)((s.get_latitude() - min_lat) / (max_lat - min_lat));
    float range_m = (float)s.get_range();
    u_pos_range.push_back(u);
    u_pos_range.push_back(v);
    u_pos_range.push_back(range_m);
    u_pos_range.push_back(0.0f);
    u_p1.push_back((float)s.get_tx_power_dbm());
    u_p1.push_back((float)s.get_frequency_mhz());
    u_p1.push_back((float)s.get_azimuth_deg());
    u_p1.push_back((float)s.get_beamwidth_deg());
    u_p2.push_back((float)s.get_tx_antenna_gain_dbi());
    u_p2.push_back((float)s.get_mast_height());
    u_p2.push_back((float)s.get_ground_elevation());
    u_p2.push_back((float)s.get_propagation_model());
    count++;
  }

  m_shader->set_int("u_sensor_count", count);
  if (count > 0)
  {
    m_shader->set_vec4_array("u_sensor_pos_range", count, u_pos_range.data());
    m_shader->set_vec4_array("u_sensor_params_1", count, u_p1.data());
    m_shader->set_vec4_array("u_sensor_params_2", count, u_p2.data());
  }

  update_antenna_pattern_texture(sensors);

  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, m_elevation_texture);
  m_shader->set_int("u_elevation_tex", 1);

  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, m_antenna_pattern_texture);
  m_shader->set_int("u_antenna_pattern_tex", 2);

  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  glViewport(0, 0, m_fbo_width, m_fbo_height);

  // Clear both targets manually if needed, or just clear buffer bit
  GLenum draw_buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  glDrawBuffers(2, draw_buffers);

  glClearColor(0, 0, 0, 0);
  // Clear Data to -200 (noise)
  float noise[] = {-200.0f, -200.0f, -200.0f, -200.0f};
  glClearBufferfv(GL_COLOR, 1, noise);
  glClear(GL_COLOR_BUFFER_BIT); // Clears Attachment 0 with clear color

  glBindVertexArray(m_quad_vao);
  glDrawArrays(GL_TRIANGLE_FAN, 0, 4);
  glBindVertexArray(0);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  m_shader->unbind();

  return m_result_texture;
}

void gpu_rf_engine_t::update_antenna_pattern_texture(const std::vector<sensor_t> &sensors)
{
  if (m_antenna_pattern_texture == 0)
  {
    glGenTextures(1, &m_antenna_pattern_texture);
    glBindTexture(GL_TEXTURE_2D, m_antenna_pattern_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  }

  std::vector<float> pattern_data(360 * m_max_pattern_sensors, 0.0f);
  int num_sensors = std::min((int)sensors.size(), m_max_pattern_sensors);

  for (int i = 0; i < num_sensors; ++i)
  {
    const auto &sensor = sensors[i];
    for (int angle = 0; angle < 360; ++angle)
    {
      float gain_db = static_cast<float>(sensor.get_antenna_gain(static_cast<double>(angle)));
      pattern_data[i * 360 + angle] = gain_db;
    }
  }

  glBindTexture(GL_TEXTURE_2D, m_antenna_pattern_texture);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, 360, m_max_pattern_sensors, 0, GL_RED, GL_FLOAT, pattern_data.data());
}

auto gpu_rf_engine_t::read_dbm_at(int x, int y) -> float
{
  if (m_fbo == 0)
    return -200.0f;

  glBindFramebuffer(GL_FRAMEBUFFER, m_fbo);
  glReadBuffer(GL_COLOR_ATTACHMENT1);

  float dbm = -200.0f;
  glReadPixels(x, y, 1, 1, GL_RED, GL_FLOAT, &dbm);

  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  return dbm;
}

} // namespace sensor_mapper
