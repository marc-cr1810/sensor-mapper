#pragma once

#include "antenna_pattern.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace sensor_mapper {

// Visualization data structures
struct polar_plot_point_t {
  float angle_deg;
  float radius; // Normalized gain (0-1)
  float gain_db;
};

struct polar_plot_data_t {
  std::vector<polar_plot_point_t> points;
  std::string title;
  float min_gain_db = -40.0f;
  float max_gain_db = 0.0f;
  bool show_grid = true;
  bool show_beamwidth = true;
  float beamwidth_deg = 0.0f;
};

struct spherical_point_t {
  float x, y, z;
  float gain_db;
  float azimuth_deg;
  float elevation_deg;
};

struct pattern_3d_mesh_t {
  std::vector<spherical_point_t> vertices;
  std::vector<unsigned int> indices; // Triangle indices
  float scale_factor = 1.0f;
  float min_gain_db = -40.0f;
  float max_gain_db = 0.0f;
};

struct pattern_cut_t {
  std::vector<float> angles;
  std::vector<float> gains_db;
  std::string label;
  float angle_step_deg = 1.0f;
};

// Color mapping for gain visualization
struct gain_color_t {
  float r, g, b, a;
};

// Visualization parameters
struct viz_params_t {
  float min_gain_db = -40.0f;       // Display floor
  float max_gain_db = 0.0f;          // Display ceiling
  bool normalize_to_peak = true;     // Normalize display to peak gain
  bool show_in_linear = false;       // Show linear (not dB)
  bool show_3db_contour = true;      // Highlight 3dB beamwidth
  bool show_side_lobes = true;       // Show side lobes
  int resolution_deg = 1;            // Angular resolution for plots
  float scale_factor = 1.0f;         // Scale for 3D visualization
};

class antenna_pattern_viz_t {
public:
  // Generate polar plot data for 2D azimuth pattern
  static auto generate_polar_plot(const antenna_pattern_t& pattern,
                                  float elevation_deg = 0.0f,
                                  const viz_params_t& params = viz_params_t())
      -> polar_plot_data_t {
    polar_plot_data_t plot;
    plot.title = pattern.name + " @ " + std::to_string(elevation_deg) + "° elevation";
    plot.min_gain_db = params.min_gain_db;
    plot.max_gain_db = params.max_gain_db;
    plot.beamwidth_deg = pattern.horizontal_beamwidth_deg;
    plot.show_beamwidth = params.show_3db_contour;
    
    // Generate points around full circle
    for (int angle = 0; angle < 360; angle += params.resolution_deg) {
      polar_plot_point_t point;
      point.angle_deg = static_cast<float>(angle);
      point.gain_db = pattern.get_gain(static_cast<float>(angle), elevation_deg, true);
      
      // Normalize to display range
      if (params.normalize_to_peak) {
        point.gain_db -= pattern.max_gain_dbi;
      }
      
      // Clamp to display range
      point.gain_db = std::max(point.gain_db, params.min_gain_db);
      
      // Convert to normalized radius (0-1)
      if (params.show_in_linear) {
        // Linear scale
        float linear_gain = std::pow(10.0f, point.gain_db / 10.0f);
        float linear_max = std::pow(10.0f, params.max_gain_db / 10.0f);
        point.radius = linear_gain / linear_max;
      } else {
        // dB scale
        point.radius = (point.gain_db - params.min_gain_db) / 
                      (params.max_gain_db - params.min_gain_db);
      }
      
      point.radius = std::clamp(point.radius, 0.0f, 1.0f);
      
      plot.points.push_back(point);
    }
    
    return plot;
  }
  
  // Generate elevation cut at specific azimuth
  static auto generate_elevation_cut(const antenna_pattern_t& pattern,
                                     float azimuth_deg = 0.0f,
                                     const viz_params_t& params = viz_params_t())
      -> pattern_cut_t {
    pattern_cut_t cut;
    cut.label = "Elevation cut @ " + std::to_string(azimuth_deg) + "° azimuth";
    cut.angle_step_deg = static_cast<float>(params.resolution_deg);
    
    for (int elev = -90; elev <= 90; elev += params.resolution_deg) {
      float gain_db = pattern.get_gain(azimuth_deg, static_cast<float>(elev), true);
      
      if (params.normalize_to_peak) {
        gain_db -= pattern.max_gain_dbi;
      }
      
      cut.angles.push_back(static_cast<float>(elev));
      cut.gains_db.push_back(gain_db);
    }
    
    return cut;
  }
  
  // Generate azimuth cut at specific elevation
  static auto generate_azimuth_cut(const antenna_pattern_t& pattern,
                                   float elevation_deg = 0.0f,
                                   const viz_params_t& params = viz_params_t())
      -> pattern_cut_t {
    pattern_cut_t cut;
    cut.label = "Azimuth cut @ " + std::to_string(elevation_deg) + "° elevation";
    cut.angle_step_deg = static_cast<float>(params.resolution_deg);
    
    for (int azi = 0; azi < 360; azi += params.resolution_deg) {
      float gain_db = pattern.get_gain(static_cast<float>(azi), elevation_deg, true);
      
      if (params.normalize_to_peak) {
        gain_db -= pattern.max_gain_dbi;
      }
      
      cut.angles.push_back(static_cast<float>(azi));
      cut.gains_db.push_back(gain_db);
    }
    
    return cut;
  }
  
  // Generate 3D spherical mesh of radiation pattern
  static auto generate_3d_mesh(const antenna_pattern_t& pattern,
                               const viz_params_t& params = viz_params_t())
      -> pattern_3d_mesh_t {
    pattern_3d_mesh_t mesh;
    mesh.min_gain_db = params.min_gain_db;
    mesh.max_gain_db = params.max_gain_db;
    mesh.scale_factor = params.scale_factor;
    
    // Generate vertices in spherical coordinates
    int elev_steps = 180 / params.resolution_deg + 1;
    int azi_steps = 360 / params.resolution_deg;
    
    for (int elev_idx = 0; elev_idx < elev_steps; ++elev_idx) {
      float elev = -90.0f + elev_idx * params.resolution_deg;
      
      for (int azi_idx = 0; azi_idx < azi_steps; ++azi_idx) {
        float azi = azi_idx * params.resolution_deg;
        
        // Get gain at this direction
        float gain_db = pattern.get_gain(azi, elev, true);
        
        if (params.normalize_to_peak) {
          gain_db -= pattern.max_gain_dbi;
        }
        
        // Clamp to display range
        gain_db = std::max(gain_db, params.min_gain_db);
        
        // Convert gain to radius with minimum base radius for visibility
        float base_radius = 0.2f * params.scale_factor; // Minimum visible radius
        float gain_radius;
        
        if (params.show_in_linear) {
          float linear_gain = std::pow(10.0f, gain_db / 10.0f);
          float linear_max = std::pow(10.0f, params.max_gain_db / 10.0f);
          gain_radius = (linear_gain / linear_max) * 0.8f * params.scale_factor;
        } else {
          float norm_gain = (gain_db - params.min_gain_db) / 
                           (params.max_gain_db - params.min_gain_db);
          norm_gain = std::clamp(norm_gain, 0.0f, 1.0f);
          gain_radius = norm_gain * 0.8f * params.scale_factor;
        }
        
        float radius = base_radius + gain_radius;
        
        // Convert spherical to Cartesian
        // Antenna convention: azimuth from North (Y-axis), elevation from horizon
        float azi_rad = azi * M_PI / 180.0f;
        float elev_rad = elev * M_PI / 180.0f;
        
        spherical_point_t point;
        point.azimuth_deg = azi;
        point.elevation_deg = elev;
        point.gain_db = gain_db;
        
        // X = East, Y = North, Z = Up
        point.x = radius * std::cos(elev_rad) * std::sin(azi_rad);
        point.y = radius * std::cos(elev_rad) * std::cos(azi_rad);
        point.z = radius * std::sin(elev_rad);
        
        mesh.vertices.push_back(point);
      }
    }
    
    // Generate triangle indices
    for (int elev_idx = 0; elev_idx < elev_steps - 1; ++elev_idx) {
      for (int azi_idx = 0; azi_idx < azi_steps; ++azi_idx) {
        int next_azi = (azi_idx + 1) % azi_steps;
        
        // Current row vertices
        unsigned int v0 = elev_idx * azi_steps + azi_idx;
        unsigned int v1 = elev_idx * azi_steps + next_azi;
        
        // Next row vertices
        unsigned int v2 = (elev_idx + 1) * azi_steps + azi_idx;
        unsigned int v3 = (elev_idx + 1) * azi_steps + next_azi;
        
        // First triangle
        mesh.indices.push_back(v0);
        mesh.indices.push_back(v2);
        mesh.indices.push_back(v1);
        
        // Second triangle
        mesh.indices.push_back(v1);
        mesh.indices.push_back(v2);
        mesh.indices.push_back(v3);
      }
    }
    
    return mesh;
  }
  
  // Map gain value to color (rainbow or heatmap)
  static auto gain_to_color(float gain_db, float min_db, float max_db, 
                            bool use_heatmap = true) -> gain_color_t {
    // Normalize to 0-1
    float t = (gain_db - min_db) / (max_db - min_db);
    t = std::clamp(t, 0.0f, 1.0f);
    
    gain_color_t color;
    color.a = 1.0f;
    
    if (use_heatmap) {
      // Blue -> Cyan -> Green -> Yellow -> Red heatmap
      if (t < 0.25f) {
        // Blue to Cyan
        float s = t / 0.25f;
        color.r = 0.0f;
        color.g = s;
        color.b = 1.0f;
      } else if (t < 0.5f) {
        // Cyan to Green
        float s = (t - 0.25f) / 0.25f;
        color.r = 0.0f;
        color.g = 1.0f;
        color.b = 1.0f - s;
      } else if (t < 0.75f) {
        // Green to Yellow
        float s = (t - 0.5f) / 0.25f;
        color.r = s;
        color.g = 1.0f;
        color.b = 0.0f;
      } else {
        // Yellow to Red
        float s = (t - 0.75f) / 0.25f;
        color.r = 1.0f;
        color.g = 1.0f - s;
        color.b = 0.0f;
      }
    } else {
      // Rainbow spectrum
      float h = t * 300.0f; // 0-300 degrees (blue to red)
      color = hsv_to_rgb(h, 1.0f, 1.0f);
    }
    
    return color;
  }
  
  // Calculate 3dB beamwidth contour points
  static auto get_3db_contour(const antenna_pattern_t& pattern,
                              float elevation_deg = 0.0f)
      -> std::vector<float> {
    std::vector<float> contour_angles;
    
    float threshold = pattern.max_gain_dbi - 3.0f;
    bool inside_beam = false;
    float start_angle = 0.0f;
    
    for (int angle = 0; angle < 360; ++angle) {
      float gain = pattern.get_gain(static_cast<float>(angle), elevation_deg, true);
      
      if (gain >= threshold && !inside_beam) {
        // Entering beam
        start_angle = static_cast<float>(angle);
        inside_beam = true;
      } else if (gain < threshold && inside_beam) {
        // Exiting beam
        contour_angles.push_back(start_angle);
        contour_angles.push_back(static_cast<float>(angle));
        inside_beam = false;
      }
    }
    
    return contour_angles;
  }
  
  // Generate comparison plot for multiple patterns
  static auto generate_comparison_plot(
      const std::vector<std::shared_ptr<antenna_pattern_t>>& patterns,
      float elevation_deg = 0.0f,
      const viz_params_t& params = viz_params_t())
      -> std::vector<polar_plot_data_t> {
    std::vector<polar_plot_data_t> plots;
    
    for (const auto& pattern : patterns) {
      if (pattern) {
        plots.push_back(generate_polar_plot(*pattern, elevation_deg, params));
      }
    }
    
    return plots;
  }
  
  // Generate overlay visualization data for map rendering
  struct map_overlay_data_t {
    std::vector<float> vertices; // x, y pairs in lat/lon offset
    std::vector<float> gains;    // Gain at each vertex
    std::vector<float> colors;   // RGBA colors
  };
  
  static auto generate_map_overlay(const antenna_pattern_t& pattern,
                                   double /* center_lat */, double /* center_lon */,
                                   float range_meters,
                                   float azimuth_offset_deg = 0.0f,
                                   const viz_params_t& params = viz_params_t())
      -> map_overlay_data_t {
    map_overlay_data_t overlay;
    
    // Generate fan of triangles showing pattern coverage
    int num_segments = 360 / params.resolution_deg;
    
    // Center point
    overlay.vertices.push_back(0.0f);
    overlay.vertices.push_back(0.0f);
    overlay.gains.push_back(pattern.max_gain_dbi);
    
    auto center_color = gain_to_color(pattern.max_gain_dbi, 
                                      params.min_gain_db, 
                                      params.max_gain_db);
    overlay.colors.push_back(center_color.r);
    overlay.colors.push_back(center_color.g);
    overlay.colors.push_back(center_color.b);
    overlay.colors.push_back(center_color.a);
    
    // Outer points
    for (int i = 0; i <= num_segments; ++i) {
      float angle = (i * params.resolution_deg + azimuth_offset_deg);
      float gain_db = pattern.get_gain(angle, 0.0f, true);
      
      if (params.normalize_to_peak) {
        gain_db -= pattern.max_gain_dbi;
      }
      
      // Convert to normalized radius
      float radius_norm = (gain_db - params.min_gain_db) / 
                         (params.max_gain_db - params.min_gain_db);
      radius_norm = std::clamp(radius_norm, 0.0f, 1.0f);
      
      float radius = radius_norm * range_meters;
      
      // Convert to lat/lon offset (approximate)
      float angle_rad = angle * M_PI / 180.0f;
      float dx = radius * std::sin(angle_rad);
      float dy = radius * std::cos(angle_rad);
      
      overlay.vertices.push_back(dx);
      overlay.vertices.push_back(dy);
      overlay.gains.push_back(gain_db);
      
      auto color = gain_to_color(gain_db, params.min_gain_db, params.max_gain_db);
      overlay.colors.push_back(color.r);
      overlay.colors.push_back(color.g);
      overlay.colors.push_back(color.b);
      overlay.colors.push_back(color.a * 0.6f); // Semi-transparent
    }
    
    return overlay;
  }
  
private:
  // HSV to RGB conversion
  static auto hsv_to_rgb(float h, float s, float v) -> gain_color_t {
    gain_color_t color;
    color.a = 1.0f;
    
    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    
    float r, g, b;
    
    if (h < 60.0f) {
      r = c; g = x; b = 0.0f;
    } else if (h < 120.0f) {
      r = x; g = c; b = 0.0f;
    } else if (h < 180.0f) {
      r = 0.0f; g = c; b = x;
    } else if (h < 240.0f) {
      r = 0.0f; g = x; b = c;
    } else if (h < 300.0f) {
      r = x; g = 0.0f; b = c;
    } else {
      r = c; g = 0.0f; b = x;
    }
    
    color.r = r + m;
    color.g = g + m;
    color.b = b + m;
    
    return color;
  }
};

} // namespace sensor_mapper