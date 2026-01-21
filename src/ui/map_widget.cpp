#include "ui/map_widget.hpp"
#include "core/geo_math.hpp"
#include <cmath>
#include <format>
#include <string>
#include <vector>

namespace sensor_mapper {

map_widget_t::map_widget_t()
    : m_center_lat(-33.8688) // Sydney
      ,
      m_center_lon(151.2093), m_zoom(10.0), m_show_rf_gradient(false),
      m_tile_service(std::make_unique<tile_service_t>()) {}

map_widget_t::~map_widget_t() {}

auto map_widget_t::update() -> void { m_tile_service->update(); }

auto map_widget_t::set_center(double lat, double lon) -> void {
  m_center_lat = lat;
  m_center_lon = lon;
}

auto map_widget_t::set_zoom(double zoom) -> void {
  m_zoom = zoom;
  if (m_zoom < 1.0)
    m_zoom = 1.0;
  if (m_zoom > 19.0)
    m_zoom = 19.0;
}

auto map_widget_t::set_map_source(int source_index) -> void {
  if (source_index == 0) {
    m_tile_service->set_source(tile_service_t::tile_source_t::OSM);
  } else {
    m_tile_service->set_source(tile_service_t::tile_source_t::TERRARIUM);
  }
}

auto map_widget_t::get_zoom() const -> double { return m_zoom; }

auto map_widget_t::draw(const std::vector<sensor_t> &sensors,
                        int &selected_index,
                        elevation_service_t &elevation_service,
                        std::function<void(double, double)> on_add_sensor)
    -> void {
  // Update async tasks
  update();

  ImGui::Text("Map Center: %.4f, %.4f | Zoom: %.2f", m_center_lat, m_center_lon,
              m_zoom);

  // Zoom controls (Buttons)
  if (ImGui::Button("Zoom In"))
    set_zoom(m_zoom + 1.0);
  ImGui::SameLine();
  if (ImGui::Button("Zoom Out"))
    if (ImGui::Button("Zoom Out"))
      set_zoom(m_zoom - 1.0);

  ImGui::SameLine();
  static int current_source = 0;
  if (ImGui::Combo("Map Source", &current_source,
                   "OpenStreetMap\0Terrain Heightmap\0")) {
    set_map_source(current_source);
  }

  // Simple canvas
  ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
  ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
  if (canvas_sz.x < 50.0f)
    canvas_sz.x = 50.0f;
  if (canvas_sz.y < 50.0f)
    canvas_sz.y = 50.0f;

  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(
      canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y),
      IM_COL32(20, 20, 20, 255));
  draw_list->AddRect(
      canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y),
      IM_COL32(255, 255, 255, 100));

  // Handle Input
  ImGui::InvisibleButton("map_canvas", canvas_sz);
  const bool is_hovered = ImGui::IsItemHovered();
  const bool is_active = ImGui::IsItemActive();
  ImGuiIO &io = ImGui::GetIO();

  // Current World state
  double center_wx, center_wy;
  geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);

  // Tiles usually based on 256px
  const double tile_size = 256.0;
  double n = std::pow(2.0, m_zoom);
  double world_size_pixels = n * tile_size;

  // Context Menu State
  static double ctx_lat = 0.0;
  static double ctx_lon = 0.0;

  // Right Click to Open Context Menu
  if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
    ImVec2 mouse_pos = io.MousePos;
    double mx = center_wx + (mouse_pos.x - (canvas_p0.x + canvas_sz.x * 0.5f)) /
                                world_size_pixels;
    double my = center_wy + (mouse_pos.y - (canvas_p0.y + canvas_sz.y * 0.5f)) /
                                world_size_pixels;

    geo::world_to_lat_lon(mx, my, ctx_lat, ctx_lon);
    ImGui::OpenPopup("map_context_menu");
  }

  // Mouse Wheel Zoom
  if (is_hovered && io.MouseWheel != 0.0f) {
    // Zoom to mouse cursor strategy
    ImVec2 mouse_pos_screen = io.MousePos;
    ImVec2 mouse_offset_from_center =
        ImVec2(mouse_pos_screen.x - (canvas_p0.x + canvas_sz.x * 0.5f),
               mouse_pos_screen.y - (canvas_p0.y + canvas_sz.y * 0.5f));

    double mouse_wx_before =
        center_wx + (mouse_offset_from_center.x / world_size_pixels);
    double mouse_wy_before =
        center_wy + (mouse_offset_from_center.y / world_size_pixels);

    set_zoom(m_zoom + io.MouseWheel * 0.5);

    double new_n = std::pow(2.0, m_zoom);
    double new_world_size_pixels = new_n * tile_size;

    double new_center_wx =
        mouse_wx_before - (mouse_offset_from_center.x / new_world_size_pixels);
    double new_center_wy =
        mouse_wy_before - (mouse_offset_from_center.y / new_world_size_pixels);

    geo::world_to_lat_lon(new_center_wx, new_center_wy, m_center_lat,
                          m_center_lon);

    // Update locals
    n = new_n;
    world_size_pixels = new_world_size_pixels;
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  }

  // Panning
  if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left)) {
    ImVec2 delta = io.MouseDelta;

    double dx = -delta.x / world_size_pixels;
    double dy = -delta.y / world_size_pixels;

    double new_center_wx = center_wx + dx;
    double new_center_wy = center_wy + dy;

    // Wrap X
    if (new_center_wx > 1.0)
      new_center_wx -= 1.0;
    if (new_center_wx < 0.0)
      new_center_wx += 1.0;

    // Clamp Y (0.0 to 1.0)
    if (new_center_wy > 0.0 && new_center_wy < 1.0) {
      geo::world_to_lat_lon(new_center_wx, new_center_wy, m_center_lat,
                            m_center_lon);
    }

    // Refresh
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  }

  // Clip Rendering
  draw_list->PushClipRect(
      canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y),
      true);

  // Screen center
  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f,
                                canvas_p0.y + canvas_sz.y * 0.5f);

  // Determine range of visible tiles
  int tile_zoom = static_cast<int>(std::floor(m_zoom));
  double render_n = std::pow(2.0, tile_zoom);

  double view_half_w = (canvas_sz.x * 0.5f) / world_size_pixels;
  double view_half_h = (canvas_sz.y * 0.5f) / world_size_pixels;

  double min_wx = center_wx - view_half_w;
  double max_wx = center_wx + view_half_w;
  double min_wy = center_wy - view_half_h;
  double max_wy = center_wy + view_half_h;

  int min_tx = static_cast<int>(std::floor(min_wx * render_n));
  int max_tx = static_cast<int>(std::floor(max_wx * render_n));
  int min_ty = static_cast<int>(std::floor(min_wy * render_n));
  int max_ty = static_cast<int>(std::floor(max_wy * render_n));

  // Limit tiles
  if ((max_tx - min_tx) * (max_ty - min_ty) > 100) {
    ImGui::Text("Zoom in to see map");
  } else {
    for (int x = min_tx; x <= max_tx; ++x) {
      for (int y = min_ty; y <= max_ty; ++y) {
        int wrapped_x = x;
        int max_tile = (1 << tile_zoom);
        if (wrapped_x < 0)
          wrapped_x = (wrapped_x % max_tile + max_tile) % max_tile;
        if (wrapped_x >= max_tile)
          wrapped_x = wrapped_x % max_tile;

        if (y < 0 || y >= max_tile)
          continue;

        auto texture = m_tile_service->get_tile(tile_zoom, wrapped_x, y);

        double tile_wx = x / render_n;
        double tile_wy = y / render_n;

        // Calculate Draw Position based on EXACT world pixels (smooth pan/zoom)
        float draw_x = static_cast<float>(
            (tile_wx - center_wx) * world_size_pixels + screen_center.x);
        float draw_y = static_cast<float>(
            (tile_wy - center_wy) * world_size_pixels + screen_center.y);

        // Scale tile size if m_zoom > tile_zoom
        float current_tile_screen_size =
            static_cast<float>(tile_size * std::pow(2.0, m_zoom - tile_zoom));

        ImVec2 p_min = ImVec2(draw_x, draw_y);
        ImVec2 p_max = ImVec2(draw_x + current_tile_screen_size,
                              draw_y + current_tile_screen_size);

        if (texture && texture->is_valid()) {
          draw_list->AddImage((ImTextureID)(intptr_t)texture->get_id(), p_min,
                              p_max);
        } else {
          draw_list->AddRect(p_min, p_max, IM_COL32(255, 255, 255, 30));
        }
      }
    }
  }

  // --- Draw All Sensors ---
  for (size_t idx = 0; idx < sensors.size(); ++idx) {
    const auto &sensor = sensors[idx];
    bool is_selected = (static_cast<int>(idx) == selected_index);

    double s_lat = sensor.get_latitude();
    double s_lon = sensor.get_longitude();
    double range_m = sensor.get_range();

    // Attempt to get elevation for Occlusion Casting
    float sensor_h;
    bool has_elevation = false;

    if (sensor.get_use_auto_elevation()) {
      has_elevation = elevation_service.get_elevation(s_lat, s_lon, sensor_h);
      if (!has_elevation) {
        sensor_h = 0.0f; // Default to sea level if no data yet
      }
    } else {
      sensor_h = static_cast<float>(sensor.get_ground_elevation());
      has_elevation = true;
    }

    // Add mast height
    sensor_h += static_cast<float>(sensor.get_mast_height());

    // --- Caching Logic ---
    // Create a unique hash key for the current state of the sensor view
    std::string current_hash = std::format(
        "{:.6f}_{:.6f}_{:.1f}_{:.1f}_{}", s_lat, s_lon, range_m,
        (double)sensor_h, has_elevation); // sensor_h includes mast + ground

    // Check Cache
    bool cache_valid = false;
    const std::string &s_id = sensor.get_id();
    auto cache_it = m_view_cache.find(s_id);

    // Prepare points vector
    std::vector<ImVec2> points;

    // Note: We need world-space points later for drawing, but the cache should
    // store relative or world? The drawing logic converts lat/lon to world to
    // screen. Screen depends on Zoom/Center. So we should cache the relative
    // world-space offsets or just re-calculate screen pos from fixed Lat/Lon
    // points. The previous raycasting code calculated 'points' directly as
    // Screen Coordinates (px, py). Screen coords change when you Pan/Zoom. So
    // we CANNOT cache screen coords if we Pan/Zoom. We MUST cache the resulting
    // polygon vertices in Lat/Lon (or World Space). Let's modify the loop to
    // generate Lat/Lon points, then a second pass to convert to Screen.

    if (cache_it != m_view_cache.end() &&
        cache_it->second.hash_key == current_hash) {
      // Cache Hit!
      // We still need to convert the cached Screen Points? No, cache should
      // store Lat/Lon. Wait, the previous code pushed `ImVec2(px, py)`
      // directly. I need to refactor the raycast loop to produce a list of
      // Lat/Lon first.

      // Actually, let's store the polygon as a list of "Destination Lat/Lon".
      // Use the existing cache structure: std::vector<ImVec2> where x=lat,
      // y=lon.
      const auto &cached_latlons = cache_it->second.points;
      points.reserve(cached_latlons.size());

      for (const auto &pt : cached_latlons) {
        double p_lat = pt.x;
        double p_lon = pt.y;
        double p_wx, p_wy;
        geo::lat_lon_to_world(p_lat, p_lon, p_wx, p_wy);

        // Simple Wrap check
        if (p_wx - center_wx > 0.5)
          p_wx -= 1.0;
        if (p_wx - center_wx < -0.5)
          p_wx += 1.0;

        float px = static_cast<float>((p_wx - center_wx) * world_size_pixels +
                                      screen_center.x);
        float py = static_cast<float>((p_wy - center_wy) * world_size_pixels +
                                      screen_center.y);
        points.push_back(ImVec2(px, py));
      }
      cache_valid = true;
    }

    if (!cache_valid) {
      // Cache Miss - Recalculate
      const int segments = 360; // 1 degree precision
      std::vector<ImVec2> new_cache_latlons;
      std::vector<double> new_cache_signal_dbm;
      new_cache_latlons.reserve(segments);
      new_cache_signal_dbm.reserve(segments);
      points.reserve(segments);

      // Get RF parameters for propagation calculation
      double tx_power_dbm = sensor.get_tx_power_dbm();
      double frequency_mhz = sensor.get_frequency_mhz();
      double tx_gain_dbi = sensor.get_tx_antenna_gain_dbi();
      double rx_gain_dbi = sensor.get_rx_antenna_gain_dbi();
      double rx_sensitivity_dbm = sensor.get_rx_sensitivity_dbm();

      for (int i = 0; i < segments; ++i) {
        double angle = (360.0 / segments) * i;

        // Start at max range and work inward to find edge of coverage
        double coverage_dist = range_m;

        // Search for coverage boundary using binary search-like approach
        double min_dist = 0.0;
        double max_dist = range_m;
        const int search_steps = 20; // Precision of coverage boundary

        for (int step = 0; step < search_steps; ++step) {
          double test_dist = (min_dist + max_dist) / 2.0;
          if (test_dist < 10.0)
            break; // Minimum distance

          double t_lat, t_lon;
          geo::destination_point(s_lat, s_lon, test_dist, angle, t_lat, t_lon);

          // Calculate Free-Space Path Loss (FSPL)
          // FSPL(dB) = 20*log10(d_km) + 20*log10(f_MHz) + 32.44
          double d_km = test_dist / 1000.0;
          double fspl_db = 20.0 * std::log10(d_km) +
                           20.0 * std::log10(frequency_mhz) + 32.44;

          // Calculate terrain attenuation
          double terrain_loss_db = 0.0;
          if (has_elevation) {
            float target_h;
            if (elevation_service.get_elevation(t_lat, t_lon, target_h)) {
              // Check line-of-sight obstruction
              const double step_size = 50.0;
              double current_dist = 0.0;
              double max_slope = -1000.0;
              double penetration_depth = 0.0;

              while (current_dist < test_dist) {
                current_dist += step_size;
                if (current_dist > test_dist)
                  current_dist = test_dist;

                double check_lat, check_lon;
                geo::destination_point(s_lat, s_lon, current_dist, angle,
                                       check_lat, check_lon);

                float check_h;
                if (elevation_service.get_elevation(check_lat, check_lon,
                                                    check_h)) {
                  // Calculate expected height on LOS path
                  double progress = current_dist / test_dist;
                  double expected_h =
                      sensor_h + (target_h - sensor_h) * progress;

                  // Check for terrain obstruction
                  if (check_h > expected_h) {
                    penetration_depth += (check_h - expected_h);
                  }
                }

                if (current_dist >= test_dist)
                  break;
              }

              // Simplified terrain loss: ~10 dB per meter of penetration
              // (This is a conservative estimate; adjust based on
              // terrain/frequency)
              terrain_loss_db = penetration_depth * 10.0;
            }
          }

          // Calculate received power
          // P_rx = P_tx + G_tx + G_rx - FSPL - Terrain_Loss
          double p_rx_dbm = tx_power_dbm + tx_gain_dbi + rx_gain_dbi - fspl_db -
                            terrain_loss_db;

          // Check if signal is above sensitivity threshold
          if (p_rx_dbm >= rx_sensitivity_dbm) {
            // Signal is receivable, try further out
            min_dist = test_dist;
          } else {
            // Signal too weak, try closer
            max_dist = test_dist;
          }
        }

        coverage_dist = min_dist;

        double p_lat, p_lon;
        geo::destination_point(s_lat, s_lon, coverage_dist, angle, p_lat,
                               p_lon);

        // Calculate signal strength at this edge point for caching
        using geo::PI;
        double lat1_rad = s_lat * PI / 180.0;
        double lat2_rad = p_lat * PI / 180.0;
        double delta_lat = (p_lat - s_lat) * PI / 180.0;
        double delta_lon = (p_lon - s_lon) * PI / 180.0;

        double haversine_a =
            std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) +
            std::cos(lat1_rad) * std::cos(lat2_rad) *
                std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
        double c = 2.0 * std::atan2(std::sqrt(haversine_a),
                                    std::sqrt(1.0 - haversine_a));
        double dist_m = geo::EARTH_RADIUS * c;
        if (dist_m < 1.0)
          dist_m = 1.0;

        double d_km = dist_m / 1000.0;
        double fspl_db =
            20.0 * std::log10(d_km) + 20.0 * std::log10(frequency_mhz) + 32.44;

        // Quick terrain loss (simplified for edge points)
        double terrain_loss_db = 0.0;
        if (has_elevation && coverage_dist < range_m * 0.9) {
          // Only calculate terrain loss for significantly occluded points
          float edge_h;
          if (elevation_service.get_elevation(p_lat, p_lon, edge_h)) {
            if (edge_h > sensor_h + 10.0) {
              terrain_loss_db =
                  (edge_h - sensor_h) * 5.0; // Simplified estimate
            }
          }
        }

        double p_rx_dbm = tx_power_dbm + tx_gain_dbi + rx_gain_dbi - fspl_db -
                          terrain_loss_db;

        // Add to cache (Lat/Lon and Signal Strength)
        new_cache_latlons.push_back(ImVec2((float)p_lat, (float)p_lon));
        new_cache_signal_dbm.push_back(p_rx_dbm);

        // Convert to Screen for drawing now
        double p_wx, p_wy;
        geo::lat_lon_to_world(p_lat, p_lon, p_wx, p_wy);
        if (p_wx - center_wx > 0.5)
          p_wx -= 1.0;
        if (p_wx - center_wx < -0.5)
          p_wx += 1.0;

        float px = static_cast<float>((p_wx - center_wx) * world_size_pixels +
                                      screen_center.x);
        float py = static_cast<float>((p_wy - center_wy) * world_size_pixels +
                                      screen_center.y);

        points.push_back(ImVec2(px, py));
      }

      // Update Cache
      m_view_cache[s_id] = {current_hash, new_cache_latlons,
                            new_cache_signal_dbm};
    }

    // Colors
    auto col = sensor.get_color();
    // Range stays green
    ImU32 fill_col =
        is_selected ? IM_COL32(0, 255, 0, 100) : IM_COL32(0, 200, 0, 50);
    ImU32 border_col =
        is_selected ? IM_COL32(255, 255, 0, 255) : IM_COL32(0, 255, 0, 255);
    float thickness = is_selected ? 3.0f : 2.0f;

    // Calculate center screen pos (needed for triangle fan)
    double c_wx, c_wy;
    geo::lat_lon_to_world(s_lat, s_lon, c_wx, c_wy);
    if (c_wx - center_wx > 0.5)
      c_wx -= 1.0;
    if (c_wx - center_wx < -0.5)
      c_wx += 1.0;

    float cx = static_cast<float>((c_wx - center_wx) * world_size_pixels +
                                  screen_center.x);
    float cy = static_cast<float>((c_wy - center_wy) * world_size_pixels +
                                  screen_center.y);

    // Draw Fill
    if (!points.empty()) {
      ImVec2 center_pt = ImVec2(cx, cy);
      ImU32 center_col =
          is_selected ? IM_COL32(0, 255, 0, 220) : IM_COL32(0, 220, 0, 180);

      if (m_show_rf_gradient && !m_view_cache[s_id].signal_dbm.empty()) {
        // RF Gradient Mode: Use cached signal strength
        auto dbm_to_color = [](double p_rx_dbm) -> ImU32 {
          int r, g, b, a;
          if (p_rx_dbm >= -60.0) {
            r = 0;
            g = 255;
            b = 0;
            a = 200;
          } else if (p_rx_dbm >= -80.0) {
            float t = (p_rx_dbm + 80.0) / 20.0;
            r = static_cast<int>((1.0f - t) * 255);
            g = 255;
            b = 0;
            a = 180;
          } else if (p_rx_dbm >= -100.0) {
            float t = (p_rx_dbm + 100.0) / 20.0;
            r = 255;
            g = static_cast<int>(t * 255);
            b = 0;
            a = 120;
          } else {
            float t = std::max(0.0, (p_rx_dbm + 120.0) / 20.0);
            r = 255;
            g = 0;
            b = 0;
            a = static_cast<int>(t * 80);
          }
          return IM_COL32(r, g, b, a);
        };

        const ImVec2 uv = ImGui::GetFontTexUvWhitePixel();
        auto &cached_signal = m_view_cache[s_id].signal_dbm;

        for (size_t i = 0; i < points.size(); ++i) {
          ImU32 edge_col1 = dbm_to_color(cached_signal[i]);
          ImU32 edge_col2 =
              dbm_to_color(cached_signal[(i + 1) % points.size()]);

          draw_list->PrimReserve(3, 3);
          draw_list->PrimWriteVtx(center_pt, uv, center_col);
          draw_list->PrimWriteVtx(points[i], uv, edge_col1);
          draw_list->PrimWriteVtx(points[(i + 1) % points.size()], uv,
                                  edge_col2);
          draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 3));
          draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 2));
          draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 1));
        }
      } else {
        // Solid Fill Mode (default)
        for (size_t i = 0; i < points.size(); ++i) {
          draw_list->AddTriangleFilled(
              center_pt, points[i], points[(i + 1) % points.size()], fill_col);
        }
      }
    }

    // Draw Border
    draw_list->AddPolyline(points.data(), (int)points.size(), border_col, true,
                           thickness);

    // Use solid color for marker
    ImU32 marker_col =
        IM_COL32(static_cast<int>(col[0] * 255), static_cast<int>(col[1] * 255),
                 static_cast<int>(col[2] * 255), 255);
    draw_list->AddCircleFilled(ImVec2(cx, cy), 5.0f, marker_col);
    if (is_selected) {
      draw_list->AddCircle(ImVec2(cx, cy), 8.0f, IM_COL32(255, 255, 0, 255), 0,
                           2.0f);
    }

    // Hit Test for Selection
    // Check distance from cursor to center marker
    if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
      float dx = io.MousePos.x - cx;
      float dy = io.MousePos.y - cy;
      // 10 pixel radius threshold (squared is 100)
      if (dx * dx + dy * dy < 100.0f) {
        selected_index = static_cast<int>(idx);
      }
    }
  }

  // Draw Context Menu if Open
  if (ImGui::BeginPopup("map_context_menu")) {
    ImGui::Text("Location: %.4f, %.4f", ctx_lat, ctx_lon);
    ImGui::Separator();
    if (ImGui::Selectable("Add Sensor Here")) {
      if (on_add_sensor) {
        on_add_sensor(ctx_lat, ctx_lon);
      }
    }
    ImGui::EndPopup();
  }

  draw_list->PopClipRect();
}

} // namespace sensor_mapper
