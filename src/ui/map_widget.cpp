#include "map_widget.hpp"
#include "image_exporter.hpp"
#include "../core/building_service.hpp"
#include "../core/elevation_service.hpp"
#include "../core/geo_math.hpp"
#include "../core/sensor.hpp"
#include "../core/tdoa_solver.hpp"
#include "../core/tile_service.hpp"
#include "../core/log_parser.hpp"
#include "imgui.h"

#include "imgui_impl_opengl3.h"
#include "../renderer/gpu_rf_engine.hpp" // GPU Engine
#include "../core/rf_models.hpp"         // RF Models
#include <random>

#include <algorithm>
#include <cmath>
#include <format>
#include <string>
#include <vector>

// On Windows with GLFW, usually just including glfw is enough or glad
#include <GLFW/glfw3.h>

namespace sensor_mapper
{

// --- Rendering Helpers ---
static void sanitize_points(std::vector<ImVec2> &points)
{
  if (points.empty())
    return;

  // Remove duplicate consecutive points
  std::vector<ImVec2> clean_points;
  for (const auto &p : points)
  {
    if (clean_points.empty())
    {
      clean_points.push_back(p);
      continue;
    }
    float dx = p.x - clean_points.back().x;
    float dy = p.y - clean_points.back().y;
    if (std::abs(dx) > 0.01f || std::abs(dy) > 0.01f)
    {
      clean_points.push_back(p);
    }
  }

  // Check closing point again
  if (clean_points.size() > 2)
  {
    float dx = clean_points.back().x - clean_points.front().x;
    float dy = clean_points.back().y - clean_points.front().y;
    if (std::abs(dx) < 0.01f && std::abs(dy) < 0.01f)
      clean_points.pop_back();
  }

  // Remove Collinear Points
  if (clean_points.size() > 2)
  {
    std::vector<ImVec2> final_points;
    for (size_t i = 0; i < clean_points.size(); ++i)
    {
      size_t prev = (i + clean_points.size() - 1) % clean_points.size();
      size_t next = (i + 1) % clean_points.size();
      ImVec2 p1 = clean_points[prev];
      ImVec2 p2 = clean_points[i];
      ImVec2 p3 = clean_points[next];

      // Cross product for collinearity (area of triangle)
      float area = (p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x);
      if (std::abs(area) > 0.001f)
      {
        final_points.push_back(p2);
      }
    }
    clean_points = std::move(final_points);
  }
  points = std::move(clean_points);
}

static bool is_point_in_triangle(ImVec2 p, ImVec2 a, ImVec2 b, ImVec2 c)
{
  auto cross_product = [](ImVec2 a, ImVec2 b, ImVec2 c) { return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x); };
  bool s1 = cross_product(a, b, p) > 0.0f;
  bool s2 = cross_product(b, c, p) > 0.0f;
  bool s3 = cross_product(c, a, p) > 0.0f;
  return (s1 == s2) && (s2 == s3);
}

static std::vector<int> triangulate_polygon(const std::vector<ImVec2> &clean_points)
{
  if (clean_points.size() < 3)
    return {};

  // 1. Determine Winding Order
  double area_sum = 0;
  for (size_t i = 0; i < clean_points.size(); i++)
  {
    size_t j = (i + 1) % clean_points.size();
    area_sum += (double)clean_points[i].x * clean_points[j].y;
    area_sum -= (double)clean_points[j].x * clean_points[i].y;
  }
  bool is_ccw = (area_sum > 0);

  std::vector<int> indices(clean_points.size());
  for (int i = 0; i < (int)indices.size(); ++i)
    indices[i] = i;

  std::vector<int> result;
  int n = static_cast<int>(indices.size());
  int count = 2 * n;
  for (int v = n - 1; n > 2;)
  {
    if (count-- <= 0)
      break;

    int u = v;
    if (u >= n)
      u = 0;
    v = u + 1;
    if (v >= n)
      v = 0;
    int w = v + 1;
    if (w >= n)
      w = 0;

    auto is_ear = [&](int u, int v, int w, int n_curr, const std::vector<int> &idx_list, const std::vector<ImVec2> &pts, bool ccw)
    {
      ImVec2 a = pts[static_cast<size_t>(idx_list[static_cast<size_t>(u)])];
      ImVec2 b = pts[static_cast<size_t>(idx_list[static_cast<size_t>(v)])];
      ImVec2 c = pts[static_cast<size_t>(idx_list[static_cast<size_t>(w)])];

      float cp = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
      if (ccw)
      {
        if (cp <= 0.0001f)
          return false;
      }
      else
      {
        if (cp >= -0.0001f)
          return false;
      }

      for (int p = 0; p < n_curr; p++)
      {
        if (p == u || p == v || p == w)
          continue;
        if (is_point_in_triangle(pts[static_cast<size_t>(idx_list[static_cast<size_t>(p)])], a, b, c))
          return false;
      }
      return true;
    };

    if (is_ear(u, v, w, n, indices, clean_points, is_ccw))
    {
      result.push_back(indices[static_cast<size_t>(u)]);
      result.push_back(indices[static_cast<size_t>(v)]);
      result.push_back(indices[static_cast<size_t>(w)]);
      indices.erase(indices.begin() + v);
      n--;
      count = 2 * n;
    }
  }
  return result;
}

map_widget_t::map_widget_t() : m_center_lat(-33.8688), m_center_lon(151.2093), m_zoom(14.0), m_show_rf_gradient(false), m_show_heatmap_overlay(true), m_show_raster_visual(false)
{
  m_tile_service = std::make_unique<tile_service_t>();
  m_building_service = std::make_unique<building_service_t>();
  m_rf_engine = std::make_unique<gpu_rf_engine_t>();
  m_tdoa_solver = std::make_unique<tdoa_solver_t>();

  // Texture managed by gpu_rf_engine now, or we just hold the ID it returns.
  // m_heatmap_texture_id initialized to 0.
}

map_widget_t::~map_widget_t()
{
  if (m_heatmap_texture_id)
  {
    glDeleteTextures(1, &m_heatmap_texture_id);
  }
}

auto map_widget_t::update() -> void
{
  m_tile_service->update();
  if (m_building_service->update())
  {
    m_heatmap_dirty = true; // Refresh RF calculation when new buildings are loaded
  }

  // GPU engine is synchronous, no future check needed.
}

auto map_widget_t::set_center(double lat, double lon) -> void
{
  m_center_lat = lat;
  m_center_lon = lon;
}

auto map_widget_t::set_zoom(double zoom) -> void
{
  m_zoom = zoom;
  if (m_zoom < 1.0)
    m_zoom = 1.0;
  if (m_zoom > 19.0)
    m_zoom = 19.0;
}

auto map_widget_t::set_map_source(int source_index) -> void
{
  if (source_index == 0)
  {
    m_tile_service->set_source(tile_service_t::tile_source_t::OSM);
  }
  else
  {
    m_tile_service->set_source(tile_service_t::tile_source_t::SATELLITE);
  }
}

auto map_widget_t::get_map_source() const -> int
{
  auto source = m_tile_service->get_source();
  if (source == tile_service_t::tile_source_t::OSM)
    return 0;
  if (source == tile_service_t::tile_source_t::TERRARIUM)
    return 1;
  return 2;
}

auto map_widget_t::get_zoom() const -> double
{
  return m_zoom;
}

auto map_widget_t::get_building_at_location(double lat, double lon) const -> double
{
  if (!m_building_service)
    return 0.0;

  auto building = m_building_service->get_building_at(lat, lon);
  return building ? building->height_m : 0.0;
}

auto map_widget_t::get_buildings_in_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> std::vector<const building_t *>
{
  if (!m_building_service)
    return {};

  return m_building_service->get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
}

auto map_widget_t::fetch_buildings_near(double lat, double lon) -> void
{
  if (!m_building_service)
    return;

  // Fetch a small area around the point (approx +/- 0.005 degrees, ~500m)
  double delta = 0.005;
  m_building_service->fetch_buildings(lat - delta, lat + delta, lon - delta, lon + delta);
}

auto map_widget_t::fetch_buildings_in_area(double min_lat, double max_lat, double min_lon, double max_lon) -> void
{
  if (!m_building_service)
    return;

  m_building_service->fetch_buildings(min_lat, max_lat, min_lon, max_lon);
}

auto map_widget_t::draw(std::vector<sensor_t> &sensors, std::set<int> &selected_indices, elevation_service_t &elevation_service, std::function<void(double, double)> on_add_sensor) -> void
{
  // Update async tasks
  update();

  // Calculate cursor altitude

  // Update altitude periodically or every frame?
  // Every frame is cheap if elevation service caches well or is fast.
  // We'll trust elevation_service is reasonably fast (it checks cache).
  float elev_h = 0.0f;
  if (elevation_service.get_elevation(m_mouse_lat, m_mouse_lon, elev_h))
  {
    m_cursor_alt = static_cast<double>(elev_h);
  }
  else
  {
    m_cursor_alt = 0.0; // or last known
  }

  // Simple canvas
  ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
  ImVec2 canvas_sz_raw = ImGui::GetContentRegionAvail();
  ImVec2 canvas_sz(static_cast<float>(canvas_sz_raw.x < 50.0f ? 50.0f : canvas_sz_raw.x), static_cast<float>(canvas_sz_raw.y < 50.0f ? 50.0f : canvas_sz_raw.y));

  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(20, 20, 20, 255));
  draw_list->AddRect(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(255, 255, 255, 100));

  // Render Drone Icon if playing or has path
  // We use the TDOA test point as the "Drone" position if playback is active
  if (m_playback_state.is_playing && m_playback_state.current_time_sec < m_playback_state.duration_sec)
  {
    // Convert current pos to screen
    double drone_lat = m_test_point_lat; // Sync'd in update_playback
    double drone_lon = m_test_point_lon; // Sync'd in update_playback

    // Or calculate fresh if needed, but test point is fine.
    // Actually let's use the exact interpolated pos if we want smoothness independent of test point update rate?
    // update_playback runs every frame usually.

    ImVec2 drone_pos = lat_lon_to_screen(drone_lat, drone_lon, canvas_p0, canvas_sz);

    draw_list->AddCircleFilled(drone_pos, 10.0f, IM_COL32(255, 50, 50, 255));
    draw_list->AddCircle(drone_pos, 12.0f, IM_COL32(255, 255, 255, 255), 0, 2.0f);

    // Label
    char buf[32];
    snprintf(buf, sizeof(buf), "DRONE (%.1fs)", m_playback_state.current_time_sec);
    draw_list->AddText(ImVec2(drone_pos.x + 15, drone_pos.y - 15), IM_COL32(255, 255, 255, 255), buf);
  }

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

  // Screen center
  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

  // --- Calculate Mouse Lat/Lon Early ---
  ImVec2 mouse_pos_screen_current = ImGui::GetMousePos();
  // Check if mouse is inside canvas (approx)
  if (ImGui::IsWindowHovered() || m_dragging_sensor_index != -1) // Allow dragging outside slightly?
  {
    double mouse_offset_x = mouse_pos_screen_current.x - screen_center.x;
    double mouse_offset_y = mouse_pos_screen_current.y - screen_center.y;
    double mouse_wx = center_wx + (mouse_offset_x / world_size_pixels);
    double mouse_wy = center_wy + (mouse_offset_y / world_size_pixels);

    // Wrap X
    if (mouse_wx < 0.0)
      mouse_wx = std::fmod(mouse_wx, 1.0) + 1.0;
    if (mouse_wx > 1.0)
      mouse_wx = std::fmod(mouse_wx, 1.0);

    // Clamp Y
    if (mouse_wy > 0.0 && mouse_wy < 1.0)
    {
      geo::world_to_lat_lon(mouse_wx, mouse_wy, m_mouse_lat, m_mouse_lon);
    }
  }

  // --- Handle Sensor Dragging ---
  if (m_dragging_sensor_index != -1)
  {
    if (ImGui::IsMouseDown(ImGuiMouseButton_Left))
    {
      if (m_dragging_sensor_index < static_cast<int>(sensors.size()))
      {
        // Update position
        // Optional: Snap to grid or building could go here
        // For now, smooth drag
        // Constrain Lat/Lon?
        auto &s = sensors[m_dragging_sensor_index];
        s.set_latitude(m_mouse_lat);
        s.set_longitude(m_mouse_lon);

        s.set_latitude(m_mouse_lat);
        s.set_longitude(m_mouse_lon);

        // Update elevation if auto-elevation is enabled (or just always for convenience?)
        // Better to respect the flag, but if dragging we probably want it to follow terrain.
        // Let's assume yes if flag is set. AppUI sets it to true on creation.
        if (s.get_use_auto_elevation())
        {
          float h = 0.0f;
          if (elevation_service.get_elevation(m_mouse_lat, m_mouse_lon, h))
            s.set_ground_elevation(h);
        }

        invalidate_rf_heatmap();
      }
    }
    else
    {
      m_dragging_sensor_index = -1;
    }
  }

  // --- Handle Polygon Drawing ---
  if (m_is_drawing_polygon && is_hovered)
  {
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
      // Check for closing the polygon (click near the first point)
      bool closed = false;
      if (!m_target_polygon.empty())
      {
        ImVec2 first_p = lat_lon_to_screen(m_target_polygon.front().first, m_target_polygon.front().second, canvas_p0, canvas_sz);
        ImVec2 mouse_p = ImGui::GetMousePos();
        float dist_sq = (first_p.x - mouse_p.x) * (first_p.x - mouse_p.x) + (first_p.y - mouse_p.y) * (first_p.y - mouse_p.y);

        if (m_target_polygon.size() >= 3 && dist_sq < 15.0f * 15.0f) // 15px radius
        {
          m_is_drawing_polygon = false; // Closed!
          closed = true;
        }
      }

      if (!closed)
      {
        m_target_polygon.push_back({m_mouse_lat, m_mouse_lon});
      }
    }
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
    {
      m_is_drawing_polygon = false;
    }
  }

  // Right Click to Open Context Menu (Disable if drawing polygon)
  if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right) && !m_is_drawing_polygon)
  {
    ImVec2 mouse_pos = io.MousePos;
    double mx = center_wx + (mouse_pos.x - (canvas_p0.x + canvas_sz.x * 0.5f)) / world_size_pixels;
    double my = center_wy + (mouse_pos.y - (canvas_p0.y + canvas_sz.y * 0.5f)) / world_size_pixels;

    geo::world_to_lat_lon(mx, my, m_ctx_lat, m_ctx_lon);
    ImGui::OpenPopup("map_context_menu");
  }

  // Mouse Wheel Zoom
  if (is_hovered && io.MouseWheel != 0.0f)
  {
    // Zoom to mouse cursor strategy
    ImVec2 mouse_pos_screen = io.MousePos;
    ImVec2 mouse_offset_from_center = ImVec2(mouse_pos_screen.x - (canvas_p0.x + canvas_sz.x * 0.5f), mouse_pos_screen.y - (canvas_p0.y + canvas_sz.y * 0.5f));

    double mouse_wx_before = center_wx + (mouse_offset_from_center.x / world_size_pixels);
    double mouse_wy_before = center_wy + (mouse_offset_from_center.y / world_size_pixels);

    set_zoom(m_zoom + io.MouseWheel * 0.5);

    double new_n = std::pow(2.0, m_zoom);
    double new_world_size_pixels = new_n * tile_size;

    double new_center_wx = mouse_wx_before - (mouse_offset_from_center.x / new_world_size_pixels);
    double new_center_wy = mouse_wy_before - (mouse_offset_from_center.y / new_world_size_pixels);

    geo::world_to_lat_lon(new_center_wx, new_center_wy, m_center_lat, m_center_lon);

    // Update locals
    n = new_n;
    world_size_pixels = new_world_size_pixels;
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  }

  // Handle Tool State Clicks (Point Selection)
  if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
  {
    // Map mouse to Lat/Lon
    ImVec2 mouse_pos_screen = io.MousePos;
    ImVec2 mouse_offset_from_center = ImVec2(mouse_pos_screen.x - (canvas_p0.x + canvas_sz.x * 0.5f), mouse_pos_screen.y - (canvas_p0.y + canvas_sz.y * 0.5f));
    double mouse_wx_click = center_wx + (mouse_offset_from_center.x / world_size_pixels);
    double mouse_wy_click = center_wy + (mouse_offset_from_center.y / world_size_pixels);
    // Wrap
    if (mouse_wx_click > 1.0)
      mouse_wx_click -= 1.0;
    if (mouse_wx_click < 0.0)
      mouse_wx_click += 1.0;

    double click_lat, click_lon;
    if (mouse_wy_click > 0.0 && mouse_wy_click < 1.0)
    {
      geo::world_to_lat_lon(mouse_wx_click, mouse_wy_click, click_lat, click_lon);

      if (m_tool_state == tool_state_t::PathProfile_SelectA)
      {
        m_profile_a = {click_lat, click_lon};
        m_has_profile_a = true;
        m_tool_state = tool_state_t::PathProfile_SelectB; // Auto-advance?
      }
      else if (m_tool_state == tool_state_t::PathProfile_SelectB)
      {
        m_profile_b = {click_lat, click_lon};
        m_has_profile_b = true;
        m_tool_state = tool_state_t::Navigate; // Done
        m_show_profile_window = true;
      }
      else if (m_tool_state == tool_state_t::DronePath_Draw)
      {
        bool closed = false;
        // Check for closing the loop (click near first point)
        if (!m_drone_path.empty())
        {
          ImVec2 first_p = lat_lon_to_screen(m_drone_path.front().first, m_drone_path.front().second, canvas_p0, canvas_sz);
          ImVec2 mouse_p = ImGui::GetMousePos();
          float dist_sq = (first_p.x - mouse_p.x) * (first_p.x - mouse_p.x) + (first_p.y - mouse_p.y) * (first_p.y - mouse_p.y);
          if (m_drone_path.size() >= 3 && dist_sq < 15.0f * 15.0f)
          {
            // Close the loop
            m_drone_path.push_back(m_drone_path.front());
            closed = true;
            // End drawing?
            // Maybe keep drawing? No, closing usually implies finishing.
            m_tool_state = tool_state_t::Navigate;
          }
        }

        if (!closed)
        {
          m_drone_path.push_back({click_lat, click_lon});
        }

        // Invalidate results when path changes
        m_drone_path_results.clear();
      }
    }
  }

  // Panning
  if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left) && m_dragging_sensor_index == -1 && !m_is_drawing_polygon)
  {
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
    if (new_center_wy > 0.0 && new_center_wy < 1.0)
    {
      geo::world_to_lat_lon(new_center_wx, new_center_wy, m_center_lat, m_center_lon);
    }

    // Refresh
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  }

  // Clip Rendering
  draw_list->PushClipRect(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), true);

  // Screen center (already calculated)
  // ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

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
  if ((max_tx - min_tx) * (max_ty - min_ty) > 100)
  {
    ImGui::Text("Zoom in to see map");
  }
  else
  {
    // Building fetching removed from draw loop to prevent API spam
    // TODO: Implement on-demand building fetch when placing sensors or
    // calculating RF

    for (int x = min_tx; x <= max_tx; ++x)
    {
      for (int y = min_ty; y <= max_ty; ++y)
      {
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
        float draw_x = static_cast<float>((tile_wx - center_wx) * world_size_pixels + screen_center.x);
        float draw_y = static_cast<float>((tile_wy - center_wy) * world_size_pixels + screen_center.y);

        // Scale tile size if m_zoom > tile_zoom
        float current_tile_screen_size = static_cast<float>(tile_size * std::pow(2.0, m_zoom - tile_zoom));

        ImVec2 p_min = ImVec2(draw_x, draw_y);
        ImVec2 p_max = ImVec2(draw_x + current_tile_screen_size, draw_y + current_tile_screen_size);

        if (texture && texture->is_valid())
        {
          draw_list->AddImage((ImTextureID)(intptr_t)texture->get_id(), p_min, p_max);
        }
        else
        {
          // Progressive tile loading: try parent tiles at lower zoom levels
          bool found_fallback = false;
          for (int fallback_zoom = tile_zoom - 1; fallback_zoom >= 0; --fallback_zoom)
          {
            // Calculate parent tile coordinates
            int zoom_diff = tile_zoom - fallback_zoom;
            int parent_tx = wrapped_x >> zoom_diff;
            int parent_ty = y >> zoom_diff;

            // Get parent tile
            auto parent_texture = m_tile_service->get_tile(fallback_zoom, parent_tx, parent_ty);
            if (parent_texture && parent_texture->is_valid())
            {
              // Calculate which portion of the parent tile to show
              // The parent tile covers 2^zoom_diff tiles in each direction
              int tiles_per_parent = 1 << zoom_diff;
              int sub_x = wrapped_x - (parent_tx << zoom_diff);
              int sub_y = y - (parent_ty << zoom_diff);

              // Calculate UV coordinates for the sub-region
              float uv_scale = 1.0f / tiles_per_parent;
              ImVec2 uv_min(sub_x * uv_scale, sub_y * uv_scale);
              ImVec2 uv_max((sub_x + 1) * uv_scale, (sub_y + 1) * uv_scale);

              // Render the zoomed portion of parent tile
              draw_list->AddImage((ImTextureID)(intptr_t)parent_texture->get_id(), p_min, p_max, uv_min, uv_max);
              found_fallback = true;
              break;
            }
          }

          // If no fallback found, show subtle placeholder
          if (!found_fallback)
          {
            draw_list->AddRectFilled(p_min, p_max, IM_COL32(40, 40, 40, 255));
            draw_list->AddRect(p_min, p_max, IM_COL32(80, 80, 80, 100));
          }
        }
      }
    }
  }

  // --- Draw Buildings ---
  if ((m_show_buildings || m_selection_mode != SelectionMode::None || !m_target_polygon.empty()) && m_building_service)
  {
    // Determine visible area
    double min_lat, max_lat, min_lon, max_lon;
    geo::world_to_lat_lon(min_wx, min_wy, max_lat,
                          min_lon); // Note Y flip
    // wait, world_to_lat_lon(0,0) -> lat, lon.
    // world Y 0 is Top (Max Lat?), Y 1 is Bottom (Min Lat?) -> Mercator.
    // Let's just convert all corners.
    double lat1, lon1, lat2, lon2;
    geo::world_to_lat_lon(min_wx, min_wy, lat1, lon1);
    geo::world_to_lat_lon(max_wx, max_wy, lat2, lon2);
    min_lat = std::min(lat1, lat2);
    max_lat = std::max(lat1, lat2);
    min_lon = std::min(lon1, lon2);
    max_lon = std::max(lon1, lon2);

    // Fetch buildings if not loaded/ensure data is available
    // TODO: Implement cleaner on-demand fetching strategy. Current manual/area-based fetch is sufficient for now.

    auto buildings = m_building_service->get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);

    std::set<std::string> rendered_ids;

    // --- Pre-pass: Identify Topmost Hovered Building ---
    const building_t *topmost_hovered_building = nullptr;

    if (m_selection_mode != SelectionMode::None)
    {
      for (const auto *building : buildings)
      {
        if (!building || building->footprint.empty())
          continue;

        // Visibility Logic (Must match potential visibility in main loop)
        bool is_priority = m_priority_buildings.count(building->id);
        bool is_excluded = m_excluded_buildings.count(building->id);
        bool is_tagged = is_priority || is_excluded;

        if (m_show_target_area)
        {
          // In target area mode (picking), we show everything potentially
        }
        else
        {
          // If mostly hidden, we can skip hit test?
          // For safety, let's just check bounds if relevant or just check everything in 'buildings' list
          // since the list is already spatially queried.
          if (!m_show_buildings && !is_tagged)
            continue;
        }

        // Quick AABB Hit Test on Lat/Lon first?
        // No, need screen coords for precise mouse check.
        // Copy projection logic (Relevant parts)
        float min_x_s = 1e9f, max_x_s = -1e9f, min_y_s = 1e9f, max_y_s = -1e9f;
        bool valid_poly = false;

        // We only need bounds for simple hit test
        for (const auto &pt : building->footprint)
        {
          double wx, wy;
          geo::lat_lon_to_world(pt.lat, pt.lon, wx, wy);
          if (wx - center_wx > 0.5)
            wx -= 1.0;
          if (wx - center_wx < -0.5)
            wx += 1.0;

          float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
          float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);

          if (px < min_x_s)
            min_x_s = px;
          if (px > max_x_s)
            max_x_s = px;
          if (py < min_y_s)
            min_y_s = py;
          if (py > max_y_s)
            max_y_s = py;
          valid_poly = true;
        }

        if (valid_poly)
        {
          if (mouse_pos_screen_current.x >= min_x_s && mouse_pos_screen_current.x <= max_x_s && mouse_pos_screen_current.y >= min_y_s && mouse_pos_screen_current.y <= max_y_s)
          {
            topmost_hovered_building = building;
          }
        }
      }
    }

    for (const auto *building : buildings)
    {
      if (!building || building->footprint.empty())
        continue;

      // Visibility Logic:
      // 1. If the tool is open (m_show_target_area is true):
      //    - If picking (Priority/Exclude), show ALL.
      //    - If NOT picking, only show if Priority or Excluded.
      // 2. If the tool is NOT open, follow global m_show_buildings.
      bool is_priority = m_priority_buildings.count(building->id);
      bool is_excluded = m_excluded_buildings.count(building->id);
      bool is_tagged = is_priority || is_excluded;

      if (m_show_target_area)
      {
        bool is_picking = (m_selection_mode != SelectionMode::None);
        if (!is_picking && !is_tagged)
          continue;
      }
      else
      {
        if (!m_show_buildings && !is_tagged)
          continue;
      }

      // Deduplicate
      if (rendered_ids.count(building->id))
        continue;
      rendered_ids.insert(building->id);

      // Draw footprint
      std::vector<ImVec2> screen_points;
      screen_points.reserve(building->footprint.size());

      for (const auto &pt : building->footprint)
      {
        double wx, wy;
        geo::lat_lon_to_world(pt.lat, pt.lon, wx, wy);

        // Wrap/Clamp adjustments relative to center
        if (wx - center_wx > 0.5)
          wx -= 1.0;
        if (wx - center_wx < -0.5)
          wx += 1.0;

        float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
        float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);
        screen_points.push_back(ImVec2(px, py));
      }

      // Sanitize: OSM often repeats the first point at the end. Remove it for ImGui routines.
      if (screen_points.size() > 1)
      {
        float dx = screen_points.back().x - screen_points.front().x;
        float dy = screen_points.back().y - screen_points.front().y;
        if (std::sqrt(dx * dx + dy * dy) < 0.01f)
        {
          screen_points.pop_back();
        }
      }

      if (screen_points.size() >= 3)
      {
        // Calculate Bounds for Interaction & Culling
        float min_x = screen_points[0].x, max_x = screen_points[0].x;
        float min_y = screen_points[0].y, max_y = screen_points[0].y;
        for (const auto &p : screen_points)
        {
          min_x = std::min(min_x, p.x);
          max_x = std::max(max_x, p.x);
          min_y = std::min(min_y, p.y);
          max_y = std::max(max_y, p.y);
        }

        ImU32 fill_color = IM_COL32(100, 100, 100, 150);
        ImU32 outline_color = IM_COL32(200, 200, 200, 255);

        // Check Hover using Pre-calculated Topmost Building
        bool is_hovered_building = (building == topmost_hovered_building);

        if (is_hovered_building)
        {
          if (m_selection_mode != SelectionMode::None)
          {
            fill_color = IM_COL32(100, 150, 255, 200); // Blue Highlight
            outline_color = IM_COL32(150, 200, 255, 255);
          }
        }

        bool is_priority = m_priority_buildings.count(building->id);
        bool is_excluded = m_excluded_buildings.count(building->id);

        if (is_priority)
        {
          fill_color = is_hovered_building ? IM_COL32(70, 230, 70, 220) : IM_COL32(50, 200, 50, 180); // Green
          outline_color = IM_COL32(100, 255, 100, 255);
        }
        else if (is_excluded)
        {
          fill_color = is_hovered_building ? IM_COL32(230, 70, 70, 220) : IM_COL32(200, 50, 50, 180); // Red
          outline_color = IM_COL32(255, 100, 100, 255);
        }

        // --- DRAW 2D BASE ---
        // Robust fill using triangulation (fixes artifacts on concave shapes)
        std::vector<ImVec2> triang_points = screen_points;
        sanitize_points(triang_points);
        std::vector<int> tri_indices = triangulate_polygon(triang_points);

        if (triang_points.size() >= 3)
        {
          for (size_t i = 0; i + 2 < tri_indices.size(); i += 3)
          {
            draw_list->AddTriangleFilled(triang_points[static_cast<size_t>(tri_indices[i])], triang_points[static_cast<size_t>(tri_indices[i + 1])], triang_points[static_cast<size_t>(tri_indices[i + 2])], fill_color);
          }
        }
        draw_list->AddPolyline(screen_points.data(), screen_points.size(), outline_color, true, 2.0f);

        // Interaction (Selection Mode) - Collection Phase removed, we have the winner.

        // "3D" Extrusion (fake perspective)
        // Show 3D whenever the building is visible (respecting LOD), UNLESS the tool is open.
        if (!m_show_target_area)
        {
          double meters_per_pixel = (40075000.0 * std::cos(m_center_lat * 3.14159 / 180.0)) / world_size_pixels;
          float height_px = static_cast<float>(building->height_m / meters_per_pixel);

          // Culling & LOD Logic (Move UP before point generation)
          float b_width = max_x - min_x;
          float b_height = max_y - min_y;

          // Skip if completely off-screen (with margin)
          if (max_x < canvas_p0.x || min_x > canvas_p0.x + canvas_sz.x || max_y < canvas_p0.y || min_y > canvas_p0.y + canvas_sz.y)
          {
            continue;
          }

          // LOD: Skip tiny buildings or heights
          if (b_width < 3.0f || b_height < 3.0f || height_px < 0.5f)
          {
            continue;
          }

          // LOD: Simplified drawing for small buildings
          if (b_width < 10.0f)
          {
            draw_list->AddRectFilled(ImVec2(min_x, min_y - height_px), ImVec2(max_x, max_y), IM_COL32(100, 100, 100, 150));
            continue;
          }

          // Generate Roof Points from sanitized triang_points for consistency
          std::vector<ImVec2> roof_points;
          roof_points.reserve(triang_points.size());
          for (const auto &p : triang_points)
          {
            roof_points.push_back(ImVec2(p.x, p.y - height_px));
          }

          // Draw Walls first (connect base to roof) using sanitized points
          for (size_t i = 0; i < triang_points.size(); ++i)
          {
            size_t next = (i + 1) % triang_points.size();
            ImVec2 p1 = triang_points[i];
            ImVec2 p2 = triang_points[next];
            ImVec2 r1 = roof_points[i];
            ImVec2 r2 = roof_points[next];

            // Draw quad
            ImVec2 quad[4] = {p1, p2, r2, r1};
            draw_list->AddConvexPolyFilled(quad, 4, IM_COL32(60, 60, 60, 180));
            draw_list->AddLine(p1, r1, IM_COL32(100, 100, 100, 150), 1.0f);
          }

          // Draw Roof AFTER walls for correct layering effect
          if (triang_points.size() >= 3)
          {
            for (size_t i = 0; i + 2 < tri_indices.size(); i += 3)
            {
              draw_list->AddTriangleFilled(roof_points[static_cast<size_t>(tri_indices[i])], roof_points[static_cast<size_t>(tri_indices[i + 1])], roof_points[static_cast<size_t>(tri_indices[i + 2])], fill_color);
            }
          }
          draw_list->AddPolyline(roof_points.data(), roof_points.size(), outline_color, true, 1.5f);
        }
      }
    }

    // Process Interactions on the Topmost Building
    if (topmost_hovered_building)
    {
      const building_t *target_building = topmost_hovered_building;

      // Tooltip
      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(8, 8));
      ImGui::BeginTooltip();
      if (!target_building->name.empty())
      {
        ImGui::Text("%s", target_building->name.c_str());
        ImGui::TextDisabled("%s", target_building->id.c_str());
      }
      else
      {
        ImGui::Text("Building ID: %s", target_building->id.c_str());
      }
      ImGui::EndTooltip();
      ImGui::PopStyleVar();

      // Click to Toggle
      if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
      {
        bool is_priority = m_priority_buildings.count(target_building->id);
        bool is_excluded = m_excluded_buildings.count(target_building->id);

        if (m_selection_mode == SelectionMode::Priority)
        {
          if (is_priority)
            m_priority_buildings.erase(target_building->id);
          else
          {
            m_priority_buildings.insert(target_building->id);
            m_excluded_buildings.erase(target_building->id); // Mutually exclusive
          }
        }
        else if (m_selection_mode == SelectionMode::Exclude)
        {
          if (is_excluded)
            m_excluded_buildings.erase(target_building->id);
          else
          {
            m_excluded_buildings.insert(target_building->id);
            m_priority_buildings.erase(target_building->id);
          }
        }
      }
    }
  }

  // --- Draw Target Polygon ---
  if ((m_show_target_area || m_is_drawing_polygon) && !m_target_polygon.empty())
  {
    std::vector<ImVec2> points;
    for (const auto &p : m_target_polygon)
    {
      points.push_back(lat_lon_to_screen(p.first, p.second, canvas_p0, canvas_sz));
    }

    if (points.size() > 1)
    {
      for (size_t i = 0; i < points.size() - 1; ++i)
      {
        draw_list->AddLine(points[i], points[i + 1], IM_COL32(0, 255, 0, 255), 2.0f);
      }
      if (!m_is_drawing_polygon && points.size() > 2)
      {
        draw_list->AddLine(points.back(), points.front(), IM_COL32(0, 255, 0, 255), 2.0f);
        draw_list->AddConvexPolyFilled(points.data(), points.size(), IM_COL32(0, 255, 0, 40));
      }
    }

    for (const auto &p : points)
    {
      draw_list->AddCircleFilled(p, 4.0f, IM_COL32(0, 255, 0, 255));
    }
  }

  // Draw Line to Mouse if Drawing
  if (m_is_drawing_polygon && !m_target_polygon.empty())
  {
    ImVec2 last_p = lat_lon_to_screen(m_target_polygon.back().first, m_target_polygon.back().second, canvas_p0, canvas_sz);
    ImVec2 mouse_p = ImGui::GetMousePos();
    draw_list->AddLine(last_p, mouse_p, IM_COL32(0, 255, 0, 180), 1.5f);
  }

  // --- Draw Drone Path ---
  // A. Editing State (White/Yellow Lines)
  if (!m_drone_path.empty())
  {
    std::vector<ImVec2> path_points;
    for (const auto &pt : m_drone_path)
    {
      path_points.push_back(lat_lon_to_screen(pt.first, pt.second, canvas_p0, canvas_sz));
    }

    if (path_points.size() > 1)
    {
      // Draw base path line
      ImU32 path_col = IM_COL32(0, 255, 255, 200); // Cyan for better visibility
      // If we have results, we usually hide the raw path or draw it underneath?
      // Let's draw raw path if no results, or faint if results exist.
      if (!m_drone_path_results.empty())
        path_col = IM_COL32(0, 255, 255, 50);

      draw_list->AddPolyline(path_points.data(), path_points.size(), path_col, false, 3.0f); // Thicker line
    }

    // Draw Waypoints
    for (const auto &p : path_points)
    {
      draw_list->AddCircleFilled(p, 3.0f, IM_COL32(255, 255, 255, 200));
    }

    // Draw rubber band line if drawing
    if (m_tool_state == tool_state_t::DronePath_Draw)
    {
      ImVec2 last_p = path_points.back();
      ImVec2 mouse_p = ImGui::GetMousePos();
      draw_list->AddLine(last_p, mouse_p, IM_COL32(255, 255, 0, 150), 1.0f);
    }
  }

  // B. Simulation Results (Colored Segments)
  // B. Simulation Results (Colored Segments)
  if (!m_drone_path_results.empty())
  {
    // Signal > -70 (Green), -70 to -90 (Yellow/Orange), < -90 (Red)
    // Color function depends on mode
    auto get_col = [&](double val, bool blocked) -> ImU32
    {
      if (m_path_metric_mode == 0) // Signal Strength
      {
        if (blocked)
          return IM_COL32(100, 100, 100, 200);
        if (val > -70)
          return IM_COL32(0, 255, 0, 255);
        if (val > -85)
          return IM_COL32(255, 255, 0, 255);
        if (val > -95)
          return IM_COL32(255, 128, 0, 255);
        return IM_COL32(255, 0, 0, 255);
      }
      else // Position Accuracy (Error in meters)
      {
        if (val < 5.0)
          return IM_COL32(0, 255, 0, 255); // < 5m Excellent
        if (val < 15.0)
          return IM_COL32(255, 255, 0, 255); // < 15m Good
        if (val < 50.0)
          return IM_COL32(255, 128, 0, 255); // < 50m Fair
        return IM_COL32(255, 0, 0, 255);     // > 50m Poor
      }
    };

    for (size_t i = 0; i < m_drone_path_results.size() - 1; ++i)
    {
      const auto &res1 = m_drone_path_results[i];
      const auto &res2 = m_drone_path_results[i + 1];
      ImVec2 p1 = lat_lon_to_screen(res1.lat, res1.lon, canvas_p0, canvas_sz);
      ImVec2 p2 = lat_lon_to_screen(res2.lat, res2.lon, canvas_p0, canvas_sz);
      ImU32 c1 = get_col(res1.value, res1.los_blocked);
      draw_list->AddLine(p1, p2, c1, 3.0f);
    }
  }

  // C. Hover Highlight (Dot on Map)
  if (m_highlight_path_index >= 0 && m_highlight_path_index < static_cast<int>(m_drone_path_results.size()))
  {
    const auto &res = m_drone_path_results[m_highlight_path_index];
    ImVec2 p = lat_lon_to_screen(res.lat, res.lon, canvas_p0, canvas_sz);
    draw_list->AddCircleFilled(p, 8.0f, IM_COL32(255, 255, 255, 255));
    draw_list->AddCircle(p, 10.0f, IM_COL32(0, 0, 0, 255), 0, 2.0f);
  }

  // --- Draw All Sensors ---

  if (m_show_composite || m_show_heatmap_overlay)
  {
    // Composite Grid Rendering (Async Texture)
    // 1. Check if we need to trigger update
    // e.g. if camera moved significantly? For now, trigger explicitly or
    // periodical? Let's trigger if future is not valid and not running.
    // Simplifying: Trigger if we don't have a future pending.

    // Bounds
    double min_lat, max_lat, min_lon, max_lon;
    double c_min_wx = center_wx - view_half_w;
    double c_max_wx = center_wx + view_half_w;
    double c_min_wy = center_wy - view_half_h;
    double c_max_wy = center_wy + view_half_h;

    geo::world_to_lat_lon(c_min_wx, c_min_wy, max_lat, min_lon); // Note Y flip
    geo::world_to_lat_lon(c_max_wx, c_max_wy, min_lat, max_lon);
    // Ensure min/max correct
    if (min_lat > max_lat)
      std::swap(min_lat, max_lat);
    if (min_lon > max_lon)
      std::swap(min_lon, max_lon);

    // GPU RF Engine: Render composite coverage
    if (!sensors.empty())
    {
      // Get current time for debouncing
      double current_time = ImGui::GetTime();

      // Check if view or sensors changed significantly
      // Increased threshold to 0.01 degrees (~1.1 km) to reduce re-renders during pan/zoom
      // Also added 100% margin to render area for smoother panning
      double lat_margin = (max_lat - min_lat) * 1.0;
      double lon_margin = (max_lon - min_lon) * 1.0;

      // Snap to grid to prevent "swimming" artifacts during panning
      // Grid step of 0.001 degrees is approx 111 meters
      const double GRID_STEP = 0.001;

      double render_min_lat = std::floor((min_lat - lat_margin) / GRID_STEP) * GRID_STEP;
      double render_max_lat = std::ceil((max_lat + lat_margin) / GRID_STEP) * GRID_STEP;
      double render_min_lon = std::floor((min_lon - lon_margin) / GRID_STEP) * GRID_STEP;
      double render_max_lon = std::ceil((max_lon + lon_margin) / GRID_STEP) * GRID_STEP;

      bool view_changed_significantly = std::abs(render_min_lat - m_prev_min_lat) > 0.01 || std::abs(render_max_lat - m_prev_max_lat) > 0.01 || std::abs(render_min_lon - m_prev_min_lon) > 0.01 ||
                                        std::abs(render_max_lon - m_prev_max_lon) > 0.01 || sensors.size() != m_prev_sensor_count;

      // Debounce: Only render if enough time has passed (0.3 seconds) OR if dirty flag is set
      bool should_render = m_heatmap_dirty || (view_changed_significantly && (current_time - m_last_render_time) > 0.3);

      if (should_render)
      {
        if (m_building_service)
        {
          m_building_service->fetch_buildings(render_min_lat, render_max_lat, render_min_lon, render_max_lon);
        }

        auto result = m_rf_engine->render(sensors, &elevation_service, m_building_service.get(), render_min_lat, render_max_lat, render_min_lon, render_max_lon, m_min_signal_dbm, m_viz_mode, m_target_alt_agl);
        m_heatmap_texture_id = result.texture_id;

        // IMPORTANT: Only acknowledge "clean" state if result is ready/fresh.
        // If result.is_ready is false, it means we are still generating, so we stay dirty or just wait for next frame poll?
        // Actually, we should keep m_heatmap_dirty = true?
        // No, render() async logic will return old data. We display old data.
        // We set m_heatmap_dirty = false, but the engine is working.
        // The engine doesn't have a callback. We need to poll?
        // Current logic: m_heatmap_dirty controls if we CALL render().
        // If we call render() and it returns not-ready, we should probably keep calling it?
        // Or better: render() starts the work. We can just call it next frame?
        // But map_widget::draw calls render only if dirty or moved.
        // If we are "generating", we need to keep polling render() to check for completion.

        // Simplest Async polling:
        if (!result.is_ready)
        {
          // Force update next frame to check for completion
          m_heatmap_dirty = true;
        }
        else
        {
          m_heatmap_dirty = false;
          m_last_render_time = current_time;

          // Only update view tracking when we actually get a result for it
          m_prev_min_lat = render_min_lat;
          m_prev_max_lat = render_max_lat;
          m_prev_min_lon = render_min_lon;
          m_prev_max_lon = render_max_lon;
          m_prev_sensor_count = sensors.size();
        }

        // Cache the actual render bounds for UV calculation (using what was ACTUALLY returned)
        m_cached_render_min_lat = result.min_lat;
        m_cached_render_max_lat = result.max_lat;
        m_cached_render_min_lon = result.min_lon;
        m_cached_render_max_lon = result.max_lon;
      }

      // Draw GPU-rendered texture overlay (if enabled)
      if (m_heatmap_texture_id != 0 && (m_show_heatmap_overlay || m_show_composite))
      {
        // Fix for drift: Draw the texture using its WORLD coordinates, pinned to the map.
        // We calculate the screen position of the texture's corners.
        // This ensures that even if we pan and the texture is stale (async updating), it stays pinned to the correct ground location.
        // ImGui automatically handles clipping if the image is partially/fully off-screen.

        double tex_min_wx, tex_min_wy, tex_max_wx, tex_max_wy;
        // TL of Texture (MaxLat, MinLon)
        geo::lat_lon_to_world(m_cached_render_max_lat, m_cached_render_min_lon, tex_min_wx, tex_min_wy);
        // BR of Texture (MinLat, MaxLon)
        geo::lat_lon_to_world(m_cached_render_min_lat, m_cached_render_max_lon, tex_max_wx, tex_max_wy);

        // Simple Wrap check relative to center
        // If the texture is on the other side of the world wrap seam relative to center, adjust it.
        auto adjust_wrap = [&](double &v)
        {
          double c = center_wx;
          if (v - c > 0.5)
            v -= 1.0;
          if (v - c < -0.5)
            v += 1.0;
        };
        adjust_wrap(tex_min_wx);
        adjust_wrap(tex_max_wx);

        float x0 = static_cast<float>((tex_min_wx - center_wx) * world_size_pixels + screen_center.x);
        float y0 = static_cast<float>((tex_min_wy - center_wy) * world_size_pixels + screen_center.y);
        float x1 = static_cast<float>((tex_max_wx - center_wx) * world_size_pixels + screen_center.x);
        float y1 = static_cast<float>((tex_max_wy - center_wy) * world_size_pixels + screen_center.y);

        draw_list->AddImage((ImTextureID)(intptr_t)m_heatmap_texture_id, ImVec2(x0, y0), ImVec2(x1, y1), ImVec2(0, 0), ImVec2(1, 1), IM_COL32(255, 255, 255, 200));
      }
    }
  }

  // --- Draw Elevation Source Visuals ---
  if (m_show_elevation_sources && m_show_raster_visual)
  {
    const auto &sources = elevation_service.get_sources();
    for (const auto &source : sources)
    {
      int vw, vh;
      const float *vdata = source->get_visual_data(vw, vh);
      if (vdata && vw > 0 && vh > 0)
      {
        // Get or Create Texture
        auto &tex = m_source_textures[source.get()];
        if (tex.id == 0 || tex.w != vw || tex.h != vh)
        {
          if (tex.id == 0)
            glGenTextures(1, &tex.id);
          glBindTexture(GL_TEXTURE_2D, tex.id);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
          glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

          // Convert float grayscale to RGBA8 for better compatibility and transparency
          std::vector<uint32_t> rgba(vw * vh);
          for (int i = 0; i < vw * vh; ++i)
          {
            float v = vdata[i];
            if (v < -0.5f) // Nodata was set to -1.0f in geotiff_source.cpp
            {
              rgba[i] = 0x00000000; // Fully transparent
            }
            else
            {
              uint8_t c = (uint8_t)(v * 255.0f);
              rgba[i] = 0xFF000000 | (c << 16) | (c << 8) | c;
            }
          }

          glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, vw, vh, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
          tex.w = vw;
          tex.h = vh;
        }

        // Draw as image overlay (Precision QUAD rendering)
        double corners_lat[4], corners_lon[4];
        if (source->get_bounds_quad(corners_lat, corners_lon))
        {
          ImVec2 quad_pts[4];
          for (int i = 0; i < 4; ++i)
          {
            double wx, wy;
            geo::lat_lon_to_world(corners_lat[i], corners_lon[i], wx, wy);

            // World Wrap relative to center
            if (wx - center_wx > 0.5)
              wx -= 1.0;
            if (wx - center_wx < -0.5)
              wx += 1.0;

            quad_pts[i].x = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
            quad_pts[i].y = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);
          }

          // Order: TL, TR, BR, BL maps to UVs: (0,0), (1,0), (1,1), (0,1)
          draw_list->AddImageQuad((ImTextureID)(intptr_t)tex.id, quad_pts[0], quad_pts[1], quad_pts[2], quad_pts[3], ImVec2(0, 0), ImVec2(1, 0), ImVec2(1, 1), ImVec2(0, 1), IM_COL32(255, 255, 255, 128));
        }
        else if (double min_lat, max_lat, min_lon, max_lon; source->get_bounds(min_lat, max_lat, min_lon, max_lon))
        {
          // Fallback to axis-aligned if quad not available
          double min_wx, min_wy, max_wx, max_wy;
          geo::lat_lon_to_world(max_lat, min_lon, min_wx, min_wy);
          geo::lat_lon_to_world(min_lat, max_lon, max_wx, max_wy);

          auto wrap = [&](double &v, double center)
          {
            if (v - center > 0.5)
              v -= 1.0;
            if (v - center < -0.5)
              v += 1.0;
          };
          wrap(min_wx, center_wx);
          wrap(max_wx, center_wx);

          float x0 = static_cast<float>((min_wx - center_wx) * world_size_pixels + screen_center.x);
          float y0 = static_cast<float>((min_wy - center_wy) * world_size_pixels + screen_center.y);
          float x1 = static_cast<float>((max_wx - center_wx) * world_size_pixels + screen_center.x);
          float y1 = static_cast<float>((max_wy - center_wy) * world_size_pixels + screen_center.y);

          draw_list->AddImage((ImTextureID)(intptr_t)tex.id, ImVec2(x0, y0), ImVec2(x1, y1), ImVec2(0, 0), ImVec2(1, 1), IM_COL32(255, 255, 255, 128));
        }
      }
    }
  }

  // --- Draw Elevation Source Bounds ---
  if (m_show_elevation_sources)
  {
    const auto &sources = elevation_service.get_sources();
    for (const auto &source : sources)
    {
      double min_lat, max_lat, min_lon, max_lon;
      if (source->get_bounds(min_lat, max_lat, min_lon, max_lon))
      {
        // Convert to World Coords
        double min_wx, min_wy, max_wx, max_wy;
        geo::lat_lon_to_world(max_lat, min_lon, min_wx, min_wy); // Top Left (Lat is max)
        geo::lat_lon_to_world(min_lat, max_lon, max_wx, max_wy); // Bottom Right (Lat is min)

        // Adjust relative to center (World Wrap)
        auto wrap = [&](double &v, double center)
        {
          if (v - center > 0.5)
            v -= 1.0;
          if (v - center < -0.5)
            v += 1.0;
        };
        wrap(min_wx, center_wx);
        wrap(max_wx, center_wx);

        // Note: max_wx might now be LESS than min_wx if we wrapped weirdly, but usually small bounds won't wrap unless on antimeridian.
        // If crossing antimeridian, this simple logic draws a box across the world.
        // For local files, assume they don't cross antimeridian for now.

        float x0 = static_cast<float>((min_wx - center_wx) * world_size_pixels + screen_center.x);
        float y0 = static_cast<float>((min_wy - center_wy) * world_size_pixels + screen_center.y);
        float x1 = static_cast<float>((max_wx - center_wx) * world_size_pixels + screen_center.x);
        float y1 = static_cast<float>((max_wy - center_wy) * world_size_pixels + screen_center.y);

        // Draw rect
        draw_list->AddRect(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(255, 100, 0, 255), 0.0f, 0, 2.0f);
        draw_list->AddText(ImVec2(x0, y0 - 15), IM_COL32(255, 100, 0, 255), source->get_name());
      }
    }
  }

  // --- Draw All Sensors (Markers always, Coverage selectively) ---
  bool sensor_clicked = false;
  {
    // Normal Rendering Loop
    for (size_t idx = 0; idx < sensors.size(); ++idx)
    {
      const auto &sensor = sensors[idx];
      bool is_selected = selected_indices.contains(static_cast<int>(idx));
      double s_lat = sensor.get_latitude();
      double s_lon = sensor.get_longitude();

      // --- Draw Markers (Always visible) ---
      auto color_vals = sensor.get_color();
      ImU32 marker_col = IM_COL32(static_cast<int>(color_vals[0] * 255), static_cast<int>(color_vals[1] * 255), static_cast<int>(color_vals[2] * 255), 255);
      double c_wx_marker, c_wy_marker;
      geo::lat_lon_to_world(s_lat, s_lon, c_wx_marker, c_wy_marker);
      if (c_wx_marker - center_wx > 0.5)
        c_wx_marker -= 1.0;
      if (c_wx_marker - center_wx < -0.5)
        c_wx_marker += 1.0;

      float cx_marker = static_cast<float>((c_wx_marker - center_wx) * world_size_pixels + screen_center.x);
      // ... existing code ...

      float cy_marker = static_cast<float>((c_wy_marker - center_wy) * world_size_pixels + screen_center.y);

      draw_list->AddCircleFilled(ImVec2(cx_marker, cy_marker), 5.0f, marker_col);
      if (is_selected)
      {
        draw_list->AddCircle(ImVec2(cx_marker, cy_marker), 8.0f, IM_COL32(255, 255, 0, 255), 0, 2.0f);
      }

      // Hit Test for Selection
      if (is_hovered)
      {
        float dx = io.MousePos.x - cx_marker;
        float dy = io.MousePos.y - cy_marker;
        if (dx * dx + dy * dy < 100.0f)
        {
          if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
          {
            sensor_clicked = true;
            if (ImGui::GetIO().KeyCtrl)
            {
              if (is_selected)
                selected_indices.erase(static_cast<int>(idx));
              else
                selected_indices.insert(static_cast<int>(idx));
            }
            else if (ImGui::GetIO().KeyShift)
            {
              selected_indices.insert(static_cast<int>(idx));
            }
            else
            {
              if (!is_selected)
              {
                selected_indices.clear();
                selected_indices.insert(static_cast<int>(idx));
              }
            }
            if (!sensor.is_locked())
            {
              m_dragging_sensor_index = static_cast<int>(idx);
            }
          }
        }
      }
    }
  }

  // Deselection: Click on background (and missed all sensors)
  if (!sensor_clicked && is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
  {
    if (ImGui::GetIO().KeyShift)
    {
      // Shift+Click: Place TDOA Test Point
      ImVec2 mouse_pos_screen = io.MousePos;
      ImVec2 mouse_offset_from_center = ImVec2(mouse_pos_screen.x - (canvas_p0.x + canvas_sz.x * 0.5f), mouse_pos_screen.y - (canvas_p0.y + canvas_sz.y * 0.5f));
      double mouse_wx_click = center_wx + (mouse_offset_from_center.x / world_size_pixels);
      double mouse_wy_click = center_wy + (mouse_offset_from_center.y / world_size_pixels);

      // Wrap X
      if (mouse_wx_click > 1.0)
        mouse_wx_click -= 1.0;
      if (mouse_wx_click < 0.0)
        mouse_wx_click += 1.0;

      if (mouse_wy_click > 0.0 && mouse_wy_click < 1.0)
      {
        double clat, clon;
        geo::world_to_lat_lon(mouse_wx_click, mouse_wy_click, clat, clon);
        set_tdoa_test_point(clat, clon);
      }
    }
    // Don't deselect if holding modifier keys either
    else if (!ImGui::GetIO().KeyCtrl)
    {
      selected_indices.clear();
    }
  }

  // Deselection: Click on background (and missed all sensors)
  if (!sensor_clicked && is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
  {
    if (!ImGui::GetIO().KeyCtrl && !ImGui::GetIO().KeyShift)
    {
      if (m_tool_state == tool_state_t::Navigate)
        selected_indices.clear();
    }
  }

  // Draw Context Menu if Open
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 10.0f));
  if (ImGui::BeginPopup("map_context_menu"))
  {
    // Lock/Unlock Logic for Selection
    if (!selected_indices.empty())
    {
      bool all_locked = true;
      for (int idx : selected_indices)
      {
        if (idx >= 0 && idx < (int)sensors.size())
        {
          if (!sensors[idx].is_locked())
            all_locked = false;
        }
      }

      bool new_locked_state = all_locked;
      if (ImGui::MenuItem("Lock Position", nullptr, &new_locked_state))
      {
        for (int idx : selected_indices)
        {
          if (idx >= 0 && idx < (int)sensors.size())
          {
            sensors[idx].set_locked(new_locked_state);
          }
        }
      }
      ImGui::Separator();
    }

    ImGui::Text("Location: %.4f, %.4f", m_ctx_lat, m_ctx_lon);
    ImGui::Separator();
    if (ImGui::Selectable("Add Sensor Here"))
    {
      if (on_add_sensor)
      {
        on_add_sensor(m_ctx_lat, m_ctx_lon);
      }
    }
    if (ImGui::Selectable("Set TDOA Test Point"))
    {
      set_tdoa_test_point(m_ctx_lat, m_ctx_lon);
      // Logic removed: Do not auto-enable hyperbolas
    }
    ImGui::Separator();
    if (ImGui::Selectable("Set Profile Start (A)"))
    {
      m_profile_a = {m_ctx_lat, m_ctx_lon};
      m_has_profile_a = true;
      m_show_profile_window = true;
    }
    if (ImGui::Selectable("Set Profile End (B)"))
    {
      m_profile_b = {m_ctx_lat, m_ctx_lon};
      m_has_profile_b = true;
      m_show_profile_window = true;
    }
    ImGui::EndPopup();
  }
  ImGui::PopStyleVar();

  // Previous Mouse Calc Location (Removed, now done early)

  // Draw overlay info
  std::string info_text = std::format("Cursor: {:.5f}, {:.5f}", m_mouse_lat, m_mouse_lon);

  // GPU Readback
  if (m_show_heatmap_overlay && m_rf_engine)
  {
    // GPU Readback
    int w = m_rf_engine->get_width();
    int h = m_rf_engine->get_height();

    // Calculate UV in the *rendered* texture
    float u = (float)((m_mouse_lon - m_cached_render_min_lon) / (m_cached_render_max_lon - m_cached_render_min_lon));
    float v = 1.0f - (float)((m_mouse_lat - m_cached_render_min_lat) / (m_cached_render_max_lat - m_cached_render_min_lat));

    // Pixel coords (0,0 bottom-left)
    int px = static_cast<int>(u * static_cast<float>(w));
    int py = static_cast<int>(v * static_cast<float>(h));

    // Clamp
    if (px >= 0 && px < w && py >= 0 && py < h)
    {
      float dbm = m_rf_engine->read_dbm_at(px, py);
      if (dbm > -150.0f)
      {
        info_text += std::format(" | Signal: {:.1f} dBm", dbm);
      }
    }
  }

  // Padding and positioning
  float padding = 5.0f;
  ImVec2 text_size = ImGui::CalcTextSize(info_text.c_str());
  ImVec2 overlay_pos = ImVec2(canvas_p0.x + 10, canvas_p0.y + canvas_sz.y - text_size.y - 10);

  // Draw background - High Contrast
  draw_list->AddRectFilled(ImVec2(overlay_pos.x - padding, overlay_pos.y - padding), ImVec2(overlay_pos.x + text_size.x + padding, overlay_pos.y + text_size.y + padding), IM_COL32(30, 30, 30, 220), 4.0f);

  draw_list->AddText(overlay_pos, IM_COL32(255, 255, 255, 255), info_text.c_str());

  // Draw TDOA Layers (Layered below sensors but above map/buildings)
  if (m_show_tdoa_analysis)
  {
    // These might be expensive, so check flags
    if (m_show_gdop_contours)
      render_gdop_contours(sensors, draw_list, canvas_p0, canvas_sz);

    if (m_show_accuracy_heatmap)
      render_accuracy_heatmap(sensors, draw_list, canvas_p0, canvas_sz);

    if (m_show_hyperbolas)
      render_hyperbolas(sensors, draw_list, canvas_p0, canvas_sz);

    render_test_point(sensors, draw_list, canvas_p0, canvas_sz);
  }

  // Draw Profile Markers (Map Space)
  if (m_has_profile_a)
  {
    double wx, wy;
    geo::lat_lon_to_world(m_profile_a.lat, m_profile_a.lon, wx, wy);
    if (wx - center_wx > 0.5)
      wx -= 1.0;
    if (wx - center_wx < -0.5)
      wx += 1.0;
    float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
    float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);

    (void)px;
    (void)py; // Marker pos

    draw_list->AddCircleFilled(ImVec2(px, py), 6.0f, IM_COL32(255, 0, 255, 255));
    draw_list->AddText(ImVec2(px + 8, py - 8), IM_COL32(255, 0, 255, 255), "A");
  }
  if (m_has_profile_b)
  {
    double wx, wy;
    geo::lat_lon_to_world(m_profile_b.lat, m_profile_b.lon, wx, wy);
    if (wx - center_wx > 0.5)
      wx -= 1.0;
    if (wx - center_wx < -0.5)
      wx += 1.0;
    float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
    float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);

    draw_list->AddCircleFilled(ImVec2(px, py), 6.0f, IM_COL32(255, 0, 255, 255));
    draw_list->AddText(ImVec2(px + 8, py - 8), IM_COL32(255, 0, 255, 255), "B");
  }
  if (m_has_profile_a && m_has_profile_b)
  {
    // Draw line
    double wx1, wy1, wx2, wy2;
    geo::lat_lon_to_world(m_profile_a.lat, m_profile_a.lon, wx1, wy1);
    geo::lat_lon_to_world(m_profile_b.lat, m_profile_b.lon, wx2, wy2);

    // Need to handle wrapping for line? simplified for now
    if (wx1 - center_wx > 0.5)
      wx1 -= 1.0;
    if (wx1 - center_wx < -0.5)
      wx1 += 1.0;
    if (wx2 - center_wx > 0.5)
      wx2 -= 1.0;
    if (wx2 - center_wx < -0.5)
      wx2 += 1.0;

    float px1 = static_cast<float>((wx1 - center_wx) * world_size_pixels + screen_center.x);
    float py1 = static_cast<float>((wy1 - center_wy) * world_size_pixels + screen_center.y);
    float px2 = static_cast<float>((wx2 - center_wx) * world_size_pixels + screen_center.x);
    float py2 = static_cast<float>((wy2 - center_wy) * world_size_pixels + screen_center.y);

    draw_list->AddLine(ImVec2(px1, py1), ImVec2(px2, py2), IM_COL32(255, 0, 255, 150), 2.0f);
  }

  // Draw Scale Bar and Compass (Overlay)
  if (m_show_scale_bar)
  {
    render_scale_bar(draw_list, canvas_p0, canvas_sz);
  }
  if (m_show_compass)
  {
    render_compass(draw_list, canvas_p0, canvas_sz);
  }

  draw_list->PopClipRect();

  // Draw Profile Window (ImGui Window)
  draw_path_profile_window(elevation_service);

  // Draw Profile Hover Marker (if active)
  if (m_profile_hover_pos && m_show_profile_window)
  {
    double wx, wy;
    geo::lat_lon_to_world(m_profile_hover_pos->lat, m_profile_hover_pos->lon, wx, wy);
    // Wrap
    if (wx - center_wx > 0.5)
      wx -= 1.0;
    if (wx - center_wx < -0.5)
      wx += 1.0;

    float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
    float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);

    // Draw Crosshair
    draw_list->AddCircleFilled(ImVec2(px, py), 4.0f, IM_COL32(255, 255, 0, 255));
    draw_list->AddLine(ImVec2(px - 10, py), ImVec2(px + 10, py), IM_COL32(255, 255, 0, 200), 2.0f);
    draw_list->AddLine(ImVec2(px, py - 10), ImVec2(px, py + 10), IM_COL32(255, 255, 0, 200), 2.0f);
  }
}

auto map_widget_t::set_tdoa_test_point(double lat, double lon) -> void
{
  m_has_test_point = true;
  m_test_point_lat = lat;
  m_test_point_lon = lon;

  // Force update if needed
  // No cache clearing needed as we removed caching
}

auto map_widget_t::render_hyperbolas(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  if (sensors.size() < 2 || !m_tdoa_solver || !m_has_test_point)
    return;

  // Calculate TDOA for the test point
  auto test_tdoa = m_tdoa_solver->calculate_tdoa(sensors, m_test_point_lat, m_test_point_lon, 0.0);

  // Transform coordinates to screen space lambda
  auto latlon_to_screen = [&](double lat, double lon) -> ImVec2
  {
    double wx, wy;
    geo::lat_lon_to_world(lat, lon, wx, wy);

    // Wrap adjustments relative to center
    double center_wx, center_wy;
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);

    if (wx - center_wx > 0.5)
      wx -= 1.0;
    if (wx - center_wx < -0.5)
      wx += 1.0;

    ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

    double tile_size = 256.0;
    double world_size_pixels = std::pow(2.0, m_zoom) * tile_size;

    float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
    float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);
    return ImVec2(px, py);
  };

  // Draw hyperbola for ALL unique sensor pairs to form a complete graph
  // This is visually more intuitive than just the star topology (relative to sensor 0)
  // although theoretically redundant.
  for (size_t i = 0; i < sensors.size(); ++i)
  {
    for (size_t j = i + 1; j < sensors.size(); ++j)
    {
      // Calculate TDOA for this pair: t(i) - t(j)
      // We know t(k) relative to ref(0) is test_tdoa[k]
      // So t(i) - t(j) = test_tdoa[i] - test_tdoa[j]
      double tdoa_ns = test_tdoa[i] - test_tdoa[j];

      // Sample hyperbola points
      auto points = m_tdoa_solver->sample_hyperbola(sensors[i], sensors[j], tdoa_ns, 0.0, 200);

      if (points.size() < 2)
        continue;

      std::vector<ImVec2> screen_points;
      screen_points.reserve(points.size());

      for (const auto &pt : points)
      {
        screen_points.push_back(latlon_to_screen(pt.first, pt.second));
      }

      // Draw curve
      // Outline
      draw_list->AddPolyline(screen_points.data(), screen_points.size(), IM_COL32(0, 0, 0, 150), false, 5.0f);
      // Center line (Cyan)
      draw_list->AddPolyline(screen_points.data(), screen_points.size(), IM_COL32(0, 255, 255, 255), false, 2.5f);
    }
  }
}

auto map_widget_t::render_gdop_contours(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  if (sensors.size() < 3 || !m_tdoa_solver)
    return;

  if (canvas_sz.x < 1.0f || canvas_sz.y < 1.0f)
    return;

  const float grid_step = 20.0f;
  int cols = static_cast<int>(canvas_sz.x / grid_step) + 1;
  int rows = static_cast<int>(canvas_sz.y / grid_step) + 1;

  double center_wx, center_wy;
  geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  double tile_size = 256.0;
  double world_size_pixels = std::pow(2.0, m_zoom) * tile_size;
  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

  // Calculate sensor bounding box
  double min_lat = 90.0, max_lat = -90.0;
  double min_lon = 180.0, max_lon = -180.0;
  for (const auto &s : sensors)
  {
    min_lat = std::min(min_lat, s.get_latitude());
    max_lat = std::max(max_lat, s.get_latitude());
    min_lon = std::min(min_lon, s.get_longitude());
    max_lon = std::max(max_lon, s.get_longitude());
  }
  double lat_span = max_lat - min_lat;
  double lon_span = max_lon - min_lon;
  double margin_lat = std::max(lat_span * 2.0, 0.5);
  double margin_lon = std::max(lon_span * 2.0, 0.5);
  min_lat -= margin_lat;
  max_lat += margin_lat;
  min_lon -= margin_lon;
  max_lon += margin_lon;

  // Grid of GDOP values
  std::vector<double> gdop_grid((rows + 1) * (cols + 1));

  // Pre-calculate GDOP at vertices
  for (int y = 0; y <= rows; ++y)
  {
    for (int x = 0; x <= cols; ++x)
    {
      float px = canvas_p0.x + x * grid_step;
      float py = canvas_p0.y + y * grid_step;

      double wx = center_wx + (px - screen_center.x) / world_size_pixels;
      double wy = center_wy + (py - screen_center.y) / world_size_pixels;

      if (wx < 0.0)
        wx = std::fmod(wx, 1.0) + 1.0;
      if (wx > 1.0)
        wx = std::fmod(wx, 1.0);

      double lat, lon;
      bool valid = false;
      if (wy >= 0.0 && wy <= 1.0)
      {
        geo::world_to_lat_lon(wx, wy, lat, lon);
        if (lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon)
        {
          // Use 2D GDOP assuming fixed altitude at 0.0 (or terrain if available, but 2D model assumes fixed plane)
          gdop_grid[y * (cols + 1) + x] = m_tdoa_solver->calculate_2d_gdop(sensors, lat, lon, 0.0);
          valid = true;
        }
      }

      if (!valid)
      {
        gdop_grid[y * (cols + 1) + x] = std::numeric_limits<double>::infinity();
      }
    }
  }

  // Marching Squares Thresholds
  struct ContourLevel
  {
    double val;
    ImU32 color;
  };
  std::vector<ContourLevel> levels = {
      {1.5, IM_COL32(0, 0, 255, 200)},    // Blue
      {2.0, IM_COL32(0, 255, 255, 200)},  // Cyan
      {3.0, IM_COL32(0, 255, 0, 200)},    // Green
      {5.0, IM_COL32(255, 255, 0, 200)},  // Yellow
      {10.0, IM_COL32(255, 128, 0, 200)}, // Orange
      {20.0, IM_COL32(255, 0, 0, 200)}    // Red
  };

  for (const auto &level : levels)
  {
    for (int y = 0; y < rows; ++y)
    {
      for (int x = 0; x < cols; ++x)
      {
        // Indices
        int i_tl = y * (cols + 1) + x;
        int i_tr = y * (cols + 1) + (x + 1);
        int i_br = (y + 1) * (cols + 1) + (x + 1);
        int i_bl = (y + 1) * (cols + 1) + x;

        double v_tl = gdop_grid[i_tl];
        double v_tr = gdop_grid[i_tr];
        double v_br = gdop_grid[i_br];
        double v_bl = gdop_grid[i_bl];

        // Check validity
        if (std::isinf(v_tl) || std::isinf(v_tr) || std::isinf(v_br) || std::isinf(v_bl))
          continue;

        // Determine state
        int state = 0;
        if (v_tl >= level.val)
          state |= 8;
        if (v_tr >= level.val)
          state |= 4;
        if (v_br >= level.val)
          state |= 2;
        if (v_bl >= level.val)
          state |= 1;

        if (state == 0 || state == 15)
          continue; // All inside or all outside

        // Interpolation helper
        auto interp = [&](double v1, double v2, float p1, float p2) -> float
        {
          double t = (level.val - v1) / (v2 - v1);
          return p1 + t * (p2 - p1);
        };

        // Coordinates
        float x_l = canvas_p0.x + x * grid_step;
        float x_r = canvas_p0.x + (x + 1) * grid_step;
        float y_t = canvas_p0.y + y * grid_step;
        float y_b = canvas_p0.y + (y + 1) * grid_step;

        ImVec2 p_top = ImVec2(interp(v_tl, v_tr, x_l, x_r), y_t);
        ImVec2 p_right = ImVec2(x_r, interp(v_tr, v_br, y_t, y_b));
        ImVec2 p_bottom = ImVec2(interp(v_bl, v_br, x_l, x_r), y_b);
        ImVec2 p_left = ImVec2(x_l, interp(v_tl, v_bl, y_t, y_b));

        // Marching Squares Lookup Table (Standard)
        // 1: BL -> L-B
        // 2: BR -> B-R
        // 3: BL+BR -> L-R
        // 4: TR -> T-R
        // 5: TR+BL -> L-T, B-R (Ambig, simplistic: connect L-T and B-R)
        // 6: TR+BR -> T-B
        // 7: TR+BR+BL -> L-T
        // 8: TL -> T-L
        // 9: TL+BL -> T-B
        // 10: TL+BR -> T-R, B-L (Ambig)
        // 11: TL+BR+BL -> T-R
        // 12: TL+TR -> L-R
        // 13: TL+TR+BL -> B-R
        // 14: TL+TR+BR -> L-B

        switch (state)
        {
        case 1:
          draw_list->AddLine(p_left, p_bottom, level.color, 2.0f);
          break;
        case 2:
          draw_list->AddLine(p_bottom, p_right, level.color, 2.0f);
          break;
        case 3:
          draw_list->AddLine(p_left, p_right, level.color, 2.0f);
          break;
        case 4:
          draw_list->AddLine(p_top, p_right, level.color, 2.0f);
          break;
        case 5:
          draw_list->AddLine(p_left, p_top, level.color, 2.0f);
          draw_list->AddLine(p_bottom, p_right, level.color, 2.0f);
          break;
        case 6:
          draw_list->AddLine(p_top, p_bottom, level.color, 2.0f);
          break;
        case 7:
          draw_list->AddLine(p_left, p_top, level.color, 2.0f);
          break;
        case 8:
          draw_list->AddLine(p_left, p_top, level.color, 2.0f);
          break;
        case 9:
          draw_list->AddLine(p_top, p_bottom, level.color, 2.0f);
          break;
        case 10:
          draw_list->AddLine(p_top, p_right, level.color, 2.0f);
          draw_list->AddLine(p_bottom, p_left, level.color, 2.0f);
          break;
        case 11:
          draw_list->AddLine(p_top, p_right, level.color, 2.0f);
          break;
        case 12:
          draw_list->AddLine(p_left, p_right, level.color, 2.0f);
          break;
        case 13:
          draw_list->AddLine(p_bottom, p_right, level.color, 2.0f);
          break;
        case 14:
          draw_list->AddLine(p_left, p_bottom, level.color, 2.0f);
          break;
        }
      }
    }
  }
}

auto map_widget_t::render_accuracy_heatmap(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  if (sensors.size() < 3 || !m_tdoa_solver)
    return;

  if (canvas_sz.x < 1.0f || canvas_sz.y < 1.0f)
    return;

  const float grid_step = 20.0f;
  int cols = static_cast<int>(canvas_sz.x / grid_step) + 1;
  int rows = static_cast<int>(canvas_sz.y / grid_step) + 1;

  double center_wx, center_wy;
  geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);
  double tile_size = 256.0;
  double world_size_pixels = std::pow(2.0, m_zoom) * tile_size;
  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

  double jitter = static_cast<double>(m_timing_jitter_ns);

  // Calculate sensor bounding box to constrain visualization
  double min_lat = 90.0, max_lat = -90.0;
  double min_lon = 180.0, max_lon = -180.0;
  for (const auto &s : sensors)
  {
    min_lat = std::min(min_lat, s.get_latitude());
    max_lat = std::max(max_lat, s.get_latitude());
    min_lon = std::min(min_lon, s.get_longitude());
    max_lon = std::max(max_lon, s.get_longitude());
  }

  // Extend bounding box by factor of span, or minimum distance (e.g. 50km)
  double lat_span = max_lat - min_lat;
  double lon_span = max_lon - min_lon;
  double margin_lat = std::max(lat_span * 2.0, 0.5); // At least 0.5 degrees (~55km)
  double margin_lon = std::max(lon_span * 2.0, 0.5);

  min_lat -= margin_lat;
  max_lat += margin_lat;
  min_lon -= margin_lon;
  max_lon += margin_lon;

  auto should_render = [&](double lat, double lon) -> bool { return lat >= min_lat && lat <= max_lat && lon >= min_lon && lon <= max_lon; };

  // Calculate errors at grid points first to allow interpolation
  // We need one extra point row/col for the grid
  std::vector<double> errors_grid;
  errors_grid.resize((rows + 1) * (cols + 1));

  // Pre-calculate errors at vertices
  for (int y = 0; y <= rows; ++y)
  {
    for (int x = 0; x <= cols; ++x)
    {
      float px = canvas_p0.x + x * grid_step;
      float py = canvas_p0.y + y * grid_step;

      double wx = center_wx + (px - screen_center.x) / world_size_pixels;
      double wy = center_wy + (py - screen_center.y) / world_size_pixels;

      // Wrap stuff
      // Ideally we should handle wrapping seamlessly, but for now simple clamp/wrap for the query
      if (wx < 0.0)
        wx = std::fmod(wx, 1.0) + 1.0;
      if (wx > 1.0)
        wx = std::fmod(wx, 1.0);

      double lat, lon;
      if (wy >= 0.0 && wy <= 1.0)
      {
        geo::world_to_lat_lon(wx, wy, lat, lon);
        if (should_render(lat, lon))
        {
          // Use 2D Positioning Error
          errors_grid[y * (cols + 1) + x] = m_tdoa_solver->estimate_2d_positioning_error(sensors, lat, lon, 0.0, jitter);
        }
        else
        {
          errors_grid[y * (cols + 1) + x] = std::numeric_limits<double>::infinity();
        }
      }
      else
      {
        errors_grid[y * (cols + 1) + x] = std::numeric_limits<double>::infinity();
      }
    }
  }

  // Lambda to map error to color
  auto error_to_col = [](double err) -> ImU32
  {
    if (std::isinf(err))
      return IM_COL32(0, 0, 0, 0);

    // Color Scale: Green -> Yellow -> Red -> Transparent
    int r, g, b, a;

    // Clip at high error to avoid drawing huge red blobs everywhere
    if (err > 500.0)
      return IM_COL32(255, 0, 0, 0); // Fade out completely

    int base_alpha = 120;

    if (err < 10.0)
    {
      // Deep Green to Green
      r = 0;
      g = 255;
      b = 0;
      a = base_alpha;
    }
    else if (err < 50.0)
    {
      // Green to Yellow
      float t = (float)(err - 10.0) / 40.0f;
      r = (int)(t * 255.0f);
      g = 255;
      b = 0;
      a = base_alpha;
    }
    else if (err < 200.0)
    {
      // Yellow to Red
      float t = (float)(err - 50.0) / 150.0f;
      r = 255;
      g = (int)((1.0f - t) * 255.0f);
      b = 0;
      a = base_alpha;
    }
    else
    {
      // Red fading out
      r = 255;
      g = 0;
      b = 0;
      float t = (float)(err - 200.0) / 300.0f; // Fade out over next 300m
      a = (int)(base_alpha * (1.0f - t));
      if (a < 0)
        a = 0;
    }
    return IM_COL32(r, g, b, a);
  };

  // Draw interpolated quads
  for (int y = 0; y < rows; ++y)
  {
    for (int x = 0; x < cols; ++x)
    {
      float px = canvas_p0.x + x * grid_step;
      float py = canvas_p0.y + y * grid_step;

      ImU32 c_tl = error_to_col(errors_grid[y * (cols + 1) + x]);
      ImU32 c_tr = error_to_col(errors_grid[y * (cols + 1) + (x + 1)]);
      ImU32 c_bl = error_to_col(errors_grid[(y + 1) * (cols + 1) + x]);
      ImU32 c_br = error_to_col(errors_grid[(y + 1) * (cols + 1) + (x + 1)]);

      // Optimization: Don't draw if all transparent
      if ((c_tl >> 24) == 0 && (c_tr >> 24) == 0 && (c_bl >> 24) == 0 && (c_br >> 24) == 0)
        continue;

      draw_list->AddRectFilledMultiColor(ImVec2(px, py), ImVec2(px + grid_step, py + grid_step), c_tl, c_tr, c_br, c_bl);
    }
  }
}

auto map_widget_t::render_test_point(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  if (!m_has_test_point)
    return;

  // We need a result to draw (instantaneous for jitter)
  tdoa_result_t result_to_draw;

  // --- Calculate Test Point Data ---
  if (sensors.size() >= 3 && m_tdoa_solver)
  {
    // 1. Calculate ideal TDOAs
    // Use m_target_alt_agl as the "Truth" altitude (assuming map surface is 0)
    double truth_alt = static_cast<double>(m_target_alt_agl);
    auto ideal_tdoa = m_tdoa_solver->calculate_tdoa(sensors, m_test_point_lat, m_test_point_lon, truth_alt);

    // 1b. Inject Synthetic Noise (Jitter) NO CACHING - Generate every frame for visual "cloud"
    static std::mt19937 gen(12345);
    double jitter_ns = static_cast<double>(m_timing_jitter_ns);
    std::normal_distribution<double> d(0.0, jitter_ns);

    std::vector<double> noisy_tdoa = ideal_tdoa;
    for (size_t i = 1; i < noisy_tdoa.size(); ++i)
    {
      noisy_tdoa[i] += d(gen);
    }

    // 2. Solve (Verify Geometry) with NOISY data
    // Use 2D Solver
    tdoa_result_t instant_2d = m_tdoa_solver->solve_position_2d(sensors, noisy_tdoa, m_test_point_lat, m_test_point_lon, 0.0);
    instant_2d.error_estimate_m = m_tdoa_solver->estimate_2d_positioning_error(sensors, m_test_point_lat, m_test_point_lon, 0.0, jitter_ns);

    // 3D Solver
    tdoa_result_t instant_3d = m_tdoa_solver->solve_position(sensors, noisy_tdoa, m_test_point_lat, m_test_point_lon, 0.0);
    instant_3d.error_estimate_m = m_tdoa_solver->estimate_positioning_error(sensors, m_test_point_lat, m_test_point_lon, 0.0, jitter_ns);

    // --- Smoothing for UI Stability ---
    float alpha = 0.05f; // Smoothing factor (low pass)

    auto smooth_result = [&](const tdoa_result_t &current, const tdoa_result_t &instant) -> tdoa_result_t
    {
      if (!current.converged && instant.converged)
        return instant; // Snap to first valid
      if (!instant.converged)
        return current; // Keep last valid or failed state

      tdoa_result_t res = instant;
      // LERP
      res.latitude = current.latitude * (1.0 - alpha) + instant.latitude * alpha;
      res.longitude = current.longitude * (1.0 - alpha) + instant.longitude * alpha;
      res.altitude = current.altitude * (1.0 - alpha) + instant.altitude * alpha;
      res.error_estimate_m = current.error_estimate_m * (1.0 - alpha) + instant.error_estimate_m * alpha;
      res.gdop = current.gdop * (1.0 - alpha) + instant.gdop * alpha;
      return res;
    };

    // Update stored results (SMOOTHED) for UI
    m_test_result_2d = smooth_result(m_test_result_2d, instant_2d);
    m_test_result_3d = smooth_result(m_test_result_3d, instant_3d);

    // Use INSTANT 2D result for drawing (Visual Jitter)
    result_to_draw = instant_2d;
  }
  else
  {
    m_test_result_2d = tdoa_result_t{};
    m_test_result_3d = tdoa_result_t{};
    result_to_draw = tdoa_result_t{};
  }

  // --- Render Markers ---

  auto latlon_to_screen = [&](double lat, double lon) -> ImVec2
  {
    double wx, wy;
    geo::lat_lon_to_world(lat, lon, wx, wy);

    double center_wx, center_wy;
    geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);

    if (wx - center_wx > 0.5)
      wx -= 1.0;
    if (wx - center_wx < -0.5)
      wx += 1.0;

    ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);
    double tile_size = 256.0;
    double world_size_pixels = std::pow(2.0, m_zoom) * tile_size;

    float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
    float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);
    return ImVec2(px, py);
  };

  ImVec2 p_truth = latlon_to_screen(m_test_point_lat, m_test_point_lon);

  // Draw Truth Marker (Red Crosshair)
  float size = 10.0f;
  draw_list->AddLine(ImVec2(p_truth.x - size, p_truth.y), ImVec2(p_truth.x + size, p_truth.y), IM_COL32(255, 255, 255, 200), 2.0f);
  draw_list->AddLine(ImVec2(p_truth.x, p_truth.y - size), ImVec2(p_truth.x, p_truth.y + size), IM_COL32(255, 255, 255, 200), 2.0f);
  draw_list->AddCircle(p_truth, size * 0.6f, IM_COL32(255, 0, 0, 255), 0, 2.0f);
  draw_list->AddText(ImVec2(p_truth.x + size, p_truth.y - size), IM_COL32(255, 255, 255, 255), "Truth");

  // Draw Estimated Marker (Blue X) using INSTANT result
  if (sensors.size() >= 3 && result_to_draw.converged)
  {
    ImVec2 p_est = latlon_to_screen(result_to_draw.latitude, result_to_draw.longitude);

    // Draw Error Vector (Yellow Dashed)
    draw_list->AddLine(p_truth, p_est, IM_COL32(255, 255, 0, 180), 2.0f);

    // Draw 'X'
    draw_list->AddLine(ImVec2(p_est.x - size * 0.7f, p_est.y - size * 0.7f), ImVec2(p_est.x + size * 0.7f, p_est.y + size * 0.7f), IM_COL32(0, 100, 255, 255), 3.0f);
    draw_list->AddLine(ImVec2(p_est.x + size * 0.7f, p_est.y - size * 0.7f), ImVec2(p_est.x - size * 0.7f, p_est.y + size * 0.7f), IM_COL32(0, 100, 255, 255), 3.0f);
    draw_list->AddText(ImVec2(p_est.x + size, p_est.y - size), IM_COL32(100, 200, 255, 255), "Est");
  }
}

auto map_widget_t::draw_path_profile_window(elevation_service_t &elevation_service) -> void
{
  if (!m_show_profile_window || !m_has_profile_a || !m_has_profile_b)
    return;

  if (ImGui::Begin("Path Profile", &m_show_profile_window))
  {
    double dist_m = geo::distance(m_profile_a.lat, m_profile_a.lon, m_profile_b.lat, m_profile_b.lon);
    ImGui::Text("Distance: %.2f km", dist_m / 1000.0);

    // Settings
    static float freq_mhz = 2400.0f;
    ImGui::DragFloat("Frequency (MHz)", &freq_mhz, 10.0f, 100.0f, 60000.0f);
    static float h_tx = 10.0f;
    static float h_rx = 2.0f;
    ImGui::DragFloat("Tx Height (m)", &h_tx, 1.0f, 1.0f, 100.0f);
    ImGui::DragFloat("Rx Height (m)", &h_rx, 1.0f, 1.0f, 100.0f);

    // Get Profile (100 samples)
    int samples = 100;
    auto profile = elevation_service.get_profile(m_profile_a.lat, m_profile_a.lon, m_profile_b.lat, m_profile_b.lon, samples);

    if (profile.empty())
    {
      ImGui::End();
      return;
    }

    // Draw Graph
    ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
    ImVec2 canvas_sz = ImGui::GetContentRegionAvail();
    if (canvas_sz.y < 200)
      canvas_sz.y = 200;

    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(50, 50, 50, 255));

    ImGui::InvisibleButton("graph_canvas", canvas_sz);
    bool is_hovered_graph = ImGui::IsItemHovered();
    ImVec2 mouse_pos_graph = ImGui::GetMousePos();

    // Reset hover pos
    m_profile_hover_pos.reset();

    // Find min/max height
    float min_h = profile[0].second;
    float max_h = profile[0].second;
    for (const auto &p : profile)
    {
      if (p.second < min_h)
        min_h = p.second;
      if (p.second > max_h)
        max_h = p.second;
    }
    // Add margin
    max_h += std::max(h_tx, h_rx) + 20.0f; // Ensure LoS fits
    if (min_h > 0)
      min_h = 0; // Show sea level if positive

    float h_range = max_h - min_h;
    if (h_range < 10.0f)
      h_range = 10.0f;

    auto world_to_screen = [&](double d, float h) -> ImVec2
    {
      float x = canvas_p0.x + (float)(d / dist_m) * canvas_sz.x;
      float y = canvas_p0.y + canvas_sz.y - (float)((h - min_h) / h_range) * canvas_sz.y;
      return ImVec2(x, y);
    };

    // Draw Terrain
    for (size_t i = 0; i < profile.size() - 1; ++i)
    {
      ImVec2 p1 = world_to_screen(profile[i].first, profile[i].second);
      ImVec2 p2 = world_to_screen(profile[i + 1].first, profile[i + 1].second);
      draw_list->AddLine(p1, p2, IM_COL32(100, 200, 100, 255), 2.0f);
      // Fill below?
      draw_list->AddQuadFilled(p1, p2, ImVec2(p2.x, canvas_p0.y + canvas_sz.y), ImVec2(p1.x, canvas_p0.y + canvas_sz.y), IM_COL32(100, 200, 100, 100));
    }

    // Draw LoS Line
    // Include Earth Curvature effects for visualization
    float R_eff = 8504000.0f; // 4/3 Earth Radius

    // Calculate heights relative to the "straight line" chord connection
    // But standard graphs usually show "flat earth" with "curved LoS" OR "curved earth" with "straight LoS"
    // Let's implement: X-axis is distance along surface. Y-axis is Elevation.
    // If we want to show LoS obstruction correctly on a "flat X" graph, we should ADD the earth bulge to the TERRAIN.
    // Because the "straight ray" is straight, and the earth "bulges up" between the points.

    std::vector<ImVec2> terrain_points;
    for (size_t i = 0; i < profile.size(); ++i)
    {
      double d = profile[i].first;
      double d1 = d;
      double d2 = dist_m - d;
      float bulge = (float)((d1 * d2) / (2.0 * R_eff));

      // Draw separate line for "Apparent Terrain" (with curvature)
      float h_curved = profile[i].second + bulge;
      terrain_points.push_back(world_to_screen(d, h_curved));
    }

    // Draw Earth Curvature Terrain (The one that actually blocks LoS)
    for (size_t i = 0; i < terrain_points.size() - 1; ++i)
    {
      draw_list->AddLine(terrain_points[i], terrain_points[i + 1], IM_COL32(150, 255, 150, 150), 1.0f);
    }

    // LoS Line (Straight on this graph)
    float start_h_eff = profile[0].second + h_tx;
    float end_h_eff = profile.back().second + h_rx;

    ImVec2 los_p1 = world_to_screen(0.0, start_h_eff);
    ImVec2 los_p2 = world_to_screen(dist_m, end_h_eff);
    draw_list->AddLine(los_p1, los_p2, IM_COL32(255, 255, 0, 255), 2.0f);

    // Fresnel Zone Visualization
    static bool show_fresnel = true;
    ImGui::SetCursorPos(ImVec2(canvas_p0.x - ImGui::GetWindowPos().x + 10, canvas_p0.y - ImGui::GetWindowPos().y + 10));
    ImGui::Checkbox("Show 1st Fresnel Zone", &show_fresnel);

    if (show_fresnel)
    {
      std::vector<ImVec2> upper_curve;
      std::vector<ImVec2> lower_curve;

      // Calculate radius at each point
      // r = 17.32 * sqrt((d1 * d2) / (f_GHz * d_total))
      double dist_km = dist_m / 1000.0;
      double f_ghz = freq_mhz / 1000.0f;

      for (size_t i = 0; i < profile.size(); ++i)
      {
        double d1_m = profile[i].first;
        double d2_m = dist_m - d1_m;

        // Avoid division by zero at endpoints
        if (d1_m < 0.1 || d2_m < 0.1)
        {
          double h_los_at_point = start_h_eff + (end_h_eff - start_h_eff) * (d1_m / dist_m);
          upper_curve.push_back(world_to_screen(d1_m, (float)h_los_at_point));
          lower_curve.push_back(world_to_screen(d1_m, (float)h_los_at_point));
          continue;
        }

        double d1_km = d1_m / 1000.0;
        double d2_km = d2_m / 1000.0;

        // Radius in meters
        double radius_m = 17.32 * std::sqrt((d1_km * d2_km) / (f_ghz * dist_km));

        double h_los_at_point = start_h_eff + (end_h_eff - start_h_eff) * (d1_m / dist_m);

        upper_curve.push_back(world_to_screen(d1_m, (float)(h_los_at_point + radius_m)));
        lower_curve.push_back(world_to_screen(d1_m, (float)(h_los_at_point - radius_m)));
      }

      // Draw Curves
      if (upper_curve.size() > 1)
      {
        ImU32 fresnel_col = IM_COL32(100, 200, 255, 120); // Light Blue transparent
        draw_list->AddPolyline(upper_curve.data(), static_cast<int>(upper_curve.size()), fresnel_col, false, 1.5f);
        draw_list->AddPolyline(lower_curve.data(), static_cast<int>(lower_curve.size()), fresnel_col, false, 1.5f);

        // Connect endpoints for closed loop (optional, but Polyline is open)
      }
    }

    // Hover Logic
    if (is_hovered_graph)
    {
      float mx = mouse_pos_graph.x - canvas_p0.x;
      // Convert mx back to distance
      double d_hover = (static_cast<double>(mx) / canvas_sz.x) * dist_m;
      if (d_hover < 0)
        d_hover = 0;
      if (d_hover > dist_m)
        d_hover = dist_m;

      // Calculate interpolated position
      double bearing_AB = geo::bearing(m_profile_a.lat, m_profile_a.lon, m_profile_b.lat, m_profile_b.lon);
      double h_lat, h_lon;
      geo::destination_point(m_profile_a.lat, m_profile_a.lon, d_hover, bearing_AB, h_lat, h_lon);

      m_profile_hover_pos = {h_lat, h_lon};

      // Draw Vertical Line on Graph
      float screen_x = canvas_p0.x + mx;
      draw_list->AddLine(ImVec2(screen_x, canvas_p0.y), ImVec2(screen_x, canvas_p0.y + canvas_sz.y), IM_COL32(255, 255, 255, 100), 1.0f);

      // Find elevation at this point
      float h_terrain = 0;
      // Simple lookup in profile
      if (samples > 1)
      {
        int idx = static_cast<int>((d_hover / dist_m) * (samples - 1));
        if (idx >= 0 && static_cast<size_t>(idx) < profile.size())
          h_terrain = profile[idx].second;
      }

      // Tooltip
      ImGui::SetTooltip("Dist: %.2f km\nElev: %.1f m", d_hover / 1000.0, h_terrain);
    }

    // Fresnel Zone (1st)
    std::vector<ImVec2> fresnel_upper;
    std::vector<ImVec2> fresnel_lower;
    double lambda = 299.79 / freq_mhz;

    for (int i = 1; i < samples - 1; ++i)
    {
      double d = profile[i].first;
      double d1 = d;
      double d2 = dist_m - d;

      // Radius
      double r = rf_engine_t::calculate_fresnel_zone(d1, d2, lambda, 1);

      // Center of zone at this distance along LoS
      float t = (float)(d / dist_m);
      float h_los = start_h_eff + t * (end_h_eff - start_h_eff);

      fresnel_upper.push_back(world_to_screen(d, h_los + (float)r));
      fresnel_lower.push_back(world_to_screen(d, h_los - (float)r));
    }

    // Draw Fresnel Bounds
    for (size_t i = 0; i < fresnel_upper.size() - 1; ++i)
    {
      draw_list->AddLine(fresnel_upper[i], fresnel_upper[i + 1], IM_COL32(255, 100, 100, 100), 1.0f);
      draw_list->AddLine(fresnel_lower[i], fresnel_lower[i + 1], IM_COL32(255, 100, 100, 100), 1.0f);
    }
  }
  ImGui::End();
}

auto map_widget_t::export_coverage_map(const std::string &path) -> bool
{
  if (m_heatmap_texture_id == 0 || !m_rf_engine)
    return false;

  return image_exporter_t::save_texture_to_png(m_heatmap_texture_id, m_rf_engine->get_width(), m_rf_engine->get_height(), path);
}

auto map_widget_t::lat_lon_to_screen(double lat, double lon, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) const -> ImVec2
{
  double wx, wy;
  geo::lat_lon_to_world(lat, lon, wx, wy);

  double center_wx, center_wy;
  geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);

  // Wrap adjustments relative to center
  if (wx - center_wx > 0.5)
    wx -= 1.0;
  if (wx - center_wx < -0.5)
    wx += 1.0;

  const double tile_size = 256.0;
  double n = std::pow(2.0, m_zoom);
  double world_size_pixels = n * tile_size;

  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);
  float px = static_cast<float>((wx - center_wx) * world_size_pixels + screen_center.x);
  float py = static_cast<float>((wy - center_wy) * world_size_pixels + screen_center.y);

  return ImVec2(px, py);
}

auto map_widget_t::screen_to_lat_lon(const ImVec2 &p, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz, double &lat_out, double &lon_out) const -> void
{
  double center_wx, center_wy;
  geo::lat_lon_to_world(m_center_lat, m_center_lon, center_wx, center_wy);

  const double tile_size = 256.0;
  double n = std::pow(2.0, m_zoom);
  double world_size_pixels = n * tile_size;

  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

  double dx = (p.x - screen_center.x) / world_size_pixels;
  double dy = (p.y - screen_center.y) / world_size_pixels;

  double wx = center_wx + dx;
  double wy = center_wy + dy;

  // Wrap X
  if (wx < 0.0)
    wx = std::fmod(wx, 1.0) + 1.0;
  if (wx > 1.0)
    wx = std::fmod(wx, 1.0);

  // Clamp Y
  if (wy < 0.0)
    wy = 0.0;
  if (wy > 1.0)
    wy = 1.0;

  geo::world_to_lat_lon(wx, wy, lat_out, lon_out);
}

auto map_widget_t::has_buildings_for_area(double min_lat, double max_lat, double min_lon, double max_lon) const -> bool
{
  if (!m_building_service)
    return false;
  return m_building_service->has_buildings_for_area(min_lat, max_lat, min_lon, max_lon);
}

auto map_widget_t::get_building_loading_status() const -> std::pair<int, int>
{
  if (!m_building_service)
    return {0, 0};
  return m_building_service->get_loading_status();
}

auto map_widget_t::render_scale_bar(ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  // Position: Bottom Left, with some padding

  // Calculate Meters per Pixel at current latitude
  // Earth Circumference at Equator ~ 40,075,017 m
  // Meters per pixel = C * cos(lat) / 2^(zoom + 8)  (assuming 256px tiles)
  double world_size_px = 256.0 * std::pow(2.0, m_zoom);
  double equatorial_circumference = 40075017.0;
  double lat_rad = m_center_lat * geo::PI / 180.0;
  double meters_per_pixel = (equatorial_circumference * std::cos(lat_rad)) / world_size_px;

  // Target width in pixels (approx 100px)
  double target_width_px = 100.0;
  double target_meters = target_width_px * meters_per_pixel;

  // Round target_meters to nice number (1, 2, 5, 10, 20, 50, 100, 200, 500, 1km, etc)
  double magnitude = std::pow(10.0, std::floor(std::log10(target_meters)));
  double residual = target_meters / magnitude;
  double rounded_meters;

  if (residual > 5.0)
    rounded_meters = 5.0 * magnitude;
  else if (residual > 2.0)
    rounded_meters = 2.0 * magnitude;
  else
    rounded_meters = 1.0 * magnitude;

  // Recalculate pixel width
  float bar_width_px = static_cast<float>(rounded_meters / meters_per_pixel);

  // Draw
  ImVec2 bar_start = ImVec2(canvas_p0.x + canvas_sz.x - 20.0f - bar_width_px, canvas_p0.y + canvas_sz.y - 9.0f);
  ImVec2 bar_end = ImVec2(bar_start.x + bar_width_px, bar_start.y);

  // Background for contrast - Darker and more opaque (High Contrast)
  draw_list->AddRectFilled(ImVec2(bar_start.x - 8, bar_start.y - 18), ImVec2(bar_end.x + 8, bar_start.y + 4), IM_COL32(30, 30, 30, 220), 4.0f);

  // Bar line - White
  draw_list->AddLine(bar_start, bar_end, IM_COL32(255, 255, 255, 255), 2.0f);

  // Ticks - White
  draw_list->AddLine(ImVec2(bar_start.x, bar_start.y - 6), ImVec2(bar_start.x, bar_start.y + 0), IM_COL32(255, 255, 255, 255), 2.0f);
  draw_list->AddLine(ImVec2(bar_end.x, bar_end.y - 6), ImVec2(bar_end.x, bar_end.y + 0), IM_COL32(255, 255, 255, 255), 2.0f);

  // Label
  std::string label;
  if (rounded_meters >= 1000.0)
    label = std::format("{:.0f} km", rounded_meters / 1000.0);
  else
    label = std::format("{:.0f} m", rounded_meters);

  ImVec2 text_sz = ImGui::CalcTextSize(label.c_str());
  ImVec2 text_pos = ImVec2(bar_start.x + (bar_width_px - text_sz.x) * 0.5f, bar_start.y - text_sz.y - 2.0f);

  // Shadow text
  draw_list->AddText(ImVec2(text_pos.x + 1, text_pos.y + 1), IM_COL32(0, 0, 0, 255), label.c_str());
  draw_list->AddText(text_pos, IM_COL32(255, 255, 255, 255), label.c_str());
}

auto map_widget_t::render_compass(ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  // Position: Top Right
  float pad = 20.0f;
  float x = canvas_p0.x + canvas_sz.x - pad - 20.0f;
  float y = canvas_p0.y + pad + 30.0f; // Below menu bar/tabs? Or just Top Right.

  // ImGui::InvisibleButton("map_canvas", canvas_sz); covers the whole area, so inputs might be eaten,
  // but this is just rendering.

  ImVec2 center = ImVec2(x, y);
  float radius = 15.0f;

  // Draw Circle Background
  draw_list->AddCircleFilled(center, radius, IM_COL32(0, 0, 0, 100));
  draw_list->AddCircle(center, radius, IM_COL32(255, 255, 255, 200), 0, 1.5f);

  // Draw Arrow (North is always Up for now)
  ImVec2 north = ImVec2(center.x, center.y - radius + 4.0f);
  ImVec2 south = ImVec2(center.x, center.y + radius - 4.0f);
  ImVec2 west = ImVec2(center.x - radius * 0.4f, center.y);
  ImVec2 east = ImVec2(center.x + radius * 0.4f, center.y);

  // Red North Tip
  draw_list->AddTriangleFilled(north, west, east, IM_COL32(255, 50, 50, 255));

  // White South Tail
  draw_list->AddTriangleFilled(south, west, east, IM_COL32(230, 230, 230, 255));

  // N label
}

auto map_widget_t::import_drone_path(const std::string &filepath) -> bool
{
  std::vector<log_point_t> points;
  if (filepath.ends_with(".gpx") || filepath.ends_with(".GPX"))
  {
    points = LogParser::parse_gpx(filepath);
  }
  else
  {
    points = LogParser::parse_csv(filepath);
  }

  if (points.empty())
    return false;

  m_drone_path.clear();
  m_path_time_offsets.clear();

  // Convert logs to path
  for (const auto &p : points)
  {
    m_drone_path.push_back({p.lat, p.lon});
    // For time, if not present, we can synthesize it based on distance + speed (e.g. 10m/s)
    // For now, if log has time, use it.
    if (p.has_time)
      m_path_time_offsets.push_back(p.time_sec);
  }

  // Synthesize time if missing
  if (m_path_time_offsets.empty() && !m_drone_path.empty())
  {
    double current_time = 0.0;
    m_path_time_offsets.push_back(0.0);
    for (size_t i = 1; i < m_drone_path.size(); ++i)
    {
      double d = geo::distance(m_drone_path[i - 1].first, m_drone_path[i - 1].second, m_drone_path[i].first, m_drone_path[i].second);
      // Assume 10 m/s
      current_time += d / 15.0; // 15 m/s (~54km/h) default
      m_path_time_offsets.push_back(current_time);
    }
  }

  m_playback_state.duration_sec = m_path_time_offsets.empty() ? 0.0 : m_path_time_offsets.back();
  m_playback_state.current_time_sec = 0.0;
  m_playback_state.is_playing = false;

  // Center map on path start
  if (!m_drone_path.empty())
  {
    set_center(m_drone_path[0].first, m_drone_path[0].second);
  }

  return true;
}

auto map_widget_t::update_playback(double dt) -> void
{
  if (!m_playback_state.is_playing || m_drone_path.empty())
    return;

  // Synthesize timing for manual paths if missing
  if (m_path_time_offsets.empty() && !m_drone_path.empty())
  {
    m_path_time_offsets.clear();
    m_path_time_offsets.push_back(0.0);
    double current_time = 0.0;
    for (size_t i = 1; i < m_drone_path.size(); ++i)
    {
      double d = geo::distance(m_drone_path[i - 1].first, m_drone_path[i - 1].second, m_drone_path[i].first, m_drone_path[i].second);
      // Assume 15 m/s (~54km/h) default speed
      current_time += d / 15.0;
      m_path_time_offsets.push_back(current_time);
    }
    m_playback_state.duration_sec = current_time;
    // If loop was closed (first == last), calculation is correct.
  }

  if (m_path_time_offsets.empty())
    return;

  m_playback_state.current_time_sec += dt * m_playback_state.speed_multiplier;

  if (m_playback_state.current_time_sec >= m_playback_state.duration_sec)
  {
    if (m_playback_state.loop)
    {
      m_playback_state.current_time_sec = 0.0;
    }
    else
    {
      m_playback_state.current_time_sec = m_playback_state.duration_sec;
      m_playback_state.is_playing = false;
    }
  }

  set_playback_pos(m_playback_state.current_time_sec / m_playback_state.duration_sec);
}

auto map_widget_t::set_playback_pos(double t) -> void
{
  if (m_path_time_offsets.empty())
    return;

  double target_time = t * m_playback_state.duration_sec;
  m_playback_state.current_time_sec = target_time;

  // Find index in m_path_time_offsets
  auto it = std::lower_bound(m_path_time_offsets.begin(), m_path_time_offsets.end(), target_time);
  size_t idx_next = std::distance(m_path_time_offsets.begin(), it);

  double lat = 0.0, lon = 0.0;

  if (idx_next == 0)
  {
    lat = m_drone_path[0].first;
    lon = m_drone_path[0].second;
  }
  else if (idx_next >= m_path_time_offsets.size())
  {
    lat = m_drone_path.back().first;
    lon = m_drone_path.back().second;
  }
  else
  {
    // Interpolate
    size_t idx_prev = idx_next - 1;
    double t0 = m_path_time_offsets[idx_prev];
    double t1 = m_path_time_offsets[idx_next];
    if (t1 > t0)
    {
      double ratio = (target_time - t0) / (t1 - t0);
      geo::interpolate(m_drone_path[idx_prev].first, m_drone_path[idx_prev].second, m_drone_path[idx_next].first, m_drone_path[idx_next].second, ratio, lat, lon);
    }
    else
    {
      lat = m_drone_path[idx_prev].first;
      lon = m_drone_path[idx_prev].second;
    }
  }

  // Update TDOA Test Point to match drone position
  // This drives the TDOA visualization
  set_tdoa_test_point(lat, lon);

  // Also we can update signal indicators here if we had logic for it,
  // but TDOA test point update will trigger needed recalculations if we hook it up.
  // Although `set_tdoa_test_point` just sets member variables and runs solver.
  // It doesn't update the UI bars automatically unless UI reads from it.

  // For the plan purpose (Visualise TDOA Hyperbolas shifting), this is sufficient.
}
} // namespace sensor_mapper
