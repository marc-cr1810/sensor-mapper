#include "ui/map_widget.hpp"
#include "core/building_service.hpp"
#include "core/elevation_service.hpp"
#include "core/geo_math.hpp"
#include "core/sensor.hpp"
#include "core/tdoa_solver.hpp"
#include "core/tile_service.hpp"
#include "imgui.h"

#include "imgui_impl_opengl3.h"
#include "renderer/gpu_rf_engine.hpp" // GPU Engine
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

map_widget_t::map_widget_t() : m_center_lat(-33.8688), m_center_lon(151.2093), m_zoom(14.0), m_show_rf_gradient(false)
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
  m_building_service->update();

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
  else if (source_index == 1)
  {
    m_tile_service->set_source(tile_service_t::tile_source_t::TERRARIUM);
  }
  else
  {
    m_tile_service->set_source(tile_service_t::tile_source_t::SATELLITE);
  }
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

auto map_widget_t::fetch_buildings_near(double lat, double lon) -> void
{
  if (!m_building_service)
    return;

  // Fetch a small area around the point (approx +/- 0.005 degrees, ~500m)
  double delta = 0.005;
  m_building_service->fetch_buildings(lat - delta, lat + delta, lon - delta, lon + delta);
}

auto map_widget_t::draw(const std::vector<sensor_t> &sensors, int &selected_index, elevation_service_t &elevation_service, std::function<void(double, double)> on_add_sensor) -> void
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

  ImGui::Text("Cursor: %.5f, %.5f | Alt: %.1f m | Zoom: %.2f", m_mouse_lat, m_mouse_lon, m_cursor_alt, m_zoom);

  // Zoom controls (Buttons)
  if (ImGui::Button("Zoom In"))
    set_zoom(m_zoom + 1.0);
  ImGui::SameLine();
  if (ImGui::Button("Zoom Out"))
    if (ImGui::Button("Zoom Out"))
      set_zoom(m_zoom - 1.0);

  ImGui::SameLine();
  if (ImGui::Combo("Map Source", &m_current_map_source, "OpenStreetMap\0Terrain Heightmap\0Satellite (Esri)\0"))
  {
    set_map_source(m_current_map_source);
  }

  // Simple canvas
  ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();
  ImVec2 canvas_sz_raw = ImGui::GetContentRegionAvail();
  ImVec2 canvas_sz(static_cast<float>(canvas_sz_raw.x < 50.0f ? 50.0f : canvas_sz_raw.x), static_cast<float>(canvas_sz_raw.y < 50.0f ? 50.0f : canvas_sz_raw.y));

  ImDrawList *draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(20, 20, 20, 255));
  draw_list->AddRect(canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), IM_COL32(255, 255, 255, 100));

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

  // Right Click to Open Context Menu
  if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Right))
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
    }
  }

  // Panning
  if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
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

  // Screen center
  ImVec2 screen_center = ImVec2(canvas_p0.x + canvas_sz.x * 0.5f, canvas_p0.y + canvas_sz.y * 0.5f);

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
  if (m_show_buildings && m_building_service)
  {
    // Determine visible area
    double min_lat, max_lat, min_lon, max_lon;
    geo::world_to_lat_lon(min_wx, min_wy, max_lat,
                          min_lon); // Y is inverted in world coords usually?
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

    for (const auto *building : buildings)
    {
      if (!building || building->footprint.empty())
        continue;

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

      if (screen_points.size() >= 3)
      {
        // Draw simple filled polygon
        draw_list->AddConvexPolyFilled(screen_points.data(), screen_points.size(), IM_COL32(100, 100, 100, 150));
        draw_list->AddPolyline(screen_points.data(), screen_points.size(), IM_COL32(200, 200, 200, 255), true, 1.0f);

        // "3D" Extrusion (fake perspective)
        // If we want 3D, we need to project "up".
        // In top-down 2D map, "up" is towards the camera.
        // A simple way to visualize height is to offset the "roof" based on the
        // vector from map center to building center (creating a vanishing point
        // effect), OR just purely based on a fixed "up" vector (isometric-ish).
        // Let's try a simple "perspective" offset from the center of the screen
        // to simulate 3D.

        // Calculate building center screen pos

        // Perspective factor: further from center = more tilt.
        // But height_m needs to be converted to pixels.
        // 1 meter in pixels approx?
        // Earth circum ~ 40Mm. World size = 256 * 2^zoom.
        // pixels_per_meter = world_size_pixels / (40075000.0 * cos(lat))
        // pixels_per_meter = world_size_pixels / (40075000.0 * cos(lat))
        double meters_per_pixel = (40075000.0 * std::cos(m_center_lat * 3.14159 / 180.0)) / world_size_pixels;
        float height_px = static_cast<float>(building->height_m / meters_per_pixel);

        // Vanishing point is screen_center.
        // Roof points = Base points + (Base points - screen_center) *
        // scale_factor? Actually, just moving "up" in Y makes it look like we
        // are looking from South? Let's do a "radial" displacement for a
        // top-down perspective view.

        std::vector<ImVec2> roof_points;
        roof_points.reserve(screen_points.size());

        for (const auto &p : screen_points)
        {

          // Limit dir length to avoid exploding infinity
          // Just use a fixed "view angle" simulation.
          // Let's simply offset Y by height_px (isometric view from slightly
          // below/South) roof_points.push_back(ImVec2(p.x, p.y - height_px));
          // // Isometric-ish

          // Radial displacement (Fish-eye / True Top-down perspective)
          // If we are looking straight down at the center, things at the edges
          // lean out. offset = dir * (height / camera_height_simulation) Assume
          // camera is at some height? Let's try simple Y offset for now to be
          // safe and clear.
          roof_points.push_back(ImVec2(p.x, p.y - height_px));
        }

        // Culling & LOD Logic
        // Calculate screen-space bounding box
        float min_x = screen_points[0].x, max_x = screen_points[0].x;
        float min_y = screen_points[0].y, max_y = screen_points[0].y;
        for (const auto &p : screen_points)
        {
          min_x = std::min(min_x, p.x);
          max_x = std::max(max_x, p.x);
          min_y = std::min(min_y, p.y);
          max_y = std::max(max_y, p.y);
        }

        // Add roof height to bbox
        if (!roof_points.empty())
        {
          for (const auto &p : roof_points)
          {
            min_y = std::min(min_y, p.y); // Roof is above (lower Y)
          }
        }

        float width = max_x - min_x;
        float height = max_y - min_y;

        // Skip if completely off-screen (with margin)
        if (max_x < canvas_p0.x || min_x > canvas_p0.x + canvas_sz.x || max_y < canvas_p0.y || min_y > canvas_p0.y + canvas_sz.y)
        {
          continue;
        }

        // LOD: Skip tiny buildings
        if (width < 3.0f || height < 3.0f)
        {
          // Too small to matter
          continue;
        }

        // LOD: Simplified drawing for small buildings
        if (width < 10.0f)
        {
          // Draw simple rect
          draw_list->AddRectFilled(ImVec2(min_x, min_y), ImVec2(max_x, max_y), IM_COL32(100, 100, 100, 150));
          continue;
        }

        // Draw Roof
        draw_list->AddConvexPolyFilled(roof_points.data(), roof_points.size(), IM_COL32(140, 140, 140, 200));
        draw_list->AddPolyline(roof_points.data(), roof_points.size(), IM_COL32(220, 220, 220, 255), true, 1.0f);

        // Draw Walls (connect base to roof)
        for (int i = 0; i < static_cast<int>(screen_points.size()); ++i)
        {
          size_t next = (static_cast<size_t>(i) + 1) % screen_points.size();
          ImVec2 p1 = screen_points[static_cast<size_t>(i)];
          ImVec2 p2 = screen_points[next];
          ImVec2 r1 = roof_points[static_cast<size_t>(i)];
          ImVec2 r2 = roof_points[next];

          // Draw quad
          ImVec2 quad[4] = {p1, p2, r2, r1};
          draw_list->AddConvexPolyFilled(quad, 4, IM_COL32(80, 80, 80, 180));
        }
      }
    }
  }

  // --- Draw All Sensors ---
  auto dbm_to_color_lambda = [](double p_rx_dbm) -> ImU32
  {
    int r, g, b, a;
    if (p_rx_dbm >= -60.0)
    {
      r = 0;
      g = 255;
      b = 0;
      a = 200;
    }
    else if (p_rx_dbm >= -80.0)
    {
      float t = static_cast<float>((p_rx_dbm + 80.0) / 20.0);
      r = static_cast<int>((1.0f - t) * 255.0f);
      g = 255;
      b = 0;
      a = 180;
    }
    else if (p_rx_dbm >= -100.0)
    {
      float t = static_cast<float>((p_rx_dbm + 100.0) / 20.0);
      r = 255;
      g = static_cast<int>(t * 255.0f);
      b = 0;
      a = 120;
    }
    else
    {
      float t = static_cast<float>(std::max(0.0, (p_rx_dbm + 120.0) / 20.0));
      r = 255;
      g = 0;
      b = 0;
      a = static_cast<int>(t * 80.0f);
    }
    return IM_COL32(r, g, b, a);
  };

  if (m_show_composite)
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
      double render_min_lat = min_lat - lat_margin;
      double render_max_lat = max_lat + lat_margin;
      double render_min_lon = min_lon - lon_margin;
      double render_max_lon = max_lon + lon_margin;

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

        m_heatmap_texture_id = m_rf_engine->render(sensors, &elevation_service, m_building_service.get(), render_min_lat, render_max_lat, render_min_lon, render_max_lon, m_min_signal_dbm);
        m_heatmap_dirty = false;
        m_last_render_time = current_time;

        // Update tracked view with expanded bounds
        m_prev_min_lat = render_min_lat;
        m_prev_max_lat = render_max_lat;
        m_prev_min_lon = render_min_lon;
        m_prev_max_lon = render_max_lon;
        m_prev_sensor_count = sensors.size();

        // Cache the actual render bounds for UV calculation
        m_cached_render_min_lat = render_min_lat;
        m_cached_render_max_lat = render_max_lat;
        m_cached_render_min_lon = render_min_lon;
        m_cached_render_max_lon = render_max_lon;
      }

      // Draw GPU-rendered texture overlay (if enabled)
      if (m_heatmap_texture_id != 0 && m_show_heatmap_overlay)
      {
        // Calculate UV coordinates to map viewport to the larger rendered area
        float u_min = (float)((min_lon - m_cached_render_min_lon) / (m_cached_render_max_lon - m_cached_render_min_lon));
        float u_max = (float)((max_lon - m_cached_render_min_lon) / (m_cached_render_max_lon - m_cached_render_min_lon));
        float v_min = (float)((max_lat - m_cached_render_max_lat) / (m_cached_render_min_lat - m_cached_render_max_lat));
        float v_max = (float)((min_lat - m_cached_render_max_lat) / (m_cached_render_min_lat - m_cached_render_max_lat));

        // Clamp UV coordinates to valid range
        // Since we use GL_CLAMP_TO_EDGE, falling outside 0-1 will repeat edge pixels (stretching).
        // With the 100% margin above, this should rarely happen unless panning is extremely fast.
        u_min = std::max(0.0f, std::min(1.0f, u_min));
        u_max = std::max(0.0f, std::min(1.0f, u_max));
        v_min = std::max(0.0f, std::min(1.0f, v_min));
        v_max = std::max(0.0f, std::min(1.0f, v_max));

        draw_list->AddImage((ImTextureID)(intptr_t)m_heatmap_texture_id, canvas_p0, ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y), ImVec2(u_min, v_min), ImVec2(u_max, v_max));
      }
    }

    // Draw Markers Only
    for (size_t idx = 0; idx < sensors.size(); ++idx)
    {
      const auto &sensor = sensors[idx];
      bool is_selected = (static_cast<int>(idx) == selected_index);

      // Calculate center screen pos
      double s_lat = sensor.get_latitude();
      double s_lon = sensor.get_longitude();
      double c_wx, c_wy;
      geo::lat_lon_to_world(s_lat, s_lon, c_wx, c_wy);
      if (c_wx - center_wx > 0.5)
        c_wx -= 1.0;
      if (c_wx - center_wx < -0.5)
        c_wx += 1.0;

      float cx = static_cast<float>((c_wx - center_wx) * world_size_pixels + screen_center.x);
      float cy = static_cast<float>((c_wy - center_wy) * world_size_pixels + screen_center.y);

      auto col = sensor.get_color();
      ImU32 marker_col = IM_COL32(static_cast<int>(col[0] * 255), static_cast<int>(col[1] * 255), static_cast<int>(col[2] * 255), 255);
      draw_list->AddCircleFilled(ImVec2(cx, cy), 5.0f, marker_col);
      if (is_selected)
      {
        draw_list->AddCircle(ImVec2(cx, cy), 8.0f, IM_COL32(255, 255, 0, 255), 0, 2.0f);
      }

      // Hit Test
      if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
      {
        float dx = io.MousePos.x - cx;
        float dy = io.MousePos.y - cy;
        if (dx * dx + dy * dy < 100.0f)
        {
          selected_index = static_cast<int>(idx);
        }
      }
    }
  }
  else
  {
    // Normal Rendering Loop
    for (size_t idx = 0; idx < sensors.size(); ++idx)
    {
      const auto &sensor = sensors[idx];
      bool is_selected = (static_cast<int>(idx) == selected_index);
      double s_lat = sensor.get_latitude();
      double s_lon = sensor.get_longitude();
      double range_m = sensor.get_range();

      // Attempt to get elevation for Occlusion Casting
      float sensor_h;
      bool has_elevation = false;

      if (sensor.get_use_auto_elevation())
      {
        has_elevation = elevation_service.get_elevation(s_lat, s_lon, sensor_h);
        if (!has_elevation)
        {
          sensor_h = 0.0f; // Default to sea level if no data yet
        }
      }
      else
      {
        sensor_h = static_cast<float>(sensor.get_ground_elevation());
        has_elevation = true;
      }

      // Add mast height
      sensor_h += static_cast<float>(sensor.get_mast_height());

      // --- Caching Logic ---
      // Create a unique hash key for the current state of the sensor view
      std::string current_hash = std::format("{:.6f}_{:.6f}_{:.1f}_{:.1f}_{}", s_lat, s_lon, range_m, (double)sensor_h,
                                             has_elevation); // sensor_h includes mast + ground

      // Add Directional & Model parameters to hash
      double azimuth_deg = sensor.get_azimuth_deg();
      double beamwidth_deg = sensor.get_beamwidth_deg();
      PropagationModel prop_model = sensor.get_propagation_model();

      // Add antenna pattern to hash to invalidate cache when pattern changes
      auto pattern = sensor.get_pattern();
      std::string pattern_name = pattern ? pattern->name : "none";

      current_hash += std::format("_{:.1f}_{:.1f}_{}_{}", azimuth_deg, beamwidth_deg, static_cast<int>(prop_model), pattern_name);

      // Check Cache
      bool cache_valid = false;
      const std::string &s_id = sensor.get_id();
      auto cache_it = m_view_cache.find(s_id);

      // Prepare points vector
      std::vector<ImVec2> points;

      // Note: We need world-space points later for drawing, but the cache
      // should store relative or world? The drawing logic converts lat/lon to
      // world to screen. Screen depends on Zoom/Center. So we CANNOT cache screen coords if we Pan/Zoom. We MUST
      // cache the resulting polygon vertices in Lat/Lon (or World Space). Let's
      // modify the loop to generate Lat/Lon points, then a second pass to
      // convert to Screen.

      if (cache_it != m_view_cache.end() && cache_it->second.hash_key == current_hash)
      {
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

        for (const auto &pt : cached_latlons)
        {
          double p_lat = pt.x;
          double p_lon = pt.y;
          double p_wx, p_wy;
          geo::lat_lon_to_world(p_lat, p_lon, p_wx, p_wy);

          // Simple Wrap check
          if (p_wx - center_wx > 0.5)
            p_wx -= 1.0;
          if (p_wx - center_wx < -0.5)
            p_wx += 1.0;

          float px = static_cast<float>((p_wx - center_wx) * world_size_pixels + screen_center.x);
          float py = static_cast<float>((p_wy - center_wy) * world_size_pixels + screen_center.y);
          points.push_back(ImVec2(px, py));
        }
        cache_valid = true;
      }

      if (!cache_valid)
      {
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

        for (int i = 0; i < segments; ++i)
        {
          double angle = (360.0 / segments) * i;

          // Start at max range and work inward to find edge of coverage
          double coverage_dist = range_m;

          // Search for coverage boundary using binary search-like approach
          double min_dist = 0.0;
          double max_dist = range_m;
          const int search_steps = 20; // Precision of coverage boundary

          for (int step = 0; step < search_steps; ++step)
          {
            double test_dist = (min_dist + max_dist) / 2.0;
            if (test_dist < 10.0)
              break; // Minimum distance

            double t_lat, t_lon;
            geo::destination_point(s_lat, s_lon, test_dist, angle, t_lat, t_lon);

            // Calculate Path Loss based on Model
            double d_km = test_dist / 1000.0;
            double path_loss_db = 0.0;
            if (prop_model == PropagationModel::FreeSpace)
            {
              path_loss_db = rf_engine_t::calculate_fspl(d_km, frequency_mhz);
            }
            else
            {
              // Assume 2m receiver height
              path_loss_db = rf_engine_t::calculate_hata(d_km, frequency_mhz, sensor_h, 2.0, prop_model);
            }

            // Calculate terrain attenuation
            double terrain_loss_db = 0.0;
            if (has_elevation)
            {
              float target_h_at_pt; // Renamed to avoid shadowing
              if (elevation_service.get_elevation(t_lat, t_lon, target_h_at_pt))
              {
                // Check line-of-sight obstruction
                const double step_size = 50.0;
                double current_dist = 0.0;

                double penetration_depth = 0.0;

                while (current_dist < test_dist)
                {
                  current_dist += step_size;
                  if (current_dist > test_dist)
                    current_dist = test_dist;

                  double check_lat, check_lon;
                  geo::destination_point(s_lat, s_lon, current_dist, angle, check_lat, check_lon);

                  float check_h;
                  if (elevation_service.get_elevation(check_lat, check_lon, check_h))
                  {
                    // Calculate expected height on LOS path
                    double progress = current_dist / test_dist;
                    double expected_h = sensor_h + (target_h_at_pt - sensor_h) * progress;

                    // Check for terrain obstruction
                    if (check_h > expected_h)
                    {
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

            // Calculate building attenuation
            double building_loss_db = 0.0;
            if (m_building_service)
            {
              geo_point_t start = {s_lat, s_lon};
              geo_point_t end = {t_lat, t_lon};
              auto buildings = m_building_service->get_buildings_on_path(start, end);

              for (const auto &ix : buildings)
              {
                // Check if valid intersection
                if (ix.building && ix.distance_through_m > 0.1)
                {
                  // Check height - does line of sight go over the building?
                  // Need building ground elevation. Assuming flat-ish/terrain
                  // elevation at sensor? Let's refine: Building base height =
                  // terrain height at entry point Ray height at entry point =
                  // sensor_h + (target_h - sensor_h) * progress

                  // Optimization: Just apply flat 15dB if it hits, assuming
                  // it's tall enough. Or use building->height_m.

                  // Simplified: If sensor is lower than building + 20m, apply
                  // attenuation. Ideally we interpolate terrain height at
                  // intersection point. For now, raw meters of concrete
                  // penetration.

                  // ITU-R P.2040 for concrete at 1GHz ~ 10-20 dB?
                  // Let's say 15 dB per wall (entry/exit) -> 30dB total + 0.5
                  // dB/m? Simple model: 20dB per building penetration.
                  building_loss_db += 20.0;
                }
              }
            }

            // Calculate Antenna Pattern Loss using actual antenna pattern
            double antenna_pattern_gain = sensor.get_antenna_gain(angle);
            double antenna_pattern_loss = -antenna_pattern_gain; // Convert gain to loss

            // Calculate received power
            // P_rx = P_tx + G_tx + G_rx - Path_Loss - Terrain - Building -
            // Pattern
            double p_rx_dbm = tx_power_dbm + tx_gain_dbi + rx_gain_dbi - path_loss_db - terrain_loss_db - building_loss_db - antenna_pattern_loss;

            // Check if signal is above sensitivity threshold
            if (p_rx_dbm >= rx_sensitivity_dbm)
            {
              // Signal is receivable, try further out
              min_dist = test_dist;
            }
            else
            {
              // Signal too weak, try closer
              max_dist = test_dist;
            }
          }

          coverage_dist = min_dist;

          double p_lat, p_lon;
          geo::destination_point(s_lat, s_lon, coverage_dist, angle, p_lat, p_lon);

          // Calculate signal strength at this edge point for caching
          using geo::PI;
          double lat1_rad = s_lat * PI / 180.0;
          double lat2_rad = p_lat * PI / 180.0;
          double delta_lat = (p_lat - s_lat) * PI / 180.0;
          double delta_lon = (p_lon - s_lon) * PI / 180.0;

          double haversine_a = std::sin(delta_lat / 2.0) * std::sin(delta_lat / 2.0) + std::cos(lat1_rad) * std::cos(lat2_rad) * std::sin(delta_lon / 2.0) * std::sin(delta_lon / 2.0);
          double c = 2.0 * std::atan2(std::sqrt(haversine_a), std::sqrt(1.0 - haversine_a));
          double dist_m = geo::EARTH_RADIUS * c;
          if (dist_m < 1.0)
            dist_m = 1.0;

          double d_km = dist_m / 1000.0;
          double path_loss_db = 0.0;
          if (prop_model == PropagationModel::FreeSpace)
          {
            path_loss_db = rf_engine_t::calculate_fspl(d_km, frequency_mhz);
          }
          else
          {
            path_loss_db = rf_engine_t::calculate_hata(d_km, frequency_mhz, sensor_h, 2.0, prop_model);
          }

          // Quick terrain loss (simplified for edge points)
          double terrain_loss_db = 0.0;
          if (has_elevation && coverage_dist < range_m * 0.9)
          {
            // Only calculate terrain loss for significantly occluded points
            float h_at_pt; // Renamed to avoid shadowing
            if (elevation_service.get_elevation(p_lat, p_lon, h_at_pt))
            {
              if (h_at_pt > sensor_h + 10.0)
              {
                terrain_loss_db = (h_at_pt - sensor_h) * 5.0; // Simplified estimate
              }
            }
          }

          // Recalculate Antenna Pattern Loss for edge point using actual antenna pattern
          double antenna_pattern_gain = sensor.get_antenna_gain(angle);
          double antenna_pattern_loss = -antenna_pattern_gain; // Convert gain to loss

          double p_rx_dbm = tx_power_dbm + tx_gain_dbi + rx_gain_dbi - path_loss_db - terrain_loss_db - antenna_pattern_loss;

          // Add to cache (Lat/Lon and Signal Strength)
          new_cache_latlons.push_back(ImVec2(static_cast<float>(p_lat), static_cast<float>(p_lon)));
          new_cache_signal_dbm.push_back(p_rx_dbm);

          // Convert to Screen for drawing now
          double p_wx, p_wy;
          geo::lat_lon_to_world(p_lat, p_lon, p_wx, p_wy);
          if (p_wx - center_wx > 0.5)
            p_wx -= 1.0;
          if (p_wx - center_wx < -0.5)
            p_wx += 1.0;

          float px = static_cast<float>((p_wx - center_wx) * world_size_pixels + screen_center.x);
          float py = static_cast<float>((p_wy - center_wy) * world_size_pixels + screen_center.y);

          points.push_back(ImVec2(px, py));
        }

        // Update Cache
        m_view_cache[s_id] = {current_hash, new_cache_latlons, new_cache_signal_dbm};
      }

      // Colors
      auto col = sensor.get_color();
      // Range stays green
      ImU32 fill_col = is_selected ? IM_COL32(0, 255, 0, 100) : IM_COL32(0, 200, 0, 50);
      ImU32 border_col = is_selected ? IM_COL32(255, 255, 0, 255) : IM_COL32(0, 255, 0, 255);
      float thickness = is_selected ? 3.0f : 2.0f;

      // Calculate center screen pos (needed for triangle fan)
      double c_wx, c_wy;
      geo::lat_lon_to_world(s_lat, s_lon, c_wx, c_wy);
      if (c_wx - center_wx > 0.5)
        c_wx -= 1.0;
      if (c_wx - center_wx < -0.5)
        c_wx += 1.0;

      float cx = static_cast<float>((c_wx - center_wx) * world_size_pixels + screen_center.x);
      float cy = static_cast<float>((c_wy - center_wy) * world_size_pixels + screen_center.y);

      // Draw Fill
      if (!points.empty())
      {
        ImVec2 center_pt = ImVec2(cx, cy);
        ImU32 center_col = is_selected ? IM_COL32(0, 255, 0, 220) : IM_COL32(0, 220, 0, 180);

        if (m_show_rf_gradient && !m_view_cache[s_id].signal_dbm.empty())
        {
          // RF Gradient Mode: Use cached signal strength
          [[maybe_unused]] auto dbm_to_color = [](double p_rx_dbm) -> ImU32
          {
            int r, g, b, a;
            if (p_rx_dbm >= -60.0)
            {
              r = 0;
              g = 255;
              b = 0;
              a = 200;
            }
            else if (p_rx_dbm >= -80.0)
            {
              float t = (p_rx_dbm + 80.0) / 20.0;
              r = static_cast<int>((1.0f - t) * 255);
              g = 255;
              b = 0;
              a = 180;
            }
            else if (p_rx_dbm >= -100.0)
            {
              float t = (p_rx_dbm + 100.0) / 20.0;
              r = 255;
              g = static_cast<int>(t * 255);
              b = 0;
              a = 120;
            }
            else
            {
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

          for (size_t i = 0; i < points.size(); ++i)
          {
            ImU32 edge_col1 = dbm_to_color_lambda(cached_signal[i]);
            ImU32 edge_col2 = dbm_to_color_lambda(cached_signal[(i + 1) % points.size()]);

            draw_list->PrimReserve(3, 3);
            draw_list->PrimWriteVtx(center_pt, uv, center_col);
            draw_list->PrimWriteVtx(points[i], uv, edge_col1);
            draw_list->PrimWriteVtx(points[(i + 1) % points.size()], uv, edge_col2);
            draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 3));
            draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 2));
            draw_list->PrimWriteIdx((ImDrawIdx)(draw_list->_VtxCurrentIdx - 1));
          }
        }
        else
        {
          // Solid Fill Mode (default)
          for (size_t i = 0; i < points.size(); ++i)
          {
            draw_list->AddTriangleFilled(center_pt, points[i], points[(i + 1) % points.size()], fill_col);
          }
        }
      }

      // Draw Border
      draw_list->AddPolyline(points.data(), (int)points.size(), border_col, true, thickness);

      // Use solid color for marker
      ImU32 marker_col = IM_COL32(static_cast<int>(col[0] * 255), static_cast<int>(col[1] * 255), static_cast<int>(col[2] * 255), 255);
      draw_list->AddCircleFilled(ImVec2(cx, cy), 5.0f, marker_col);
      if (is_selected)
      {
        draw_list->AddCircle(ImVec2(cx, cy), 8.0f, IM_COL32(255, 255, 0, 255), 0, 2.0f);
      }

      // Hit Test for Selection
      // Check distance from cursor to center marker
      if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
      {
        float dx = io.MousePos.x - cx;
        float dy = io.MousePos.y - cy;
        // 10 pixel radius threshold (squared is 100)
        if (dx * dx + dy * dy < 100.0f)
        {
          selected_index = static_cast<int>(idx);
        }
      }
    }
  }

  // Draw Context Menu if Open
  if (ImGui::BeginPopup("map_context_menu"))
  {
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
      // Automatically enable analysis and hyperbolas for convenience
      m_show_hyperbolas = true;
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

  // Calculate mouse Lat/Lon
  ImVec2 mouse_pos = ImGui::GetMousePos();
  // Check if mouse is inside canvas
  if (ImGui::IsWindowHovered())
  {
    double mouse_offset_x = mouse_pos.x - screen_center.x;
    double mouse_offset_y = mouse_pos.y - screen_center.y;
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

  // Draw background
  draw_list->AddRectFilled(ImVec2(overlay_pos.x - padding, overlay_pos.y - padding), ImVec2(overlay_pos.x + text_size.x + padding, overlay_pos.y + text_size.y + padding), IM_COL32(255, 255, 255, 200), 4.0f);

  draw_list->AddText(overlay_pos, IM_COL32(0, 0, 0, 255), info_text.c_str());

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

      ImVec2 pA = ImVec2(px, py);

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

    draw_list->PopClipRect();

    // Draw Profile Window
    draw_path_profile_window(elevation_service);
  }
}

auto map_widget_t::set_tdoa_test_point(double lat, double lon) -> void
{
  m_has_test_point = true;
  m_test_point_lat = lat;
  m_test_point_lon = lon;

  // Force update if needed
}

auto map_widget_t::render_hyperbolas(const std::vector<sensor_t> &sensors, ImDrawList *draw_list, const ImVec2 &canvas_p0, const ImVec2 &canvas_sz) -> void
{
  if (sensors.size() < 2 || !m_tdoa_solver || !m_has_test_point)
    return;

  // Calculate TDOA for the test point
  auto test_tdoa = m_tdoa_solver->calculate_tdoa(sensors, m_test_point_lat, m_test_point_lon);

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
      auto points = m_tdoa_solver->sample_hyperbola(sensors[i], sensors[j], tdoa_ns, 200);

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
          gdop_grid[y * (cols + 1) + x] = m_tdoa_solver->calculate_gdop(sensors, lat, lon);
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
          errors_grid[y * (cols + 1) + x] = m_tdoa_solver->estimate_positioning_error(sensors, lat, lon, jitter);
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

  // --- Calculate Test Point Data ---
  if (sensors.size() >= 3 && m_tdoa_solver)
  {
    // 1. Calculate ideal TDOAs
    auto ideal_tdoa = m_tdoa_solver->calculate_tdoa(sensors, m_test_point_lat, m_test_point_lon);

    // 1b. Inject Synthetic Noise (Jitter)
    // To visualize the "Estimated" point wandering, we add random timing error.
    static std::mt19937 gen(12345); // Fixed seed for stability, or random_device
    // Actually, we want it to 'jitter' every frame? Or static?
    // If static, it won't move. If frame-based, it will jump wildly.
    // User probably wants to see *a* sample. Let's let it jitter every frame for now, or slow it down.
    // Fast jitter is good to see the "cloud" of probability.

    double jitter_ns = static_cast<double>(m_timing_jitter_ns);
    std::normal_distribution<double> d(0.0, jitter_ns);

    std::vector<double> noisy_tdoa = ideal_tdoa;
    for (size_t i = 1; i < noisy_tdoa.size(); ++i)
    {
      noisy_tdoa[i] += d(gen);
    }

    // 2. Solve (Verify Geometry) with NOISY data
    tdoa_result_t result = m_tdoa_solver->solve_position(sensors, noisy_tdoa, m_test_point_lat, m_test_point_lon);

    // 3. Update Error Estimate
    result.error_estimate_m = m_tdoa_solver->estimate_positioning_error(sensors, m_test_point_lat, m_test_point_lon, jitter_ns);

    // Store Result
    m_test_result = result;
  }
  else
  {
    m_test_result = tdoa_result_t{};
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

  // Draw Estimated Marker (Blue X) if valid
  if (sensors.size() >= 3 && m_test_result.converged)
  {
    ImVec2 p_est = latlon_to_screen(m_test_result.latitude, m_test_result.longitude);

    // Draw Error Vector (Yellow Dashed)
    draw_list->AddLine(p_truth, p_est, IM_COL32(255, 255, 0, 180), 2.0f);

    // Draw 'X'
    draw_list->AddLine(ImVec2(p_est.x - size * 0.7f, p_est.y - size * 0.7f), ImVec2(p_est.x + size * 0.7f, p_est.y + size * 0.7f), IM_COL32(0, 100, 255, 255), 3.0f);
    draw_list->AddLine(ImVec2(p_est.x + size * 0.7f, p_est.y - size * 0.7f), ImVec2(p_est.x - size * 0.7f, p_est.y + size * 0.7f), IM_COL32(0, 100, 255, 255), 3.0f);
    draw_list->AddText(ImVec2(p_est.x + size, p_est.y - size), IM_COL32(100, 200, 255, 255), "Est");
  }

} // function

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

} // namespace sensor_mapper
