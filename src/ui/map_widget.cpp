#include "ui/map_widget.hpp"
#include "core/geo_math.hpp"
#include <cmath>
#include <string>
#include <vector>

namespace sensor_mapper {

map_widget_t::map_widget_t()
    : m_center_lat(-33.8688) // Sydney
      ,
      m_center_lon(151.2093), m_zoom(10.0),
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

auto map_widget_t::get_zoom() const -> double { return m_zoom; }

auto map_widget_t::draw(const std::vector<sensor_t> &sensors,
                        int selected_index,
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
    set_zoom(m_zoom - 1.0);

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

    // Calculate approximation in degrees
    constexpr double METERS_PER_DEG_LAT = 111132.0;
    double meters_per_deg_lon = 111132.0 * std::cos(s_lat * geo::PI / 180.0);
    if (meters_per_deg_lon < 1.0)
      meters_per_deg_lon = 1.0;

    double radius_deg_lat = range_m / METERS_PER_DEG_LAT;
    double radius_deg_lon = range_m / meters_per_deg_lon;

    const int segments = 64;
    std::vector<ImVec2> points;
    points.reserve(segments);

    for (int i = 0; i < segments; ++i) {
      double angle = (2.0 * geo::PI * i) / segments;
      double d_lat = radius_deg_lat * std::sin(angle);
      double d_lon = radius_deg_lon * std::cos(angle);

      double p_lat = s_lat + d_lat;
      double p_lon = s_lon + d_lon;

      double p_wx, p_wy;
      geo::lat_lon_to_world(p_lat, p_lon, p_wx, p_wy);

      // Simple Wrap check logic for drawing
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

    // Colors
    ImU32 fill_col =
        is_selected ? IM_COL32(0, 255, 0, 100) : IM_COL32(0, 200, 0, 50);
    ImU32 border_col =
        is_selected ? IM_COL32(255, 255, 0, 255) : IM_COL32(0, 255, 0, 255);
    float thickness = is_selected ? 3.0f : 2.0f;

    draw_list->AddConvexPolyFilled(points.data(), segments, fill_col);
    draw_list->AddPolyline(points.data(), segments, border_col, true,
                           thickness);

    // Draw center marker
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

    draw_list->AddCircleFilled(ImVec2(cx, cy), 5.0f, IM_COL32(255, 0, 0, 255));
    if (is_selected) {
      draw_list->AddCircle(ImVec2(cx, cy), 8.0f, IM_COL32(255, 255, 0, 255), 0,
                           2.0f);
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
