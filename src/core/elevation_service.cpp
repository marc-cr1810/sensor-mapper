#include "core/elevation_service.hpp"
#include "core/geo_math.hpp"
#include "stb_image.h"
#include <cpr/cpr.h>
#include <format>
#include <iostream>

namespace sensor_mapper {

// Terrarium tiles are usually 256x256
constexpr int TILE_SIZE = 256;
// Standard zoom level for elevation lookups (balance between detail and data
// size)
constexpr int ELEVATION_ZOOM = 12;

elevation_service_t::elevation_service_t() {}
elevation_service_t::~elevation_service_t() {}

auto elevation_service_t::make_key(int z, int x, int y) -> tile_key_t {
  return std::format("{}/{}/{}", z, x, y);
}

auto elevation_service_t::get_elevation(double lat, double lon,
                                        float &out_height) -> bool {
  double wx, wy;
  geo::lat_lon_to_world(lat, lon, wx, wy);

  int tx, ty;
  geo::world_to_tile(wx, wy, ELEVATION_ZOOM, tx, ty);

  auto key = make_key(ELEVATION_ZOOM, tx, ty);
  auto it = m_cache.find(key);

  // If tile exists and is valid
  if (it != m_cache.end() && it->second->valid) {
    // Calculate sub-tile position
    double n = static_cast<double>(1 << ELEVATION_ZOOM);
    double tile_x_float = wx * n - tx;
    double tile_y_float = wy * n - ty;

    int px = static_cast<int>(tile_x_float * TILE_SIZE);
    int py = static_cast<int>(tile_y_float * TILE_SIZE);

    // Clamp
    if (px < 0)
      px = 0;
    if (px >= TILE_SIZE)
      px = TILE_SIZE - 1;
    if (py < 0)
      py = 0;
    if (py >= TILE_SIZE)
      py = TILE_SIZE - 1;

    int idx = py * TILE_SIZE + px;
    if (idx < it->second->heights.size()) {
      out_height = it->second->heights[idx];
      return true;
    }
  }

  // If not found or loading, ensure it's being fetched
  bool is_loading = false;
  for (const auto &k : m_loading_keys) {
    if (k == key) {
      is_loading = true;
      break;
    }
  }

  if (!is_loading) {
    fetch_tile(ELEVATION_ZOOM, tx, ty);
  }

  return false;
}

auto elevation_service_t::fetch_tile(int z, int x, int y) -> void {
  auto key = make_key(z, x, y);
  m_loading_keys.push_back(key);

  // Mapzen Terrarium format (hosted on AWS)
  // https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png
  std::string url = std::format(
      "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{}/{}/{}.png", z,
      x, y);

  m_pending.push_back({z, x, y, std::async(std::launch::async, [url]() {
                         cpr::Response r = cpr::Get(cpr::Url{url});
                         if (r.status_code == 200) {
                           return r.text;
                         }
                         return std::string();
                       })});
}

auto elevation_service_t::update() -> void {
  auto it = m_pending.begin();
  while (it != m_pending.end()) {
    if (it->data_future.wait_for(std::chrono::seconds(0)) ==
        std::future_status::ready) {
      std::string data = it->data_future.get();
      auto key = make_key(it->z, it->x, it->y);

      // Remove from loading
      std::erase_if(m_loading_keys, [&](const auto &k) { return k == key; });

      auto tile = std::make_shared<elevation_tile_t>();
      tile->z = it->z;
      tile->x = it->x;
      tile->y = it->y;

      if (!data.empty()) {
        int width, height, channels;
        // Decode PNG from memory
        unsigned char *img_data = stbi_load_from_memory(
            reinterpret_cast<const unsigned char *>(data.data()),
            static_cast<int>(data.size()), &width, &height, &channels, 0);

        if (img_data) {
          tile->heights.resize(width * height);
          for (int i = 0; i < width * height; i++) {
            // Mapzen Terrarium encoding:
            // meters = (r * 256 + g + b / 256) - 32768
            // channels usually 3 or 4
            int r = img_data[i * channels + 0];
            int g = img_data[i * channels + 1];
            int b = img_data[i * channels + 2];

            float h = (r * 256.0f + g + b / 256.0f) - 32768.0f;
            tile->heights[i] = h;
          }
          tile->valid = true;
          stbi_image_free(img_data);
        }
      }

      m_cache[key] = tile;
      it = m_pending.erase(it);
    } else {
      ++it;
    }
  }
}

} // namespace sensor_mapper
