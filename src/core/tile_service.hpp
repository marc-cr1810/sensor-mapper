#pragma once

#include "ui/texture.hpp"
#include <future>
#include <map>
#include <memory>
#include <string>

namespace sensor_mapper {

// Key for tile cache: "z/x/y"
using tile_key_t = std::string;

class tile_service_t {
public:
  enum class tile_source_t { OSM, TERRARIUM };

  tile_service_t();
  ~tile_service_t();

  auto set_source(tile_source_t source) -> void;
  auto get_source() const -> tile_source_t;

  // Request a tile. Returns pointer to texture if loaded, nullptr if
  // pending/loading. If not in cache, triggers a fetch.
  auto get_tile(int z, int x, int y) -> std::shared_ptr<texture_t>;

  // Call once per frame to process loaded tiles into textures
  // (OpenGL texture creation must happen on main thread)
  auto update() -> void;

private:
  struct pending_tile_t {
    int z, x, y;
    std::future<std::string> data_future;
  };

  std::map<tile_key_t, std::shared_ptr<texture_t>> m_cache;
  std::vector<pending_tile_t> m_pending;
  std::vector<tile_key_t> m_loading_keys; // to avoid duplicate requests
  tile_source_t m_source = tile_source_t::OSM;

  auto make_key(int z, int x, int y) -> tile_key_t;
};

} // namespace sensor_mapper
