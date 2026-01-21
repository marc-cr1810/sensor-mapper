#pragma once

#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sensor_mapper {

// Stores raw elevation data for a tile (256x256 typically)
struct elevation_tile_t {
  int z, x, y;
  std::vector<float> heights; // Row-major, 256x256
  bool valid = false;
};

class elevation_service_t {
public:
  elevation_service_t();
  ~elevation_service_t();

  // Get elevation in meters at specific latitude/longitude
  // Returns false if data is not yet available (and triggers fetch)
  auto get_elevation(double lat, double lon, float &out_height) -> bool;

  // Process pending network requests
  auto update() -> void;

private:
  using tile_key_t = std::string;

  struct pending_fetch_t {
    int z, x, y;
    std::future<std::string> data_future;
  };

  std::map<tile_key_t, std::shared_ptr<elevation_tile_t>> m_cache;
  std::vector<pending_fetch_t> m_pending;
  std::vector<tile_key_t> m_loading_keys;

  auto make_key(int z, int x, int y) -> tile_key_t;
  auto fetch_tile(int z, int x, int y) -> void;
};

} // namespace sensor_mapper
