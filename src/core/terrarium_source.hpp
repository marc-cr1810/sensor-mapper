#pragma once

#include "core/elevation_source.hpp"
#include <string>
#include <vector>
#include <map>
#include <future>
#include <mutex>

namespace sensor_mapper
{

struct elevation_tile_t
{
  int z, x, y;
  std::vector<float> heights;
  bool valid = false;
};

class terrarium_source_t : public elevation_source_t
{
public:
  terrarium_source_t();
  ~terrarium_source_t() override;

  auto get_elevation(double lat, double lon, float &out_height) -> bool override;
  auto update() -> void override;
  auto get_name() const -> const char * override
  {
    return "Terrarium Online";
  }

private:
  auto make_key(int z, int x, int y) -> std::string;
  auto fetch_tile(int z, int x, int y) -> void;

  struct pending_tile_t
  {
    int z, x, y;
    std::future<std::string> data_future;
  };

  std::map<std::string, std::shared_ptr<elevation_tile_t>> m_cache;
  std::vector<std::string> m_loading_keys;
  std::vector<pending_tile_t> m_pending;
  std::mutex m_mutex;
};

} // namespace sensor_mapper
