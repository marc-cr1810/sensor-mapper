#include "core/tile_service.hpp"
#include "ui/texture.hpp"
#include <cpr/cpr.h>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>

namespace fs = std::filesystem;

namespace sensor_mapper
{

tile_service_t::tile_service_t()
{
}

tile_service_t::~tile_service_t()
{
}

auto tile_service_t::make_key(int z, int x, int y) -> tile_key_t
{
  return std::format("{}/{}/{}", z, x, y);
}

auto tile_service_t::set_source(tile_source_t source) -> void
{
  if (m_source != source)
  {
    m_source = source;
    // Clear in-memory cache
    m_cache.clear();
    m_pending.clear();
    m_loading_keys.clear();
  }
}

auto tile_service_t::get_source() const -> tile_source_t
{
  return m_source;
}

auto tile_service_t::get_tile(int z, int x, int y) -> std::shared_ptr<texture_t>
{
  auto key = make_key(z, x, y);

  // Check cache
  auto it = m_cache.find(key);
  if (it != m_cache.end())
  {
    return it->second;
  }

  // Check if already loading
  for (const auto &k : m_loading_keys)
  {
    if (k == key)
      return nullptr;
  }

  // Check disk cache
  fs::path cache_dir = ".cache/tiles";
  if (m_source == tile_source_t::OSM)
    cache_dir /= "osm";
  else if (m_source == tile_source_t::TERRARIUM)
    cache_dir /= "terrarium";
  else
    cache_dir /= "satellite";

  fs::path file_path = cache_dir / std::format("{}_{}_{}.png", z, x, y);

  // If exists on disk, load immediately
  if (fs::exists(file_path))
  {
    auto texture = std::make_shared<texture_t>();
    // Load raw bytes
    std::ifstream f(file_path, std::ios::binary);
    std::vector<unsigned char> buffer(std::istreambuf_iterator<char>(f), {});

    if (!buffer.empty())
    {
      if (texture->load_from_memory(buffer.data(), buffer.size()))
      {
        m_cache[key] = texture;
        return texture;
      }
    }
  }

  // Start fetch
  m_loading_keys.push_back(key);

  std::string url;
  if (m_source == tile_source_t::OSM)
  {
    url = std::format("https://tile.openstreetmap.org/{}/{}/{}.png", z, x, y);
  }
  else if (m_source == tile_source_t::TERRARIUM)
  {
    // AWS Terrain Tiles (Mapzen Terrarium)
    url = std::format("https://s3.amazonaws.com/elevation-tiles-prod/"
                      "terrarium/{}/{}/{}.png",
                      z, x, y);
  }
  else
  {
    // Esri World Imagery (Satellite)
    url = std::format("https://server.arcgisonline.com/ArcGIS/rest/services/"
                      "World_Imagery/MapServer/tile/{}/{}/{}",
                      z, y, x);
  }

  // Pass file_path to the async task to save it later
  std::string save_path = file_path.string();

  m_pending.push_back({z, x, y,
                       std::async(std::launch::async,
                                  [url, save_path]()
                                  {
                                    cpr::Response r = cpr::Get(cpr::Url{url}, cpr::Header{{"User-Agent", "SensorMapper/0.1"}});
                                    if (r.status_code == 200)
                                    {
                                      // Save to disk
                                      try
                                      {
                                        fs::path p(save_path);
                                        fs::create_directories(p.parent_path());
                                        std::ofstream f(p, std::ios::binary);
                                        f.write(r.text.data(), r.text.size());
                                      }
                                      catch (const std::exception &e)
                                      {
                                        std::cerr << "Tile download failed: " << e.what() << "\n";
                                      }
                                      catch (...)
                                      {
                                        std::cerr << "Tile download failed: Unknown error\n";
                                      }
                                      return r.text;
                                    }
                                    return std::string();
                                  })});

  return nullptr;
}

auto tile_service_t::update() -> void
{
  auto it = m_pending.begin();
  while (it != m_pending.end())
  {
    if (it->data_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      std::string data = it->data_future.get();
      auto key = make_key(it->z, it->x, it->y);

      // Remove from loading keys
      std::erase_if(m_loading_keys, [&](const auto &k) { return k == key; });

      if (!data.empty())
      {
        auto texture = std::make_shared<texture_t>();
        if (texture->load_from_memory(reinterpret_cast<const unsigned char *>(data.data()), data.size()))
        {
          m_cache[key] = texture;
        }
      }

      it = m_pending.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

} // namespace sensor_mapper
