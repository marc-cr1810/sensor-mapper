#pragma once

#include "building_service.hpp"
#include "geo_math.hpp"
#include <algorithm>
#include <cmath>
#include <vector>

namespace sensor_mapper
{

// Rasterize buildings into a float grid (DSM)
// grid: Flat array of size width * height (initialized to terrain or 0)
// bounds: min_lat, max_lat, min_lon, max_lon
// add_mode: If true, adds height to existing value. If false, takes max(existing, height).
inline void rasterize_buildings(std::vector<float> &grid, int width, int height, double min_lat, double max_lat, double min_lon, double max_lon, const std::vector<const building_t *> &buildings, bool add_mode = true)
{
  for (const auto *b : buildings)
  {
    if (!b || b->footprint.empty())
      continue;

    // Convert polygon to grid coordinates
    std::vector<std::pair<int, int>> polygon_px;
    int min_y = height, max_y = 0;

    for (const auto &pt : b->footprint)
    {
      double u = (pt.lon - min_lon) / (max_lon - min_lon);
      double v = (pt.lat - min_lat) / (max_lat - min_lat); // 0 at min_lat, 1 at max_lat

      int x = static_cast<int>(u * (width - 1));
      int y = static_cast<int>(v * (height - 1));

      // Clamp
      x = std::max(0, std::min(width - 1, x));
      y = std::max(0, std::min(height - 1, y));

      polygon_px.push_back({x, y});
      min_y = std::min(min_y, y);
      max_y = std::max(max_y, y);
    }

    // Scanline Rasterization
    for (int y = min_y; y <= max_y; ++y)
    {
      std::vector<int> node_x;
      size_t n = polygon_px.size();
      for (size_t i = 0; i < n; ++i)
      {
        size_t j = (i + 1) % n;
        int x1 = polygon_px[i].first;
        int y1 = polygon_px[i].second;
        int x2 = polygon_px[j].first;
        int y2 = polygon_px[j].second;

        if ((y1 < y && y2 >= y) || (y2 < y && y1 >= y))
        {
          node_x.push_back(x1 + (y - y1) * (x2 - x1) / (y2 - y1));
        }
      }
      std::sort(node_x.begin(), node_x.end());

      for (size_t i = 0; i < node_x.size(); i += 2)
      {
        if (i + 1 >= node_x.size())
          break;
        int start_x = node_x[i];
        int end_x = node_x[i + 1];

        if (start_x >= width)
          continue;
        if (end_x < 0)
          continue;
        start_x = std::max(0, start_x);
        end_x = std::min(width - 1, end_x);

        for (int x = start_x; x <= end_x; ++x)
        {
          int idx = y * width + x;
          if (add_mode)
          {
            grid[idx] += static_cast<float>(b->height_m);
          }
          else
          {
            grid[idx] = std::max(grid[idx], static_cast<float>(b->height_m));
          }
        }
      }
    }
  }
}

} // namespace sensor_mapper
