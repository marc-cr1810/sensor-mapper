#include "core/geotiff_source.hpp"
#include <tiffio.h>
#include <iostream>
#include <cmath>

namespace sensor_mapper
{

auto geotiff_source_t::load() -> bool
{
  std::cout << "GeoTIFF: Starting load for " << m_path << std::endl;

  // Suppress warnings for GeoTIFF tags (they're expected and harmless)
  TIFFSetWarningHandler(nullptr);

  std::cout << "GeoTIFF: Opening file..." << std::endl;
  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
  {
    std::cerr << "Failed to open GeoTIFF: " << m_path << std::endl;
    return false;
  }
  std::cout << "GeoTIFF: File opened successfully" << std::endl;

  uint32_t w, h;
  if (!TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w) || !TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h))
  {
    std::cerr << "Failed to read GeoTIFF dimensions: " << m_path << std::endl;
    TIFFClose(tif);
    return false;
  }
  std::cout << "GeoTIFF: Dimensions: " << w << "x" << h << std::endl;

  m_width = w;
  m_height = h;

  // Read Georeferencing Tags - use defaults if not present
  std::cout << "GeoTIFF: Reading georeferencing tags..." << std::endl;

  // Set safe defaults first
  m_scale_x = 1.0;
  m_scale_y = 1.0;
  m_origin_x = 0.0;
  m_origin_y = 0.0;

  // Read GeoTIFF tags safely using raw data access
  std::cout << "GeoTIFF: Reading georeferencing tags..." << std::endl;

  // Try reading ModelPixelScale (tag 33550) as raw data
  std::cout << "GeoTIFF: Attempting to read ModelPixelScale..." << std::endl;
  uint32_t scale_count = 0;
  void *scale_raw = nullptr;

  // Get the tag as raw data
  if (TIFFGetField(tif, 33550, &scale_count, &scale_raw) && scale_count >= 3 && scale_raw)
  {
    // It's an array of doubles
    double *scales = (double *)scale_raw;
    m_scale_x = scales[0];
    m_scale_y = scales[1];
    std::cout << "GeoTIFF: Scale: " << m_scale_x << ", " << m_scale_y << std::endl;
  }
  else
  {
    std::cout << "GeoTIFF: No ModelPixelScale, using defaults (1.0, 1.0)" << std::endl;
  }

  // Try reading ModelTiepoint (tag 33922) as raw data
  std::cout << "GeoTIFF: Attempting to read ModelTiepoint..." << std::endl;
  uint32_t tiepoint_count = 0;
  void *tiepoint_raw = nullptr;

  if (TIFFGetField(tif, 33922, &tiepoint_count, &tiepoint_raw) && tiepoint_count >= 6 && tiepoint_raw)
  {
    // It's an array of doubles [I, J, K, X, Y, Z]
    double *tiepoints = (double *)tiepoint_raw;
    m_origin_x = tiepoints[3]; // X
    m_origin_y = tiepoints[4]; // Y
    std::cout << "GeoTIFF: Origin: " << m_origin_x << ", " << m_origin_y << std::endl;
  }
  else
  {
    std::cout << "GeoTIFF: No ModelTiepoint, using defaults (0.0, 0.0)" << std::endl;
  }

  // Don't keep file open across threads - close it after reading metadata
  // We'll reopen it on-demand for better thread safety
  std::cout << "GeoTIFF: Closing file..." << std::endl;
  TIFFClose(tif);

  std::cout << "GeoTIFF: Setting m_loaded = true..." << std::endl;
  m_loaded = true;
  std::cout << "Loaded GeoTIFF: " << m_path << " (" << m_width << "x" << m_height << ") [on-demand mode]" << std::endl;
  return true;
}

geotiff_source_t::geotiff_source_t(const std::filesystem::path &path) : m_path(path)
{
  // TEMPORARY: Synchronous load for debugging
  load();

  // Async load (disabled for debugging)
  // m_load_future = std::async(std::launch::async, [this]() { return load(); });
  // m_loading_started = true;
}

geotiff_source_t::~geotiff_source_t()
{
  if (m_loading_started && m_load_future.valid())
  {
    m_load_future.wait();
  }

  // Close TIFF file if open
  if (m_tif)
  {
    TIFFClose(static_cast<TIFF *>(m_tif));
    m_tif = nullptr;
  }

  // Clear cache
  m_scanline_cache.clear();
}

auto geotiff_source_t::update() -> void
{
  if (!m_loaded && m_loading_started && m_load_future.valid())
  {
    if (m_load_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
      bool success = m_load_future.get();
      if (success)
      {
        m_loaded = true;
      }
      else
      {
        std::cerr << "GeoTIFF: Async load failed for " << m_path << std::endl;
        m_loading_started = false;
      }
    }
  }
}

auto geotiff_source_t::get_bounds(double &min_lat, double &max_lat, double &min_lon, double &max_lon) const -> bool
{
  if (!m_loaded)
    return false;

  // Get corners in projected coordinates
  double x0 = m_origin_x;
  double y0 = m_origin_y;
  double x1 = m_origin_x + m_width * m_scale_x;
  double y1 = m_origin_y - m_height * m_scale_y;

  // Simple UTM Zone 55 to Lat/Lon conversion
  // This is approximate but works for visualization
  const double k0 = 0.9996;
  const double e = 0.00669438;
  const double e2 = e * e;
  const double e3 = e2 * e;
  const double e_p2 = e / (1.0 - e);
  const double sqrt_e = sqrt(1 - e);
  const double _e = (1 - sqrt_e) / (1 + sqrt_e);
  const double _e2 = _e * _e;
  const double _e3 = _e2 * _e;
  const double _e4 = _e3 * _e;
  const double _e5 = _e4 * _e;
  const double M1 = (1 - e / 4 - 3 * e2 / 64 - 5 * e3 / 256);
  const double M2 = (3 * e / 8 + 3 * e2 / 32 + 45 * e3 / 1024);
  const double M3 = (15 * e2 / 256 + 45 * e3 / 1024);
  const double M4 = (35 * e3 / 3072);
  const double a = 6378137.0;
  const double b = 6356752.314245;

  auto utm_to_latlon = [&](double easting, double northing) -> std::pair<double, double>
  {
    double x = easting - 500000.0;
    // Southern Hemisphere: subtract from false northing
    double y = northing - 10000000.0; // MGA Zone 55 is Southern Hemisphere

    double m = y / k0;
    double mu = m / (a * M1);

    double phi1 = mu + M2 * sin(2 * mu) / M1 + M3 * sin(4 * mu) / M1 + M4 * sin(6 * mu) / M1;

    double n = a / sqrt(1 - e * sin(phi1) * sin(phi1));
    double t = tan(phi1) * tan(phi1);
    double c = e_p2 * cos(phi1) * cos(phi1);
    double a_val = x / (n * k0);
    double a2 = a_val * a_val;
    double a3 = a2 * a_val;
    double a4 = a3 * a_val;
    double a5 = a4 * a_val;
    double a6 = a5 * a_val;

    double lat_rad = phi1 - (n * tan(phi1) / n) * (a2 / 2 - (5 + 3 * t + 10 * c - 4 * c * c - 9 * e_p2) * a4 / 24 + (61 + 90 * t + 298 * c + 45 * t * t - 252 * e_p2 - 3 * c * c) * a6 / 720);

    double lon_rad = (a_val - (1 + 2 * t + c) * a3 / 6 + (5 - 2 * c + 28 * t - 3 * c * c + 8 * e_p2 + 24 * t * t) * a5 / 120) / cos(phi1);

    // Zone 55 central meridian
    lon_rad += (55 * 6 - 183) * M_PI / 180.0;

    return {lat_rad * 180.0 / M_PI, lon_rad * 180.0 / M_PI};
  };

  auto [lat0, lon0] = utm_to_latlon(x0, y0);
  auto [lat1, lon1] = utm_to_latlon(x1, y1);

  min_lat = std::min(lat0, lat1);
  max_lat = std::max(lat0, lat1);
  min_lon = std::min(lon0, lon1);
  max_lon = std::max(lon0, lon1);

  std::cout << "GeoTIFF Bounds (UTM): (" << x0 << ", " << y0 << ") to (" << x1 << ", " << y1 << ")" << std::endl;
  std::cout << "GeoTIFF Bounds (Lat/Lon): (" << min_lat << ", " << min_lon << ") to (" << max_lat << ", " << max_lon << ")" << std::endl;

  return true;
}

auto geotiff_source_t::get_scanline(int row) -> const float *
{
  std::lock_guard<std::mutex> lock(m_file_mutex);

  if (row < 0 || row >= m_height)
    return nullptr;

  // Check cache first
  for (auto &entry : m_scanline_cache)
  {
    if (entry.row == row)
    {
      // Update access time
      entry.last_access = std::chrono::steady_clock::now();
      return entry.data.data();
    }
  }

  // Not in cache - open file fresh for thread safety
  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
    return nullptr;

  std::vector<float> scanline(m_width);

  // Allocate buffer for reading
  float *buf = (float *)_TIFFmalloc(TIFFScanlineSize(tif));
  if (!buf)
  {
    TIFFClose(tif);
    return nullptr;
  }

  // Read scanline
  if (TIFFReadScanline(tif, buf, row) < 0)
  {
    _TIFFfree(buf);
    TIFFClose(tif);
    return nullptr;
  }

  // Copy to our vector
  for (int i = 0; i < m_width; ++i)
  {
    scanline[i] = buf[i];
  }
  _TIFFfree(buf);
  TIFFClose(tif); // Close immediately after reading

  // Add to cache (evict oldest if full)
  if (m_scanline_cache.size() >= MAX_CACHED_SCANLINES)
  {
    // Find oldest entry
    auto oldest = std::min_element(m_scanline_cache.begin(), m_scanline_cache.end(), [](const scanline_cache_entry_t &a, const scanline_cache_entry_t &b) { return a.last_access < b.last_access; });

    // Replace oldest
    oldest->row = row;
    oldest->data = std::move(scanline);
    oldest->last_access = std::chrono::steady_clock::now();
    return oldest->data.data();
  }
  else
  {
    // Add new entry
    m_scanline_cache.push_back({row, std::move(scanline), std::chrono::steady_clock::now()});
    return m_scanline_cache.back().data.data();
  }
}

auto geotiff_source_t::get_elevation(double lat, double lon, float &out_height) -> bool
{
  if (!m_loaded)
    return false;

  // Inverse mapping: World -> Raster
  // pixel_x = (lon - OriginX) / ScaleX
  // pixel_y = (OriginY - lat) / ScaleY  (Assuming Top-Left origin, Y decreases down)

  double px = (lon - m_origin_x) / m_scale_x;
  double py = (m_origin_y - lat) / m_scale_y;

  int x = static_cast<int>(std::floor(px));
  int y = static_cast<int>(std::floor(py));

  if (x >= 0 && x < m_width && y >= 0 && y < m_height)
  {
    // Get scanline (from cache or disk)
    const float *row_data = get_scanline(y);
    if (!row_data)
      return false;

    // Read pixel value
    out_height = row_data[x];

    // Check for nodata (often -9999 or extremely low/high values)
    if (out_height < -10000.0f)
      return false;

    return true;
  }

  return false;
}

} // namespace sensor_mapper
