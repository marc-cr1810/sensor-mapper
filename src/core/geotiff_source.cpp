#include "core/geotiff_source.hpp"
#include <tiffio.h>
#include <iostream>
#include <cmath>

namespace sensor_mapper
{

geotiff_source_t::geotiff_source_t(const std::filesystem::path &path) : m_path(path)
{
  load();
}

geotiff_source_t::~geotiff_source_t()
{
}

auto geotiff_source_t::load() -> bool
{
  TIFF *tif = TIFFOpen(m_path.string().c_str(), "r");
  if (!tif)
  {
    std::cerr << "Failed to open GeoTIFF: " << m_path << std::endl;
    return false;
  }

  uint32_t w, h;
  TIFFGetField(tif, TIFFTAG_IMAGEWIDTH, &w);
  TIFFGetField(tif, TIFFTAG_IMAGELENGTH, &h);

  m_width = w;
  m_height = h;

  // Read Georeferencing Tags
  // ModelPixelScaleTag (33550) = [ScaleX, ScaleY, ScaleZ]
  double *model_pixel_scale = nullptr;
  if (TIFFGetField(tif, 33550, &model_pixel_scale))
  {
    m_scale_x = model_pixel_scale[0];
    m_scale_y = model_pixel_scale[1]; // Typically negative?
  }

  // ModelTiepointTag (33922) = [I, J, K, X, Y, Z] (Mapping Raster 0,0 to World X,Y)
  double *model_tiepoint = nullptr;
  int count = 0;
  if (TIFFGetField(tif, 33922, &count, &model_tiepoint) && count >= 6)
  {
    m_origin_x = model_tiepoint[3];
    m_origin_y = model_tiepoint[4];
  }

  // Handling negative scale Y (common in GeoTIFF where origin is top-left)
  // If origin is top-left, and Y decreases downwards, m_scale_y might be positive in tag but logic implies subtraction?
  // Usually PixelScale is positive.
  // Raster Space: (0,0) top-left.
  // Model Space: OriginX, OriginY.
  // X = OriginX + pixel_x * ScaleX
  // Y = OriginY - pixel_y * ScaleY (Standard Northern Hemisphere images)
  // We'll assume standard top-left origin, Y decreases.

  // Read Strip Buffer
  m_data.resize(w * h);

  // Simple scanline reading (assuming float32 for DSM)
  float *buf = (float *)_TIFFmalloc(TIFFScanlineSize(tif));
  if (buf)
  {
    for (uint32_t row = 0; row < h; row++)
    {
      TIFFReadScanline(tif, buf, row);
      for (uint32_t col = 0; col < w; col++)
      {
        m_data[row * w + col] = buf[col];
      }
    }
    _TIFFfree(buf);
  }

  TIFFClose(tif);
  m_loaded = true;
  std::cout << "Loaded GeoTIFF: " << m_path << " (" << m_width << "x" << m_height << ")" << std::endl;
  return true;
}

auto geotiff_source_t::update() -> void
{
  // No async updates for now, loaded fully on start
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
    // Bilinear interpolation could be done here, nearest neighbor for now
    out_height = m_data[y * m_width + x];

    // Check for nodata (often -9999 or extremely low/high values)
    if (out_height < -10000.0f)
      return false;

    return true;
  }

  return false;
}

} // namespace sensor_mapper
