#include "crs_transformer.hpp"
#include <iostream>
#include <cmath>
#include <regex>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifdef HAVE_PROJ
#include <proj.h>
#endif

namespace sensor_mapper
{

struct crs_transformer_t::impl_t
{
#ifdef HAVE_PROJ
  PJ_CONTEXT *proj_ctx = nullptr;
  PJ *transform_pj = nullptr;
#endif

  // Fallback UTM parameters
  bool use_fallback = false;
  bool is_utm = false;
  int utm_zone = 0;
  bool utm_north = true; // true = Northern Hemisphere, false = Southern

  // WGS84 Constants
  static constexpr double A = 6378137.0;
  static constexpr double F = 1 / 298.257223563;
  static constexpr double K0 = 0.9996;

  ~impl_t()
  {
#ifdef HAVE_PROJ
    if (transform_pj)
      proj_destroy(transform_pj);
    if (proj_ctx)
      proj_context_destroy(proj_ctx);
#endif
  }
};

crs_transformer_t::crs_transformer_t() : m_impl(std::make_unique<impl_t>())
{
}

crs_transformer_t::~crs_transformer_t() = default;

auto crs_transformer_t::init(const std::string &source_crs, const std::string &target_crs) -> bool
{
#ifdef HAVE_PROJ
  m_impl->proj_ctx = proj_context_create();
  m_impl->transform_pj = proj_create_crs_to_crs(m_impl->proj_ctx, source_crs.c_str(), target_crs.c_str(), nullptr);

  if (m_impl->transform_pj)
  {
    return true;
  }
#endif

  m_impl->use_fallback = true;

  if (target_crs != "EPSG:4326")
  {
    return false;
  }

  std::regex re(R"((\d+))");
  std::smatch match;
  if (std::regex_search(source_crs, match, re))
  {
    int code = std::stoi(match[1]);

    if (code >= 32601 && code <= 32660)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = true;
      m_impl->utm_zone = code - 32600;
      return true;
    }
    if (code >= 32701 && code <= 32760)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = false;
      m_impl->utm_zone = code - 32700;
      return true;
    }
    // GDA2020 / MGA Zones 47-59 (Australia): 7847 - 7859
    if (code >= 7847 && code <= 7859)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = false;
      m_impl->utm_zone = code - 7800;
      return true;
    }
    // GDA94 / MGA Zones 47-59 (Older Australia): 28347 - 28359
    if (code >= 28347 && code <= 28359)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = false;
      m_impl->utm_zone = code - 28300;
      return true;
    }
  }

  return false;
}

auto crs_transformer_t::is_valid() const -> bool
{
#ifdef HAVE_PROJ
  if (m_impl->transform_pj)
    return true;
#endif
  return m_impl->use_fallback && m_impl->is_utm;
}

auto crs_transformer_t::transform(double x, double y, double &out_x, double &out_y, bool forward) const -> bool
{
#ifdef HAVE_PROJ
  if (m_impl->transform_pj)
  {
    PJ_COORD input, output;
    input.xy.x = x;
    input.xy.y = y;
    output = proj_trans(m_impl->transform_pj, forward ? PJ_FWD : PJ_INV, input);

    if (output.lp.lam == HUGE_VAL)
      return false;

    out_x = output.xy.x;
    out_y = output.xy.y;
    return true;
  }
#endif

  if (m_impl->use_fallback && m_impl->is_utm)
  {
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double k0 = 0.9996;
    const double e2 = f * (2 - f);

    if (forward) // UTM -> Lat/Lon
    {
      double zone = (double)m_impl->utm_zone;
      bool north = m_impl->utm_north;
      const double e4 = e2 * e2;
      const double e6 = e4 * e2;
      const double e1 = (1 - std::sqrt(1 - e2)) / (1 + std::sqrt(1 - e2));

      double xi = (y - (north ? 0.0 : 10000000.0)) / k0;
      double eta = (x - 500000.0) / k0;

      double M = xi;
      double mu = M / (a * (1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256));

      double phi1 = mu + (3 * e1 / 2 - 27 * e1 * e1 * e1 / 32) * std::sin(2 * mu) + (21 * e1 * e1 / 16 - 55 * e1 * e1 * e1 * e1 / 32) * std::sin(4 * mu) + (151 * e1 * e1 * e1 / 96) * std::sin(6 * mu);

      double N1 = a / std::sqrt(1 - e2 * std::sin(phi1) * std::sin(phi1));
      double T1 = std::tan(phi1) * std::tan(phi1);
      double C1 = e2 * std::cos(phi1) * std::cos(phi1) / (1 - e2);
      double R1 = a * (1 - e2) / std::pow(1 - e2 * std::sin(phi1) * std::sin(phi1), 1.5);
      double D = eta / N1;

      double phi = phi1 - (N1 * std::tan(phi1) / R1) * (D * D / 2 - (5 + 3 * T1 + 10 * C1 - 4 * C1 * C1 - 9 * e2) * D * D * D * D / 24 + (61 + 90 * T1 + 298 * C1 + 45 * T1 * T1 - 252 * e2 - 3 * C1 * C1) * D * D * D * D * D * D / 720);
      double lam = (D - (1 + 2 * T1 + C1) * D * D * D / 6 + (5 - 2 * C1 + 28 * T1 - 3 * C1 * C1 + 8 * e2 + 24 * T1 * T1) * D * D * D * D * D / 120) / std::cos(phi1);

      out_x = phi * 180.0 / M_PI;
      out_y = (zone * 6 - 183.0) + lam * 180.0 / M_PI;
      return true;
    }
    else // Lat/Lon -> UTM
    {
      // Basic Transverse Mercator (Snyder)
      double lat_rad = x * M_PI / 180.0;
      double lon_rad = y * M_PI / 180.0;
      double zone_cm = (m_impl->utm_zone * 6 - 183.0) * M_PI / 180.0;

      double N = a / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad));
      double T = std::tan(lat_rad) * std::tan(lat_rad);
      double C = e2 * std::cos(lat_rad) * std::cos(lat_rad) / (1 - e2);
      double A_val = (lon_rad - zone_cm) * std::cos(lat_rad);

      double M = a * ((1 - e2 / 4 - 3 * e2 * e2 / 64 - 5 * e2 * e2 * e2 / 256) * lat_rad - (3 * e2 / 8 + 3 * e2 * e2 / 32 + 45 * e2 * e2 * e2 / 1024) * std::sin(2 * lat_rad) +
                      (15 * e2 * e2 / 256 + 45 * e2 * e2 * e2 / 1024) * std::sin(4 * lat_rad) - (35 * e2 * e2 * e2 / 3072) * std::sin(6 * lat_rad));

      out_x = k0 * N * (A_val + (1 - T + C) * A_val * A_val * A_val / 6.0 + (5 - 18 * T + T * T + 72 * C - 58 * e2) * A_val * A_val * A_val * A_val * A_val / 120.0) + 500000.0;

      out_y =
          k0 * (M + N * std::tan(lat_rad) * (A_val * A_val / 2.0 + (5 - T + 9 * C + 4 * C * C) * A_val * A_val * A_val * A_val / 24.0 + (61 - 58 * T + T * T + 600 * C - 330 * e2) * A_val * A_val * A_val * A_val * A_val * A_val / 720.0));

      if (!m_impl->utm_north)
        out_y += 10000000.0;
      return true;
    }
  }

  return false;
}

} // namespace sensor_mapper
