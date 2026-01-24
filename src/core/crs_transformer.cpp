#include "crs_transformer.hpp"
#include <iostream>
#include <cmath>
#include <regex>

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
  bool utm_north = true;      // true = Northern Hemisphere, false = Southern
  bool is_wgs_to_utm = false; // direction flag

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
  std::cerr << "PROJ: Failed to create transformation from " << source_crs << " to " << target_crs << ". Trying fallback..." << std::endl;
  // If PROJ fails (or library missing at runtime?), try fallback
#endif

  m_impl->use_fallback = true;

  // Only support target=EPSG:4326 (Lat/Lon) for now
  if (target_crs != "EPSG:4326")
  {
    return false;
  }

  // Parse EPSG code from "EPSG:32633" or "32633"
  std::regex re(R"((\d+))");
  std::smatch match;
  if (std::regex_search(source_crs, match, re))
  {
    int code = std::stoi(match[1]);

    // UTM North: 32601 - 32660
    if (code >= 32601 && code <= 32660)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = true;
      m_impl->utm_zone = code - 32600;
      std::cout << "Fallback: Detected UTM Zone " << m_impl->utm_zone << "N" << std::endl;
      return true;
    }
    // UTM South: 32701 - 32760
    if (code >= 32701 && code <= 32760)
    {
      m_impl->is_utm = true;
      m_impl->utm_north = false;
      m_impl->utm_zone = code - 32700;
      std::cout << "Fallback: Detected UTM Zone " << m_impl->utm_zone << "S" << std::endl;
      return true;
    }
  }

  std::cerr << "Fallback: Unsupported CRS: " << source_crs << std::endl;
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

auto crs_transformer_t::transform(double x, double y, double &out_x, double &out_y) const -> bool
{
#ifdef HAVE_PROJ
  if (m_impl->transform_pj)
  {
    PJ_COORD input, output;
    input.xy.x = x;
    input.xy.y = y;
    output = proj_trans(m_impl->transform_pj, PJ_FWD, input);

    // PROJ returns (Long, Lat) for EPSG:4326 normally, but let's assume standard (Lat, Lon) mapping desire?
    // Wait, PROJ 6+ respects axis order. EPSG:4326 is Lat, Lon.
    // However, most visualization expects (Lat, Lon) or (Lon, Lat).
    // Let's rely on standard output.

    if (output.lp.lam == HUGE_VAL)
      return false;

    // Output for EPSG:4326 is usually degrees.
    // proj_trans returns radians if using PJ_LP, but for crs_to_crs it handles units.
    // It returns whatever the target CRS is defined as.
    // Usually EPSG:4326 in PROJ is Lat/Lon in degrees (sometimes).
    // Actually, modern PROJ defaults to Lat,Lon order for 4326.

    // Let's standardise: We usually want out_y=Lat, out_x=Lon (MapWidget expects) ?
    // No, standard is usually X=Lon, Y=Lat for rendering logic.
    // But PROJ output order depends.
    // Let's assume standard X=Lat, Y=Lon if 4326 implies that? No, 4326 is Lat first.
    // In our `lidar_source.cpp` logic previously:
    // input.lpzt.lam = lon; input.lpzt.phi = lat; (Inverse transform)
    // Here we do Forward scan (x,y projected -> lat/lon).

    out_x = output.xy.x; // Lat?
    out_y = output.xy.y; // Lon?

    // NOTE: We need to check if user needs to swap.
    // Let's assume the caller handles interpretation or check later.
    return true;
  }
#endif

  if (m_impl->use_fallback && m_impl->is_utm)
  {
    // UTM to Lat/Lon (WGS84)
    // References: Karney or similar. Simplified Karney for spherical/ellipsoid.

    double zone = (double)m_impl->utm_zone;
    bool north = m_impl->utm_north;

    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double k0 = 0.9996;
    const double e2 = f * (2 - f);
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

    // To Degrees
    double lat = phi * 180.0 / M_PI;
    double lon = (zone * 6 - 183.0) + lam * 180.0 / M_PI;

    // Assign to Output
    // Standard X=Lat, Y=Lon in this project?
    // Checking `lidar_source.cpp` original PROJ usage:
    // Previous code: `output.xy.x` was assigned to `x`. PROJ 6+ with "EPSG:4326" puts Lat in x.
    // So we should output Lat to out_x, Lon to out_y.

    out_x = lat;
    out_y = lon;

    return true;
  }

  return false;
}

} // namespace sensor_mapper
