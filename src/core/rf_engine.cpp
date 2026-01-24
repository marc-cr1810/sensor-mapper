#include "rf_engine.hpp"
#include "geo_math.hpp"
#include "rasterizer_utils.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace sensor_mapper
{

rf_engine_t::rf_engine_t()
{
}
rf_engine_t::~rf_engine_t()
{
}

auto rf_engine_t::calculate_fspl(double d_km, double f_mhz) -> double
{
  if (d_km <= 0.001)
    return 0.0;
  return 20.0 * std::log10(d_km) + 20.0 * std::log10(f_mhz) + 32.44;
}

auto rf_engine_t::calculate_hata(double d_km, double f_mhz, double h_tx, double h_rx, PropagationModel model) -> double
{
  if (d_km <= 0.001)
    return 0.0;

  // Clamp values to valid ranges for Hata or allow extension
  double log_f = std::log10(f_mhz);
  double log_h_tx = std::log10(std::max(1.0, h_tx));
  double log_d = std::log10(d_km);

  // Correction factor a(h_rx) for small/medium city
  double a_h_rx = (1.1 * log_f - 0.7) * h_rx - (1.56 * log_f - 0.8);

  double loss_urban = 69.55 + 26.16 * log_f - 13.82 * log_h_tx - a_h_rx + (44.9 - 6.55 * log_h_tx) * log_d;

  if (model == PropagationModel::HataUrban)
  {
    return loss_urban;
  }
  else if (model == PropagationModel::HataSuburban)
  {
    double term = std::log10(f_mhz / 28.0);
    return loss_urban - 2.0 * term * term - 5.4;
  }
  else
  { // Rural
    return loss_urban - 4.78 * log_f * log_f + 18.33 * log_f - 40.94;
  }
}

auto rf_engine_t::compute_coverage(const std::vector<sensor_t> &sensors, elevation_service_t *elevation_service, const building_service_t *building_service, double min_lat, double max_lat, double min_lon, double max_lon, int width,
                                   int height) -> std::future<std::shared_ptr<coverage_grid_t>>
{

  // Copy sensors to avoid race conditions
  auto sensor_copy = sensors;

  return std::async(std::launch::async,
                    [=]()
                    {
                      auto grid = std::make_shared<coverage_grid_t>();
                      grid->width = width;
                      grid->height = height;
                      grid->min_lat = min_lat;
                      grid->max_lat = max_lat;
                      grid->min_lon = min_lon;
                      grid->max_lon = max_lon;
                      grid->signal_dbm.resize(width * height, -200.0f); // Init to noise

                      // Rasterize Buildings into Clutter Grid
                      std::vector<float> clutter_grid(width * height, 0.0f);
                      if (building_service)
                      {
                        auto buildings = building_service->get_buildings_in_area(min_lat, max_lat, min_lon, max_lon);
                        rasterize_buildings(clutter_grid, width, height, min_lat, max_lat, min_lon, max_lon, buildings, false); // max_mode
                      }

                      for (int y = 0; y < height; ++y)
                      {
                        double t = (double)y / (height - 1);
                        double cell_lat = min_lat + t * (max_lat - min_lat);

                        for (int x = 0; x < width; ++x)
                        {
                          double u = (double)x / (width - 1);
                          double cell_lon = min_lon + u * (max_lon - min_lon);

                          // Cache cell elevation locally if possible
                          float cell_elevation_m = 0.0f;
                          if (elevation_service)
                          {
                            elevation_service->get_elevation(cell_lat, cell_lon, cell_elevation_m);
                          }

                          double max_dbm = -200.0;

                          for (const auto &sensor : sensor_copy)
                          {
                            double dist_m = geo::distance(sensor.get_latitude(), sensor.get_longitude(), cell_lat, cell_lon);

                            if (dist_m > sensor.get_range())
                              continue;

                            double angle = geo::bearing(sensor.get_latitude(), sensor.get_longitude(), cell_lat, cell_lon);

                            // Phase 3: Antenna Pattern Gain
                            // get_antenna_gain returns relative gain (usually negative
                            // attenuation or 0)
                            double pattern_gain = sensor.get_antenna_gain(angle);

                            // Path Loss
                            double d_km = dist_m / 1000.0;
                            double path_loss = 0.0;
                            double h_tx = sensor.get_mast_height() + sensor.get_ground_elevation();
                            double h_rx = 2.0 + cell_elevation_m;

                            // Single Knife-Edge Diffraction (Fresnel)
                            double total_diffraction_loss = 0.0;

                            if (elevation_service)
                            {
                              // Search for the "dominant" obstacle (highest Fresnel parameter v)
                              double max_v = -10.0; // Start with a value indicating clear LoS
                              // float any_obstruction = false;

                              int steps = static_cast<int>(dist_m / 50.0); // 50m resolution
                              if (steps < 10)
                                steps = 10; // Minimum checks

                              for (int i = 1; i < steps; ++i)
                              {
                                double step_t = (double)i / steps;
                                double d1 = dist_m * step_t;
                                double d2 = dist_m * (1.0 - step_t);

                                // Point on the line (Great Circle approximated as line for lat/lon)
                                double p_lat = sensor.get_latitude() + step_t * (cell_lat - sensor.get_latitude());
                                double p_lon = sensor.get_longitude() + step_t * (cell_lon - sensor.get_longitude());

                                float p_elev = 0.0f;
                                elevation_service->get_elevation(p_lat, p_lon, p_elev);

                                // Heights relative to sea level, ADJUSTED for Earth Curvature (4/3 R)
                                // Effective height drops as we move away from both endpoints
                                // But here we model the "bulge" of the earth between points
                                // Or simpler: Flat earth with curved ray?
                                // Standard approach: Curved Earth, Straight Ray.
                                // h_obstacle_effective = h_terrain + h_curvature_bulge

                                // Height of the straight line ray at this point
                                double ray_h = h_tx + step_t * (h_rx - h_tx);

                                // Earth curvature "bulge" at distance d1 from TX and d2 from RX
                                // bulge = (d1 * d2) / (2 * k * R)
                                double bulge = (d1 * d2) / (2.0 * geo::EARTH_K_FACTOR * geo::EARTH_RADIUS);

                                // Effectively, the terrain is HIGHER by 'bulge' relative to the straight chord
                                double obstacle_h = p_elev + bulge;

                                // Clearance: Ray Height - Obstacle Height
                                // If clearance is negative, ray is blocked.
                                double clearance = ray_h - obstacle_h;

                                // Calculate Fresnel Parameter v for this point
                                // v is maximized when clearance is most negative (deepest obstruction)
                                // or least positive (closest pass)
                                double v = geo::fresnel_parameter(d1, d2, sensor.get_frequency_mhz(), -clearance);

                                if (v > max_v)
                                {
                                  max_v = v;
                                }
                              }

                              total_diffraction_loss = geo::knife_edge_loss(max_v);
                            }

                            if (sensor.get_propagation_model() == PropagationModel::FreeSpace)
                            {
                              path_loss = calculate_fspl(d_km, sensor.get_frequency_mhz());
                            }
                            else
                            {
                              // Hata requires effective height.
                              // h_tx is height above average terrain vs height above ground.
                              // Using Mast Height for now as per Hata spec relative to ground.
                              path_loss = calculate_hata(d_km, sensor.get_frequency_mhz(), sensor.get_mast_height(), 2.0, sensor.get_propagation_model());
                            }

                            double rx = sensor.get_tx_power_dbm() + sensor.get_tx_antenna_gain_dbi() + sensor.get_rx_antenna_gain_dbi() - path_loss + pattern_gain - total_diffraction_loss;

                            if (rx > max_dbm)
                              max_dbm = rx;
                          }

                          grid->signal_dbm[y * width + x] = static_cast<float>(max_dbm);
                        }
                      }
                      return grid;
                    });
}

auto rf_engine_t::calculate_fresnel_zone(double d1, double d2, double wavelength, int n) -> double
{
  // Rn = sqrt( (n * lambda * d1 * d2) / (d1 + d2) )
  return std::sqrt((n * wavelength * d1 * d2) / (d1 + d2));
}

} // namespace sensor_mapper
