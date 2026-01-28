#include "rf_engine.hpp"
#include "rf_models.hpp"
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
                          bool has_elevation = false;

                          double max_dbm = -200.0;

                          for (const auto &sensor : sensor_copy)
                          {
                            double dist_m = geo::distance(sensor.get_latitude(), sensor.get_longitude(), cell_lat, cell_lon);

                            if (dist_m > sensor.get_range())
                              continue;

                            if (!has_elevation && elevation_service)
                            {
                              elevation_service->get_elevation(cell_lat, cell_lon, cell_elevation_m);
                              has_elevation = true;
                            }

                            double angle = geo::bearing(sensor.get_latitude(), sensor.get_longitude(), cell_lat, cell_lon);

                            // Phase 3: Antenna Pattern Gain
                            // get_antenna_gain returns relative gain (usually negative
                            // attenuation or 0)
                            double pattern_gain = sensor.get_antenna_gain(angle);

                            // Path Loss
                            double d_km = dist_m / 1000.0;
                            double h_tx_amsl = sensor.get_mast_height() + sensor.get_ground_elevation();
                            double h_rx_amsl = 2.0 + cell_elevation_m;
                            double h_tx = sensor.get_mast_height(); // AGL for models
                            double h_rx = 2.0;                      // AGL for models (assuming flat earth model for propagation relative to ground)

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
                                // Standard approach: Curved Earth, Straight Ray.
                                // h_obstacle_effective = h_terrain + h_curvature_bulge

                                // Height of the straight line ray at this point
                                double ray_h = h_tx_amsl + step_t * (h_rx_amsl - h_tx_amsl);

                                // Earth curvature "bulge" at distance d1 from TX and d2 from RX
                                // bulge = (d1 * d2) / (2 * k * R)
                                double bulge = (d1 * d2) / (2.0 * geo::EARTH_K_FACTOR * geo::EARTH_RADIUS);

                                // Effectively, the terrain is HIGHER by 'bulge' relative to the straight chord
                                double obstacle_h = p_elev + bulge;

                                // Clearance: Ray Height - Obstacle Height
                                // If clearance is negative, ray is blocked.
                                double clearance = ray_h - obstacle_h;

                                // Calculate Fresnel Parameter v for this point
                                double v = geo::fresnel_parameter(d1, d2, sensor.get_frequency_mhz(), -clearance);

                                if (v > max_v)
                                {
                                  max_v = v;
                                }
                              }

                              total_diffraction_loss = geo::knife_edge_loss(max_v);
                            }

                            double path_loss = rf_models::calculate_path_loss(d_km, sensor.get_frequency_mhz(), h_tx, h_rx, sensor.get_propagation_model());

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
