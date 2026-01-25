#pragma once

#include "sensor.hpp"
#include <cmath>
#include <algorithm>

namespace sensor_mapper
{
namespace rf_models
{

// Free Space Path Loss
inline double calculate_fspl(double d_km, double f_mhz)
{
  if (d_km <= 0.001)
    return 0.0;
  // FSPL(dB) = 20log10(d) + 20log10(f) + 32.44
  return 20.0 * std::log10(d_km) + 20.0 * std::log10(f_mhz) + 32.44;
}

// Okumura-Hata Model (Urban/Suburban/Rural)
inline double calculate_hata(double d_km, double f_mhz, double h_tx, double h_rx, PropagationModel model)
{
  if (d_km <= 0.001)
    return 0.0;

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
  else // Rural
  {
    return loss_urban - 4.78 * log_f * log_f + 18.33 * log_f - 40.94;
  }
}

// Two-Ray Ground Reflection Model
inline double calculate_two_ray(double d_km, double f_mhz, double h_tx, double h_rx)
{
  double d_m = d_km * 1000.0;
  if (d_m <= 1.0)
    return calculate_fspl(d_km, f_mhz);

  double lambda = 299.792458 / f_mhz;

  // Breakpoint distance d_bk = 4 * h_tx * h_rx / lambda
  double d_bk = (4.0 * h_tx * h_rx) / lambda;

  if (d_m < d_bk)
  {
    return calculate_fspl(d_km, f_mhz);
  }
  else
  {
    // Plane Earth Loss: L = 40log(d) - 20log(ht) - 20log(hr)
    // We clamp to FSPL to avoid predicting gain (optimistic constructive interference)
    // making the model conservative.
    double safe_h_tx = std::max(0.1, h_tx);
    double safe_h_rx = std::max(0.1, h_rx);
    double pel = 40.0 * std::log10(d_m) - 20.0 * std::log10(safe_h_tx) - 20.0 * std::log10(safe_h_rx);
    double fspl = calculate_fspl(d_km, f_mhz);
    return std::max(fspl, pel);
  }
}

// Unified Path Loss Calculation
inline double calculate_path_loss(double d_km, double f_mhz, double h_tx, double h_rx, PropagationModel model)
{
  switch (model)
  {
  case PropagationModel::FreeSpace:
    return calculate_fspl(d_km, f_mhz);
  case PropagationModel::HataUrban:
  case PropagationModel::HataSuburban:
  case PropagationModel::HataRural:
    return calculate_hata(d_km, f_mhz, h_tx, h_rx, model);
  case PropagationModel::TwoRay:
    return calculate_two_ray(d_km, f_mhz, h_tx, h_rx);
  default:
    return calculate_fspl(d_km, f_mhz);
  }
}

} // namespace rf_models
} // namespace sensor_mapper
