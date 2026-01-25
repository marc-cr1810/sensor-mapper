#include "../core/rf_models.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <algorithm>

using namespace sensor_mapper;

void test_fspl()
{
  std::cout << "Testing FSPL..." << std::endl;
  double loss = rf_models::calculate_fspl(1.0, 1000.0);
  std::cout << "  FSPL (1km, 1000MHz): " << loss << " dB (Expected ~92.44)" << std::endl;
  assert(std::abs(loss - 92.44) < 0.1);
}

void test_two_ray()
{
  std::cout << "Testing Two-Ray..." << std::endl;
  double f = 900.0;   // MHz
  double h_tx = 10.0; // m
  double h_rx = 2.0;  // m
  double lambda = 299.79 / f;
  double d_bk_m = (4.0 * h_tx * h_rx) / lambda;
  double d_bk_km = d_bk_m / 1000.0;

  std::cout << "  Breakpoint distance: " << d_bk_m << " m (" << d_bk_km << " km)" << std::endl;

  // Test Near (should be FSPL)
  double d_near_km = d_bk_km * 0.5;
  double loss_near = rf_models::calculate_two_ray(d_near_km, f, h_tx, h_rx);
  double fspl_near = rf_models::calculate_fspl(d_near_km, f);
  std::cout << "  Near (" << d_near_km << " km): TwoRay=" << loss_near << ", FSPL=" << fspl_near << std::endl;
  assert(std::abs(loss_near - fspl_near) < 0.001);

  // Test Far
  // We expect max(FSPL, PEL)
  double d_far_km = std::max(d_bk_km * 5.0, 5.0); // 5km to ensure significant divergence
  double loss_far = rf_models::calculate_two_ray(d_far_km, f, h_tx, h_rx);
  double fspl_far = rf_models::calculate_fspl(d_far_km, f);

  // Explicit Plane Earth Formula
  double d_far_m = d_far_km * 1000.0;
  double pel_far = 40.0 * std::log10(d_far_m) - 20.0 * std::log10(h_tx) - 20.0 * std::log10(h_rx);

  std::cout << "  Far (" << d_far_km << " km):" << std::endl;
  std::cout << "    TwoRay Output: " << loss_far << std::endl;
  std::cout << "    FSPL Reference: " << fspl_far << std::endl;
  std::cout << "    PEL Reference : " << pel_far << std::endl;

  double expected = std::max(fspl_far, pel_far);
  assert(std::abs(loss_far - expected) < 0.001);

  // Verify consistency: Loss should generally Increase with distance
  // And for Far field, PEL (slope 40) should eventually exceed FSPL (slope 20).
  // Let's check if PEL > FSPL at 5km?
  // 5km = 5000m.
  // FSPL ~ 20log(5) + 85 (at 1km) + ...
  // Let's rely on the assertion that output matches max(FSPL, PEL).
}

int main()
{
  test_fspl();
  test_two_ray();
  std::cout << "RF Models Verification Passed" << std::endl;
  return 0;
}
