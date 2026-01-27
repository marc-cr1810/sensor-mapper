#pragma once

#include <vector>
#include <string>
#include <optional>

namespace sensor_mapper
{

struct log_point_t
{
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;      // Altitude (meters), often MSL
  double time_sec = 0.0; // Relative time in seconds from start
  bool has_time = false;
  bool has_alt = false;
};

class LogParser
{
public:
  static std::vector<log_point_t> parse_gpx(const std::string &path);
  static std::vector<log_point_t> parse_csv(const std::string &path);

private:
  static double parse_iso8601_to_sec(const std::string &time_str);
};

} // namespace sensor_mapper
