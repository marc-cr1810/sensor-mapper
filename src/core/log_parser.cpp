#include "log_parser.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <regex>
#include <ctime>
#include <iomanip>

namespace sensor_mapper
{

std::vector<log_point_t> LogParser::parse_gpx(const std::string &path)
{
  std::vector<log_point_t> points;
  std::ifstream file(path);
  if (!file.is_open())
  {
    std::cerr << "Failed to open GPX file: " << path << std::endl;
    return points;
  }

  std::string line;
  std::string content;
  while (std::getline(file, line))
  {
    content += line + "\n";
  }

  // Simple Regex-based parsing for <trkpt>
  // <trkpt lat="47.644548" lon="-122.326897">
  //   <ele>4.46</ele>
  //   <time>2009-10-17T18:37:26Z</time>
  // </trkpt>

  std::regex trkpt_regex(R"(<trkpt\s+lat=\"([^\"]+)\"\s+lon=\"([^\"]+)\"[^>]*>([\s\S]*?)<\/trkpt>)");
  std::regex ele_regex(R"(<ele>([^<]+)<\/ele>)");
  std::regex time_regex(R"(<time>([^<]+)<\/time>)");

  auto words_begin = std::sregex_iterator(content.begin(), content.end(), trkpt_regex);
  auto words_end = std::sregex_iterator();

  double start_time = -1.0;

  for (std::sregex_iterator i = words_begin; i != words_end; ++i)
  {
    std::smatch match = *i;
    log_point_t p;
    try
    {
      p.lat = std::stod(match[1].str());
      p.lon = std::stod(match[2].str());

      std::string inner_xml = match[3].str();
      std::smatch ele_match;
      if (std::regex_search(inner_xml, ele_match, ele_regex))
      {
        p.alt = std::stod(ele_match[1].str());
        p.has_alt = true;
      }

      std::smatch time_match;
      if (std::regex_search(inner_xml, time_match, time_regex))
      {
        double t = parse_iso8601_to_sec(time_match[1].str());
        if (start_time < 0.0)
          start_time = t;

        p.time_sec = t - start_time;
        p.has_time = true;
      }

      points.push_back(p);
    }
    catch (...)
    {
      continue;
    }
  }

  return points;
}

std::vector<log_point_t> LogParser::parse_csv(const std::string &path)
{
  std::vector<log_point_t> points;
  std::ifstream file(path);
  if (!file.is_open())
    return points;

  std::string line;
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string item;
    std::vector<std::string> tokens;
    while (std::getline(ss, item, ','))
    {
      tokens.push_back(item);
    }

    if (tokens.size() >= 2)
    {
      try
      {
        log_point_t p;
        p.lat = std::stod(tokens[0]);
        p.lon = std::stod(tokens[1]);
        if (tokens.size() >= 3)
        {
          // Check if 3rd token is numerical
          try
          {
            p.alt = std::stod(tokens[2]);
            p.has_alt = true;
          }
          catch (...)
          {
          }
        }
        points.push_back(p);
      }
      catch (...)
      {
        // Skip header or invalid lines
      }
    }
  }
  return points;
}

double LogParser::parse_iso8601_to_sec(const std::string &time_str)
{
  // Minimal ISO8601 parser: 2009-10-17T18:37:26Z
  // Uses std::get_time
  std::tm tm = {};
  std::istringstream ss(time_str);
  ss >> std::get_time(&tm, "%Y-%m-%dT%H:%M:%S");
  if (ss.fail())
    return 0.0;

  std::time_t t = std::mktime(&tm); // This assumes local time which is wrong for Z, but relative diffs might be okay if we are consistent.
  // Ideally use timegm, but it's not standard C++.
  // For relative seconds it largely doesn't matter if we are consistent, unless we cross DST.
  return static_cast<double>(t);
}

} // namespace sensor_mapper
