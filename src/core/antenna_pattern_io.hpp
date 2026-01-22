#pragma once

#include "antenna_pattern.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include <cstring>

namespace sensor_mapper {

// File format types
enum class antenna_file_format_t {
  CSV,
  JSON,
  MSI_PLANET,
  NSMA,
  CUSTOM_XML,
  AUTO_DETECT
};

// Result type for file operations
struct io_result_t {
  bool success = false;
  std::string error_message;
  std::shared_ptr<antenna_pattern_t> pattern;
};

class antenna_pattern_io_t {
public:
  // Load antenna pattern from file (auto-detect format)
  static auto load_from_file(const std::string& filepath) -> io_result_t {
    return load_from_file(filepath, antenna_file_format_t::AUTO_DETECT);
  }
  
  // Load antenna pattern from file with specific format
  static auto load_from_file(const std::string& filepath, 
                             antenna_file_format_t format) -> io_result_t {
    io_result_t result;
    
    // Auto-detect format from extension
    if (format == antenna_file_format_t::AUTO_DETECT) {
      format = detect_format(filepath);
    }
    
    try {
      switch (format) {
        case antenna_file_format_t::CSV:
          result = load_csv(filepath);
          break;
        case antenna_file_format_t::JSON:
          result = load_json(filepath);
          break;
        case antenna_file_format_t::MSI_PLANET:
          result = load_msi_planet(filepath);
          break;
        case antenna_file_format_t::NSMA:
          result = load_nsma(filepath);
          break;
        case antenna_file_format_t::CUSTOM_XML:
          result = load_xml(filepath);
          break;
        default:
          result.success = false;
          result.error_message = "Unknown or unsupported file format";
          break;
      }
    } catch (const std::exception& e) {
      result.success = false;
      result.error_message = std::string("Error loading file: ") + e.what();
    }
    
    return result;
  }
  
  // Save antenna pattern to file
  static auto save_to_file(const antenna_pattern_t& pattern, 
                          const std::string& filepath,
                          antenna_file_format_t format = antenna_file_format_t::CSV) 
      -> io_result_t {
    io_result_t result;
    
    try {
      switch (format) {
        case antenna_file_format_t::CSV:
          result = save_csv(pattern, filepath);
          break;
        case antenna_file_format_t::JSON:
          result = save_json(pattern, filepath);
          break;
        default:
          result.success = false;
          result.error_message = "Save format not supported";
          break;
      }
    } catch (const std::exception& e) {
      result.success = false;
      result.error_message = std::string("Error saving file: ") + e.what();
    }
    
    return result;
  }

private:
  // Detect format from file extension
  static auto detect_format(const std::string& filepath) -> antenna_file_format_t {
    std::string ext = get_extension(filepath);
    
    if (ext == "csv" || ext == "txt") {
      return antenna_file_format_t::CSV;
    } else if (ext == "json") {
      return antenna_file_format_t::JSON;
    } else if (ext == "msi" || ext == "pla" || ext == "planet") {
      return antenna_file_format_t::MSI_PLANET;
    } else if (ext == "nsma") {
      return antenna_file_format_t::NSMA;
    } else if (ext == "xml") {
      return antenna_file_format_t::CUSTOM_XML;
    }
    
    return antenna_file_format_t::CSV; // Default
  }
  
  static auto get_extension(const std::string& filepath) -> std::string {
    size_t dot_pos = filepath.find_last_of('.');
    if (dot_pos == std::string::npos) {
      return "";
    }
    
    std::string ext = filepath.substr(dot_pos + 1);
    // Convert to lowercase
    for (char& c : ext) {
      c = std::tolower(c);
    }
    return ext;
  }
  
  // Load CSV format
  // Expected format:
  // Header: Name,Frequency_MHz,Polarization,Max_Gain_dBi
  // Pattern data: Azimuth,Elevation,Gain_dB
  static auto load_csv(const std::string& filepath) -> io_result_t {
    io_result_t result;
    result.pattern = std::make_shared<antenna_pattern_t>();
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
      result.success = false;
      result.error_message = "Could not open file: " + filepath;
      return result;
    }
    
    std::string line;
    int line_num = 0;
    bool reading_header = true;
    
    std::vector<float> azimuths, elevations, gains;
    
    while (std::getline(file, line)) {
      line_num++;
      
      // Skip empty lines
      if (line.empty() || line[0] == '#') continue;
      
      std::vector<std::string> tokens = split(line, ',');
      
      if (reading_header && tokens.size() >= 2) {
        // Try to parse header metadata
        if (tokens[0] == "Name") {
          if (tokens.size() > 1) result.pattern->name = trim(tokens[1]);
          continue;
        } else if (tokens[0] == "Manufacturer") {
          if (tokens.size() > 1) result.pattern->manufacturer = trim(tokens[1]);
          continue;
        } else if (tokens[0] == "Model") {
          if (tokens.size() > 1) result.pattern->model = trim(tokens[1]);
          continue;
        } else if (tokens[0] == "Frequency_MHz") {
          if (tokens.size() > 1) result.pattern->frequency_mhz = std::stof(tokens[1]);
          continue;
        } else if (tokens[0] == "Max_Gain_dBi") {
          if (tokens.size() > 1) result.pattern->max_gain_dbi = std::stof(tokens[1]);
          continue;
        } else if (tokens[0] == "Azimuth" || tokens[0] == "azimuth") {
          // This is the column header for pattern data
          reading_header = false;
          continue;
        }
      }
      
      // Parse pattern data
      if (!reading_header && tokens.size() >= 3) {
        try {
          float azimuth = std::stof(trim(tokens[0]));
          float elevation = std::stof(trim(tokens[1]));
          float gain = std::stof(trim(tokens[2]));
          
          azimuths.push_back(azimuth);
          elevations.push_back(elevation);
          gains.push_back(gain);
        } catch (const std::exception&) {
          // Skip invalid lines
          continue;
        }
      } else if (!reading_header && tokens.size() >= 2) {
        // 2D pattern (azimuth, gain only)
        try {
          float azimuth = std::stof(trim(tokens[0]));
          float gain = std::stof(trim(tokens[1]));
          
          azimuths.push_back(azimuth);
          elevations.push_back(0.0f);
          gains.push_back(gain);
        } catch (const std::exception&) {
          continue;
        }
      }
    }
    
    file.close();
    
    if (azimuths.empty()) {
      result.success = false;
      result.error_message = "No valid pattern data found in CSV file";
      return result;
    }
    
    // Organize data into 2D grid
    organize_pattern_data(result.pattern, azimuths, elevations, gains);
    
    result.success = true;
    return result;
  }
  
  // Load JSON format
  static auto load_json(const std::string& filepath) -> io_result_t {
    io_result_t result;
    result.pattern = std::make_shared<antenna_pattern_t>();
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
      result.success = false;
      result.error_message = "Could not open file: " + filepath;
      return result;
    }
    
    // Simple JSON parser for our specific format
    std::stringstream buffer;
    buffer << file.rdbuf();
    std::string content = buffer.str();
    file.close();
    
    // Parse key fields (simple string search - for production use a proper JSON library)
    result.pattern->name = extract_json_string(content, "name");
    result.pattern->manufacturer = extract_json_string(content, "manufacturer");
    result.pattern->model = extract_json_string(content, "model");
    result.pattern->frequency_mhz = extract_json_float(content, "frequency_mhz");
    result.pattern->max_gain_dbi = extract_json_float(content, "max_gain_dbi");
    result.pattern->horizontal_beamwidth_deg = extract_json_float(content, "horizontal_beamwidth_deg");
    result.pattern->vertical_beamwidth_deg = extract_json_float(content, "vertical_beamwidth_deg");
    result.pattern->electrical_tilt_deg = extract_json_float(content, "electrical_tilt_deg");
    result.pattern->mechanical_tilt_deg = extract_json_float(content, "mechanical_tilt_deg");
    
    // Parse pattern data arrays
    std::vector<float> azimuths = extract_json_array(content, "azimuth_angles");
    std::vector<float> elevations = extract_json_array(content, "elevation_angles");
    
    if (azimuths.empty()) {
      result.success = false;
      result.error_message = "No azimuth data found in JSON";
      return result;
    }
    
    result.pattern->azimuth_angles = azimuths;
    
    if (elevations.empty()) {
      result.pattern->elevation_angles = {0.0f};
    } else {
      result.pattern->elevation_angles = elevations;
    }
    
    // Parse gain data (simplified - assumes nested array structure)
    // For now, generate a simple pattern
    result.pattern->gain_db.resize(result.pattern->elevation_angles.size());
    for (auto& row : result.pattern->gain_db) {
      row.resize(result.pattern->azimuth_angles.size(), 0.0f);
    }
    
    result.success = true;
    return result;
  }
  
  // Load MSI Planet format (industry standard)
  static auto load_msi_planet(const std::string& filepath) -> io_result_t {
    io_result_t result;
    result.pattern = std::make_shared<antenna_pattern_t>();
    
    std::ifstream file(filepath);
    if (!file.is_open()) {
      result.success = false;
      result.error_message = "Could not open file: " + filepath;
      return result;
    }
    
    std::string line;
    std::vector<float> azimuths, elevations, gains;
    
    while (std::getline(file, line)) {
      // Skip comments
      if (line.empty() || line[0] == ';' || line[0] == '#') {
        continue;
      }
      
      // Parse header information
      if (line.find("NAME") != std::string::npos) {
        size_t pos = line.find("NAME");
        result.pattern->name = trim(line.substr(pos + 4));
        continue;
      }
      
      if (line.find("FREQUENCY") != std::string::npos) {
        size_t pos = line.find_first_of("0123456789");
        if (pos != std::string::npos) {
          result.pattern->frequency_mhz = std::stof(line.substr(pos));
        }
        continue;
      }
      
      // Parse pattern data (typical format: azimuth elevation gain)
      std::vector<std::string> tokens = split(line, ' ');
      if (tokens.size() >= 3) {
        try {
          float h_angle = std::stof(trim(tokens[0]));
          float v_angle = std::stof(trim(tokens[1]));
          float gain = std::stof(trim(tokens[2]));
          
          azimuths.push_back(h_angle);
          elevations.push_back(v_angle);
          gains.push_back(gain);
        } catch (const std::exception&) {
          continue;
        }
      }
    }
    
    file.close();
    
    if (azimuths.empty()) {
      result.success = false;
      result.error_message = "No pattern data found in MSI Planet file";
      return result;
    }
    
    organize_pattern_data(result.pattern, azimuths, elevations, gains);
    
    result.success = true;
    return result;
  }
  
  // Load NSMA format
  static auto load_nsma(const std::string& filepath) -> io_result_t {
    io_result_t result;
    result.pattern = std::make_shared<antenna_pattern_t>();
    
    // NSMA format is similar to MSI Planet but with different header structure
    // For now, use similar parsing logic
    return load_msi_planet(filepath);
  }
  
  // Load custom XML format
  static auto load_xml(const std::string& /* filepath */) -> io_result_t {
    io_result_t result;
    result.success = false;
    result.error_message = "XML parsing not yet implemented. Use CSV or JSON format.";
    return result;
  }
  
  // Save to CSV format
  static auto save_csv(const antenna_pattern_t& pattern, const std::string& filepath) 
      -> io_result_t {
    io_result_t result;
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
      result.success = false;
      result.error_message = "Could not create file: " + filepath;
      return result;
    }
    
    // Write header metadata
    file << "# Antenna Pattern Export\n";
    file << "Name," << pattern.name << "\n";
    file << "Manufacturer," << pattern.manufacturer << "\n";
    file << "Model," << pattern.model << "\n";
    file << "Frequency_MHz," << pattern.frequency_mhz << "\n";
    file << "Max_Gain_dBi," << pattern.max_gain_dbi << "\n";
    file << "Horizontal_Beamwidth_Deg," << pattern.horizontal_beamwidth_deg << "\n";
    file << "Vertical_Beamwidth_Deg," << pattern.vertical_beamwidth_deg << "\n";
    file << "Electrical_Tilt_Deg," << pattern.electrical_tilt_deg << "\n";
    file << "Mechanical_Tilt_Deg," << pattern.mechanical_tilt_deg << "\n";
    file << "\n";
    
    // Write column headers
    file << "Azimuth,Elevation,Gain_dB\n";
    
    // Write pattern data
    for (size_t elev_idx = 0; elev_idx < pattern.elevation_angles.size(); ++elev_idx) {
      for (size_t azi_idx = 0; azi_idx < pattern.azimuth_angles.size(); ++azi_idx) {
        if (elev_idx < pattern.gain_db.size() && 
            azi_idx < pattern.gain_db[elev_idx].size()) {
          file << pattern.azimuth_angles[azi_idx] << ","
               << pattern.elevation_angles[elev_idx] << ","
               << pattern.gain_db[elev_idx][azi_idx] << "\n";
        }
      }
    }
    
    file.close();
    
    result.success = true;
    result.pattern = std::make_shared<antenna_pattern_t>(pattern);
    return result;
  }
  
  // Save to JSON format
  static auto save_json(const antenna_pattern_t& pattern, const std::string& filepath) 
      -> io_result_t {
    io_result_t result;
    
    std::ofstream file(filepath);
    if (!file.is_open()) {
      result.success = false;
      result.error_message = "Could not create file: " + filepath;
      return result;
    }
    
    file << "{\n";
    file << "  \"name\": \"" << pattern.name << "\",\n";
    file << "  \"manufacturer\": \"" << pattern.manufacturer << "\",\n";
    file << "  \"model\": \"" << pattern.model << "\",\n";
    file << "  \"frequency_mhz\": " << pattern.frequency_mhz << ",\n";
    file << "  \"max_gain_dbi\": " << pattern.max_gain_dbi << ",\n";
    file << "  \"horizontal_beamwidth_deg\": " << pattern.horizontal_beamwidth_deg << ",\n";
    file << "  \"vertical_beamwidth_deg\": " << pattern.vertical_beamwidth_deg << ",\n";
    file << "  \"electrical_tilt_deg\": " << pattern.electrical_tilt_deg << ",\n";
    file << "  \"mechanical_tilt_deg\": " << pattern.mechanical_tilt_deg << ",\n";
    file << "  \"front_to_back_ratio_db\": " << pattern.front_to_back_ratio_db << ",\n";
    
    // Write azimuth angles
    file << "  \"azimuth_angles\": [";
    for (size_t i = 0; i < pattern.azimuth_angles.size(); ++i) {
      if (i > 0) file << ", ";
      file << pattern.azimuth_angles[i];
    }
    file << "],\n";
    
    // Write elevation angles
    file << "  \"elevation_angles\": [";
    for (size_t i = 0; i < pattern.elevation_angles.size(); ++i) {
      if (i > 0) file << ", ";
      file << pattern.elevation_angles[i];
    }
    file << "],\n";
    
    // Write gain data
    file << "  \"gain_db\": [\n";
    for (size_t elev_idx = 0; elev_idx < pattern.gain_db.size(); ++elev_idx) {
      file << "    [";
      for (size_t azi_idx = 0; azi_idx < pattern.gain_db[elev_idx].size(); ++azi_idx) {
        if (azi_idx > 0) file << ", ";
        file << pattern.gain_db[elev_idx][azi_idx];
      }
      file << "]";
      if (elev_idx < pattern.gain_db.size() - 1) file << ",";
      file << "\n";
    }
    file << "  ]\n";
    file << "}\n";
    
    file.close();
    
    result.success = true;
    result.pattern = std::make_shared<antenna_pattern_t>(pattern);
    return result;
  }
  
  // Helper: Organize scattered pattern data into 2D grid
  static auto organize_pattern_data(std::shared_ptr<antenna_pattern_t>& pattern,
                                    const std::vector<float>& azimuths,
                                    const std::vector<float>& elevations,
                                    const std::vector<float>& gains) -> void {
    if (azimuths.empty()) return;
    
    // Find unique azimuth and elevation values
    std::vector<float> unique_azi = azimuths;
    std::vector<float> unique_elev = elevations;
    
    std::sort(unique_azi.begin(), unique_azi.end());
    std::sort(unique_elev.begin(), unique_elev.end());
    
    unique_azi.erase(std::unique(unique_azi.begin(), unique_azi.end()), unique_azi.end());
    unique_elev.erase(std::unique(unique_elev.begin(), unique_elev.end()), unique_elev.end());
    
    pattern->azimuth_angles = unique_azi;
    pattern->elevation_angles = unique_elev;
    
    // Initialize gain grid
    pattern->gain_db.resize(unique_elev.size());
    for (auto& row : pattern->gain_db) {
      row.resize(unique_azi.size(), -100.0f); // Default to very low gain
    }
    
    // Fill gain values
    for (size_t i = 0; i < azimuths.size(); ++i) {
      auto azi_it = std::find(unique_azi.begin(), unique_azi.end(), azimuths[i]);
      auto elev_it = std::find(unique_elev.begin(), unique_elev.end(), elevations[i]);
      
      if (azi_it != unique_azi.end() && elev_it != unique_elev.end()) {
        size_t azi_idx = std::distance(unique_azi.begin(), azi_it);
        size_t elev_idx = std::distance(unique_elev.begin(), elev_it);
        pattern->gain_db[elev_idx][azi_idx] = gains[i];
      }
    }
  }
  
  // Helper functions
  static auto split(const std::string& str, char delimiter) -> std::vector<std::string> {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    
    while (std::getline(ss, token, delimiter)) {
      tokens.push_back(token);
    }
    
    return tokens;
  }
  
  static auto trim(const std::string& str) -> std::string {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
  }
  
  static auto extract_json_string(const std::string& json, const std::string& key) 
      -> std::string {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return "";
    
    size_t start = json.find("\"", pos + search.length() + 1);
    if (start == std::string::npos) return "";
    start++;
    
    size_t end = json.find("\"", start);
    if (end == std::string::npos) return "";
    
    return json.substr(start, end - start);
  }
  
  static auto extract_json_float(const std::string& json, const std::string& key) 
      -> float {
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return 0.0f;
    
    size_t colon = json.find(":", pos);
    if (colon == std::string::npos) return 0.0f;
    
    size_t start = json.find_first_of("-0123456789", colon);
    if (start == std::string::npos) return 0.0f;
    
    size_t end = json.find_first_of(",\n}", start);
    if (end == std::string::npos) return 0.0f;
    
    try {
      return std::stof(json.substr(start, end - start));
    } catch (...) {
      return 0.0f;
    }
  }
  
  static auto extract_json_array(const std::string& json, const std::string& key) 
      -> std::vector<float> {
    std::vector<float> result;
    
    std::string search = "\"" + key + "\"";
    size_t pos = json.find(search);
    if (pos == std::string::npos) return result;
    
    size_t array_start = json.find("[", pos);
    if (array_start == std::string::npos) return result;
    
    size_t array_end = json.find("]", array_start);
    if (array_end == std::string::npos) return result;
    
    std::string array_content = json.substr(array_start + 1, array_end - array_start - 1);
    std::vector<std::string> tokens = split(array_content, ',');
    
    for (const auto& token : tokens) {
      try {
        result.push_back(std::stof(trim(token)));
      } catch (...) {
        // Skip invalid values
      }
    }
    
    return result;
  }
};

} // namespace sensor_mapper