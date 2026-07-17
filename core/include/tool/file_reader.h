#pragma once

#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace common {

/**
 * FileReader provides unified file reading interface
 * Automatically handles both regular files and encrypted files from config directory
 */
class FileReader {
 public:
  // Reads file content as binary data
  static bool ReadFile(const std::string& file_path, std::vector<uint8_t>& output);

  // Reads file content as string
  static bool ReadFileToString(const std::string& file_path, std::string& output);

  // Reads file content to stringstream
  static bool ReadFileToStream(const std::string& file_path, std::stringstream& output);

  // Checks if file exists
  static bool FileExists(const std::string& file_path);

  // Gets file size in bytes
  static uint64_t GetFileSize(const std::string& file_path);

 private:
  // Checks if file path is from config directory
  static bool IsFromConfigDirectory(const std::string& file_path);
};

}  // namespace common
