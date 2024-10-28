// Licensed under GPLv2. See LICENSE file. Copyright Carologistics.

#include "cx_utils/param_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>

namespace cx {
namespace cx_utils {

void resolve_files(const std::vector<std::string> &files_in,
                   const std::vector<std::string> &share_dirs,
                   std::vector<std::string> &files_out) {
  // Try to resolve files
  for (const auto &file : files_in) {
    bool found = false;
    std::filesystem::path file_path(file);
    // Check if the path is absolute and exists
    if (file_path.is_absolute() && std::filesystem::exists(file_path)) {
      files_out.push_back(file_path);
      found = true;
    } else {
      // Otherwise check in the respective share directories
      for (const auto &package : share_dirs) {
        std::string dir_path =
            ament_index_cpp::get_package_share_directory(package);
        std::filesystem::path share_dir_path(dir_path);

        std::filesystem::path potential_path = share_dir_path / file_path;
        if (std::filesystem::exists(potential_path)) {
          files_out.push_back(potential_path);
          found = true;
          break;
        }
      }
    }
    if (!found) {
      throw std::runtime_error("Cannot resolve file " + file);
    }
  }
}

} // namespace cx_utils
} // namespace cx
