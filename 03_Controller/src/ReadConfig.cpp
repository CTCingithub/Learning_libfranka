#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"

std::string FindConfigFile(const std::string &Filename,
                           const std::filesystem::path &CurrentPath) {
  if (CurrentPath.filename() == "bin") {
    return CurrentPath.parent_path().string() + "/config/" + Filename;
  } else {
    return CurrentPath.string() + "/config/" + Filename;
  }
}

YAML::Node GetConfig(const std::string &ConfigurationFile) {
  YAML::Node config = YAML::LoadFile(ConfigurationFile);
  return config;
}