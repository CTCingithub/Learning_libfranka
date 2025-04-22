#ifndef READCONFIG_H
#define READCONFIG_H

#include <filesystem>
#include <string>

#include "yaml-cpp/yaml.h"

namespace {
const std::filesystem::path currentPath = std::filesystem::current_path();
const std::string default_config_file = "FR3.yaml";
} // anonymous namespace

std::string
FindConfigFile(const std::string &Filename = default_config_file,
               const std::filesystem::path &CurrentPath = currentPath);
YAML::Node GetConfig(const std::string &ConfigurationFile =
                         FindConfigFile(default_config_file, currentPath));

#endif
