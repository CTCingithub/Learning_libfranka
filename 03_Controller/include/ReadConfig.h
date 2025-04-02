#ifndef READCONFIG_H
#define READCONFIG_H

#include <string>
#include <filesystem>

#include "yaml-cpp/yaml.h"

const std::filesystem::path CurrentPath = std::filesystem::current_path();

std::string FindConfigFile(const std::filesystem::path &CurrentPath);
YAML::Node GetConfig(const std::string &ConfigurationFile = FindConfigFile(CurrentPath));

#endif
