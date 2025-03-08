#include <string>
#include <filesystem>

#include "yaml-cpp/yaml.h"

YAML::Node GetConfig(const std::string &ConfigurationFile)
{
    YAML::Node config = YAML::LoadFile(ConfigurationFile);
    return config;
}