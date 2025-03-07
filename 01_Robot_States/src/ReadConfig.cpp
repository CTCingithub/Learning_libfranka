#include <string>
#include <filesystem>

#include "yaml-cpp/yaml.h"

const std::filesystem::path CurrentPath = std::filesystem::current_path();
const std::filesystem::path ParentPath = CurrentPath.parent_path();
const std::string DefaultConfigurationFile = ParentPath.string() + "/config/FR3.yaml";

YAML::Node GetConfig(const std::string &ConfigurationFile = DefaultConfigurationFile)
{
    YAML::Node config = YAML::LoadFile(ConfigurationFile);
    return config;
}