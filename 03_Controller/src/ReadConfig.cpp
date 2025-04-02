#include <string>
#include <filesystem>

#include "yaml-cpp/yaml.h"

std::string FindConfigFile(const std::filesystem::path &CurrentPath)
{
    if (CurrentPath.filename() == "bin")
    {
        return CurrentPath.parent_path().string() + "/config/FR3.yaml";
    }
    else
    {
        return CurrentPath.string() + "/config/FR3.yaml";
    }
}

YAML::Node GetConfig(const std::string &ConfigurationFile)
{
    YAML::Node config = YAML::LoadFile(ConfigurationFile);
    return config;
}