#ifndef READCONFIG_H
#define READCONFIG_H

#include <string>
#include <filesystem>

#include "yaml-cpp/yaml.h"

extern const std::string DefaultConfigurationFile;
YAML::Node GetConfig(const std::string &ConfigurationFile = DefaultConfigurationFile);

#endif
