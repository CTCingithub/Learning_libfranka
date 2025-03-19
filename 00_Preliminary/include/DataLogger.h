#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <iostream>
#include <string>
#include <filesystem>

#include "spdlog/spdlog.h"

const std::filesystem::path PWD = std::filesystem::current_path();

std::string FindLoggerPath(const std::filesystem::path &CurrentPath=PWD);

#endif