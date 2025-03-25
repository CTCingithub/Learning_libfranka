#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <string>
#include <filesystem>

const std::filesystem::path PWD = std::filesystem::current_path();

std::string FindLoggerPath(const std::filesystem::path &CurrentPath=PWD);

#endif