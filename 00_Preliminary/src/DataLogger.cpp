#include <iostream>
#include <string>
#include <filesystem>

#include "spdlog/spdlog.h"

std::string FindLoggerPath(const std::filesystem::path &CurrentPath)
{
    if (CurrentPath.filename() == "bin")
    {
        return CurrentPath.parent_path().string() + "/log";
    }
    else
    {
        return CurrentPath.string() + "/log";
    }
}
