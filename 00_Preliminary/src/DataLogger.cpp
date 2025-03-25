#include <string>
#include <filesystem>

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
