#include <iostream>

#include "spdlog/spdlog.h"

#include "DataLogger.h"

int main(int argc, char **argv)
{
    double data[4][7] = {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0},
                         {20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0},
                         {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0},
                         {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}};
    std::cout << "log file path: " << FindLoggerPath() << std::endl;

    return 0;
}