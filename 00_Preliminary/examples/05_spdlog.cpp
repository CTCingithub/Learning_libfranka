#include <iostream>
#include <spdlog/common.h>
#include <string>

#include "spdlog/logger.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"

#include "DataLogger.h"

int main(int argc, char **argv) {
  double data_1[4] = {1, 2, 3, 4};
  double data_2[4] = {5, 6, 7, 8};
  std::string FileName = "data.log";
  std::string LogDestination = FindLoggerPath() + "/" + FileName;

  std::cout << "log file path: " << FindLoggerPath() << std::endl;

  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(spdlog::level::warn);
  auto file_sink =
      std::make_shared<spdlog::sinks::basic_file_sink_mt>(LogDestination, true);
  file_sink->set_level(spdlog::level::trace);

  // 设置日志格式. 参数含义: [日志标识符] [日期] [日志级别] [线程号] [数据]
  file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

  spdlog::logger double_logger("double_sink", {console_sink, file_sink});
  double_logger.set_level(spdlog::level::info);

  double_logger.warn("Start logging");
  double_logger.info("{:.2f}, {:.2f}, {:.2f}, {:.2f}", data_1[0], data_1[1],
                     data_1[2], data_1[3]);
  double_logger.info("{:.2f}, {:.2f}, {:.2f}, {:.2f}", data_2[0], data_2[1],
                      data_2[2], data_2[3]);

  return 0;
}