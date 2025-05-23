#include <atomic>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <mutex>
#include <thread>

#include "PrintCache.h"

void push_element_thread_func(double running_rate, std::atomic_bool &is_running,
                              std::mutex &mtx,
                              boost::circular_buffer<int> &buffer) {
  int counter = 1;
  while (is_running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 / running_rate)));

    if (mtx.try_lock()) {
      // 在锁保护下进行操作
      buffer.push_back(counter);
      counter++;

      // 创建副本用于安全打印
      auto buffer_copy = buffer;
      mtx.unlock();

      PrintCache(buffer_copy);
    }
  }
}

int main() {
  std::mutex mtx;
  boost::circular_buffer<int> buffer(3);
  std::atomic_bool is_running(true);

  std::cout << "Initial buffer:" << std::endl;
  PrintCache(buffer);
  std::cout << "Start thread" << std::endl;
  std::cout << "=========================================================="
            << std::endl;

  double print_rate = 20.0;

  std::thread t(push_element_thread_func, print_rate, std::ref(is_running),
                std::ref(mtx), std::ref(buffer));

  std::this_thread::sleep_for(std::chrono::seconds(5));
  is_running = false;

  if (t.joinable()) {
    t.join();
  }

  return 0;
}