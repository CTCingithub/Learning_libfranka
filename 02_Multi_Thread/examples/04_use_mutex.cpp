#include <atomic>
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <mutex>
#include <thread>

#include "PrintCache.h"

struct Cache {
  std::mutex mutex;
  boost::circular_buffer<int> buffer{3};
};

void push_element(Cache &cache, int value) {
  if (cache.mutex.try_lock()) {
    cache.mutex.unlock();
  }
  cache.buffer.push_back(value);
}

void push_element_thread_func(double running_rate, std::atomic_bool &is_running,
                              Cache &cache) {
  int counter = 1;
  while (is_running) {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(static_cast<int>(1000 / running_rate)));

    if (cache.mutex.try_lock()) {
      push_element(cache, counter);
      counter++;
      cache.mutex.unlock();
      PrintCache(cache.buffer);
    }
  }
}

int main(int argc, char **argv) {
  Cache cache;
  std::atomic_bool is_running(true);

  std::cout << "Initial buffer:" << std::endl;
  PrintCache(cache.buffer);
  std::cout << "Start thread" << std::endl;
  std::cout << "=========================================================="
            << std::endl;

  double print_rate = 20.0;

  std::thread t(push_element_thread_func, print_rate, std::ref(is_running),
                std::ref(cache));

  // Sleep for 5 seconds
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // Stop the thread
  is_running = false;

  if (t.joinable()) {
    t.join();
  }

  return 0;
}