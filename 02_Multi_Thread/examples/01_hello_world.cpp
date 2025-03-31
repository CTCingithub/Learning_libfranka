#include <iostream>
#include <thread>

void PrintMsg(const std::string &msg_1, const std::string &msg_2) {
  std::cout << msg_1 << " " << msg_2 << std::endl;
}

int main(int argc, char **argv) {
  int num_max_parallel_threads = std::thread::hardware_concurrency();
  std::cout << "Maximum parallel threads: " << num_max_parallel_threads
            << std::endl;

  std::thread t1{PrintMsg, "Hello", "World"};
  std::cout << "Hello World thread is joinable before join(): " << t1.joinable()
            << std::endl;
  t1.join();
  std::cout << "Hello World thread is joinable after join(): " << t1.joinable()
            << std::endl;
  return 0;
}