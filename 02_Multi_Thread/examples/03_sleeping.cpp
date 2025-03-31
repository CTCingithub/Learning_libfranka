#include <chrono>
#include <iostream>
#include <thread>

void PrintMsg(const int num) { std::cout << num << std::endl; }

int main(int argc, char **argv) {
  for (int i = 0; i < 10; ++i) {
    PrintMsg(i + 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  return 0;
}