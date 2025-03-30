#include <iostream>
#include <thread>

void PrintMsg(const std::string &msg_1, const std::string &msg_2) {
  std::cout << msg_1 << " " << msg_2 << std::endl;
}

int main(int argc, char **argv) {
  std::thread t1(PrintMsg, "Hello", "World");
  t1.join();
  return 0;
}