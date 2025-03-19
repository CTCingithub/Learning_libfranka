#include <iostream>
#include <vector>
#include <thread>

void do_some_work(int num)
{
    std::cout << "thread: " << num << std::endl;
}

int main(int argc, char const *argv[])
{
    int threadNums = 8;
    std::vector<std::thread> threadList;
    threadList.reserve(threadNums);

    // 1 ���� threadNums ���߳�
    for (int idx = 0; idx < threadNums; ++idx)
    {
        threadList.emplace_back(std::thread{do_some_work, idx});
    }

    std::cout << "work in main thread" << std::endl;

    // 2 ��ֹ threadNums ���߳�
    for (int idx = 0; idx < threadNums; ++idx)
    {
        threadList[idx].join();
    }

    std::cout << "main thread end" << std::endl;
    return 0;
}