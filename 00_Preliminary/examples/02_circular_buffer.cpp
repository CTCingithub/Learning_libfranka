#include <iostream>
#include <boost/circular_buffer.hpp>

#include "PrintCache.h"

int main(int argc, char **argv)
{
    boost::circular_buffer<int> cache(5);
    std::cout << "Create a circular buffer with 5 int values"<<std::endl;
    std::cout << "Current Cache size: " << cache.size() << std::endl;
    cache.push_back(1);
    std::cout << "Push back 1" << std::endl;
    std::cout << "Current Cache size: " << cache.size() << std::endl;
    std::cout << cache.capacity() << std::endl;
    PrintCache(cache);
    return 0;
}