#include <iostream>
#include "boost/circular_buffer.hpp"

void PrintCache(boost::circular_buffer<int> &cache)
{
    for (int i = 0; i < cache.capacity(); i++)
    {
        std::cout << "Element " << i + 1 << std::endl
                  << cache[i] << std::endl;
    }
}