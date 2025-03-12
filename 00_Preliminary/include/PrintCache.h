#ifndef PRINTCACHE_H
#define PRINTCACHE_H

#include <iostream>
#include "boost/circular_buffer.hpp"

template <typename Type>
void PrintCache(boost::circular_buffer<Type> &cache)
{
    for (int i = 0; i < cache.capacity(); i++)
    {
        std::cout << "Element " << i + 1 << std::endl
                  << cache[i] << std::endl;
    }
}
// void PrintCache(boost::circular_buffer<Type> &cache);

#endif
