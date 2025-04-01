#ifndef PRINTCACHE_H
#define PRINTCACHE_H

#include "boost/circular_buffer.hpp"
#include <iostream>

template <typename Type> void PrintCache(boost::circular_buffer<Type> &cache) {
  for (int i = 0; i < cache.capacity() - 1; i++) {
    std::cout << "Element " << i + 1 << ": " << cache[i] << ", ";
  }
  std::cout << "Element " << cache.capacity() << ": " << cache.back() << std::endl;
}

#endif
