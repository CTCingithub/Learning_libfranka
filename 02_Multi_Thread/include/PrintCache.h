#ifndef PRINTCACHE_H
#define PRINTCACHE_H

#include "boost/circular_buffer.hpp"
#include <iostream>

template <typename Type> void PrintCache(boost::circular_buffer<Type> &cache) {
  for (int i = 0; i < cache.size(); i++) {
    std::cout << "Element " << i + 1 << ": " << cache[i];
    if (i != cache.size() - 1) {
      std::cout << ", ";
    }
    else {
      std::cout << std::endl;
    }
  }
}

#endif
