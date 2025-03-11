#ifndef PRINTCACHE_H
#define PRINTCACHE_H

#include <iostream>
#include "boost/circular_buffer.hpp"

void PrintCache(boost::circular_buffer<int> &cache);

#endif
