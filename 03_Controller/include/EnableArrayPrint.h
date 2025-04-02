#include <iostream>
#include <array>
#include <iterator>

/**
    This header file defines a template to enable std::array type
    variables printed with std::cout.
**/

template <class T, size_t N>
std::ostream &operator<<(std::ostream &ostream, const std::array<T, N> &array)
{
    ostream << "[";
    std::copy(
        array.cbegin(),
        array.cend() - 1,
        std::ostream_iterator<T>(ostream, ", "));
    std::copy(
        array.cend() - 1, array.cend(), std::ostream_iterator<T>(ostream));
    ostream << "]";
    return ostream;
}
