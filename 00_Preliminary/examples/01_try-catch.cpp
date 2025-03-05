#include <iostream>
#include <stdexcept>

int main(int argc, char **argv)
{
    try
    {
        // throw std::runtime_error("Something went wrong"); when encountered error
        int a = 10, b = 0;
        std::cout << "a =" << a << std::endl;
        std::cout << "b =" << b << std::endl;
        std::cout << "Trying to compute a/b" << std::endl;
        if (b == 0)
        {
            throw std::runtime_error("Divide by zero error");
        }
        int result = a / b;
    }
    catch (const std::runtime_error &e)
    {
        std::cerr << "Caught runtime error: " << e.what() << std::endl;
    }

    return 0;
}