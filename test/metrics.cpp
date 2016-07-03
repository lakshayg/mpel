#include "test.hpp"
#include <iostream>
#include <mpel/builtins.hpp>
#include <mpel/core.hpp>

using namespace mpel;
using namespace mpel::builtin;

void metric_test()
{
    Point a(10, 10);
    Point b(20, 3);
    std::cout << "Euclidean Distance: " << metric::euclidean()(a, b) << std::endl;
    std::cout << "Manhattan Distance: " << metric::manhattan()(a, b) << std::endl;
    std::cout << "Chebychev Distance: " << metric::chebychev()(a, b) << std::endl;
}

int main()
{
    MPEL_TEST(metric_test);
    return 0;
}
