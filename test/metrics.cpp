#include <iostream>
#include <mpel/core.hpp>
#include <mpel/builtins.hpp>
#include "test.hpp"

using namespace mpel;

void metric_test() {
	Point a(10, 10);
	Point b(20, 3);
	std::cout << "Euclidean Distance: " << euclidean_distance()(a, b) << std::endl;
	std::cout << "Manhattan Distance: " << manhattan_distance()(a, b) << std::endl;
	std::cout << "Chebychev Distance: " << chebychev_distance()(a, b) << std::endl;
}

int main() {
	MPEL_TEST(metric_test);
	return 0;
}
