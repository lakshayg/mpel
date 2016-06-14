#include <mpel/core.hpp>
#include "test.hpp"
#include <iostream>

using namespace mpel;

void collision_test() {
	Map m = load_map_from_image("004.bmp");
	std::cout << (is_collision(m, Point(10, 10)) ? "true" : "false") << std::endl;
	std::cout << (is_collision(m, Point(100, 100)) ? "true" : "false") << std::endl;
}

int main() {
	MPEL_TEST(collision_test);
	return 0;
}
