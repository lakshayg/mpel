#include <mpel/core.hpp>
#include <mpel/builtins.hpp>
#include "test.hpp"
#include <iostream>

using namespace mpel;

void voronoi_graph_builder_test() {
	auto gb = voronoi_graph_builder();
	Map m = load_map_from_image("004.bmp");
	Graph g = gb(m);
	std::cout << "Graph constructed" << std::endl;
}

int main() {
	MPEL_TEST(voronoi_graph_builder_test);
	return 0;
}
