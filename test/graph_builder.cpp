#include "test.hpp"
#include <iostream>
#include <mpel/builtins.hpp>
#include <mpel/core.hpp>

using namespace mpel;
using namespace mpel::builtin;

void voronoi_graph_builder_test()
{
    auto gb = graph_builder::voronoi();
    Map m = load_map_from_image("004.bmp");
    Graph g = gb(m);
    std::cout << "Graph constructed" << std::endl;
}

int main()
{
    MPEL_TEST(voronoi_graph_builder_test);
    return 0;
}
