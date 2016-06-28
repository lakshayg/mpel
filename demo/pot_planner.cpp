#include <mpel/core.hpp>
#include <mpel/builtins.hpp>
#include <iostream>

using namespace mpel;

int main(int argc, char **argv) {

	if (argc < 2) {
		std::cerr << "Demo requires a map image" << std::endl;
		return -1;
	}

	Planner::Config pc;
	pc.graph_search = dijkstra_search();
	pc.graph_builder = voronoi_graph_builder();
	pc.interpolator = potential_field_interpolator();

	Planner p(pc);
	Workspace ws;
	ws.map = load_map_from_image(argv[1]);
	p.load_workspace(ws);

	ProblemDefinition pdef;
	pdef.start = Point(50,50);
	pdef.goal = Point(400,400);

	Path path = p.solve(pdef);

	View v;
	v.add_layer(p);
	v.add_layer(pdef);
	v.add_layer(path);
	View::stay();

	v.save();

	return 0;
}
