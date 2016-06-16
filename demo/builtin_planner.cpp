#include <mpel/core.hpp>
#include <mpel/builtins.hpp>
#include <iostream>

using namespace mpel;

int main(int argc, char **argv) {

	if (argc < 2) {
		std::cerr << "Demo requires a map image filename" << std::endl;
		return -1;
	}

	Workspace ws;
	ws.map = load_map_from_image(argv[1]);

	Planner p((voronoi_planner_config()));
	p.load_workspace(ws);

	ProblemDefinition pdef;
	pdef.start = Point(50, 50);
	pdef.goal = Point(340, 213);

	Path path = p.solve(pdef);

	View v;
	v.add_layer(ws.map);
	v.add_layer(pdef);
	v.add_layer(p.roadmap());
	v.add_layer(path);
	View::stay();

	return 0;
}

