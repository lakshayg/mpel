#include <iostream>
#include <mpel/builtins.hpp>
#include <mpel/core.hpp>
#include <mpel/viz.hpp>

using namespace mpel;
using namespace mpel::builtin;

int main(int argc, char** argv)
{

    if (argc < 2) {
        std::cerr << "Demo requires a map image" << std::endl;
        return -1;
    }

	//! [Planner configuration]
    Planner::Config pc;
    pc.graph_search = graph_search::none();
    pc.graph_builder = graph_builder::none();
    pc.interpolator = interpolator::potential_field();
	//! [Planner configuration]

    Planner p(pc);
    Workspace ws;
    ws.map = load_map_from_image(argv[1]);
    p.load_workspace(ws);

    ProblemDefinition pdef;
    pdef.start = mark_point(ws);
    pdef.goal = mark_point(ws);

    Path path = p.solve(pdef);

    View v;
    v.add_layer(p);
    v.add_layer(pdef);
    v.add_layer(path);
    View::stay();

    v.save();

    return 0;
}
