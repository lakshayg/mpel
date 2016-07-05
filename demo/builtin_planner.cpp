#include <iostream>
#include <mpel/builtins.hpp>
#include <mpel/core.hpp>
#include <mpel/viz.hpp>

using namespace mpel;
using namespace mpel::builtin;

int main(int argc, char** argv)
{

    if (argc < 2) {
        std::cerr << "Demo requires a map image filename" << std::endl;
        return -1;
    }

    Workspace ws;
    ws.map = load_map_from_image(argv[1]);

    Planner p((voronoi_planner_config()));
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

    return 0;
}
