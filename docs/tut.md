# MPEL Tutorials

### Using a pre-built planner

The code below shows how to use a motion planner provided by the library
for motion planning

```
#include <mpel/core.hpp>
#include <mpel/builtins.hpp>

using namespace mpel;

int main() {
	Planner::Config pc = voronoi_planner_config();  // Pre-built planner configuration
	Planner p(pc); // Define a voronoi roadmap planner

	Workspace ws;
	ws.map = load_map_from_image("004.bmp"); // load the workspace map from a bitmap
	                                         // this is currently the ONLY method for
											 // loading maps

	ProblemDefinition pdef;      // Definition of the problem to be solved
	pdef.start = Point(50,50);
	pdef.goal = Point(400,400);

	p.load_workspace(ws);        // load workspace into the planner
	Path path = p.solve(pdef);   // solve the given problem and give the path as output

	View v("Voronoi Planner");   // Display the results
	v.add_layer(p);
	v.add_layer(path);
	View::stay(); // Prevent the program from exiting

	return 0;
}
```
