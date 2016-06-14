#include <mpel/builtins.hpp>
#include "types.hpp"
#include "planner.hpp"

namespace mpel {
struct voronoi_planner_config : Planner::Config {
	voronoi_planner_config() {
		graph_builder = voronoi_graph_builder();
		graph_search = dijkstra_search();
		interpolator = default_interpolator();
	}
};
}
