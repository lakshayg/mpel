#include <mpel/builtins.hpp>
#include "types.hpp"
#include "planner.hpp"

namespace mpel {
	voronoi_planner_config::voronoi_planner_config() {
		graph_builder = voronoi_graph_builder();
		graph_search = dijkstra_search();
		interpolator = default_interpolator();
	}
}
