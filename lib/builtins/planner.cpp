#include <mpel/builtins.hpp>
#include "types.hpp"
#include "planner.hpp"

namespace mpel {

default_planner_config::default_planner_config() {
	graph_builder = voronoi_graph_builder();
	graph_search = dijkstra_search();
	interpolator = default_interpolator();
}

voronoi_planner_config::voronoi_planner_config() {
	graph_builder = voronoi_graph_builder();
	graph_search = dijkstra_search();
	interpolator = default_interpolator();
}

PRM_planner_config::PRM_planner_config() {
	graph_builder = probabilistic_graph_builder();
	graph_search = dijkstra_search();
	interpolator = default_interpolator();
}

}
