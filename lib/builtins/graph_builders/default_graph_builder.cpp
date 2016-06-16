#include "types.hpp"
#include <mpel/builtins.hpp>

namespace mpel {
/*
 * Default graph builder does nothing on itself, it is
 * provided so that planners which only use the local
 * information for planning can be made without affecting
 * the general procedure followed when using this library
 * Use default_graph_builder and use an interpolator as planner
*/ 
default_graph_builder::default_graph_builder() {}
Graph default_graph_builder::operator()(MapRef map) {
	Graph g;
	return g;
}
}
