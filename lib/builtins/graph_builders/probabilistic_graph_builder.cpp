#include "types.hpp"
#include "common.hpp"
#include <mpel/builtins.hpp>
#include <random>
#include <vector>
#include <iostream>

namespace mpel {

probabilistic_graph_builder::probabilistic_graph_builder(size_t n) : _n(n) {}
Graph probabilistic_graph_builder::operator()(MapRef map) {
	// determine number of nodes
	size_t num = (_n > 0 ? _n : (map.rows * map.cols) / (50*50));

	// sample random configurations in the graph
	std::vector<Point> config;
	config.reserve(num);
	while (config.size() < num) {
		Point pt = random_free_space_point(map);
		config.push_back(pt);
	}

	Graph g;
	for (size_t i = 0; i < num; ++i) {
		for (size_t j = i+1; j < num; ++j) {
			Point p0 = config[i];
			Point p1 = config[j];
			Segment s(p0, p1);
			if (not is_collision(map, s))
				g.add_edge(p0, p1, distance(p0,p1));
		}
	}
	return g;
}
}
