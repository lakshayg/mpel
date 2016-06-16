#include "types.hpp"
#include "common.hpp"
#include <mpel/builtins.hpp>
#include <random>
#include <vector>
#include <iostream>

namespace mpel {

probabilistic_graph_builder::probabilistic_graph_builder(size_t n, size_t k) : _n(n) {}
Graph probabilistic_graph_builder::operator()(MapRef map) {
	// determine number of nodes
	size_t num = (_n > 0 ? _n : (map.rows * map.cols) / (50*50));

	// sample random configurations in the graph
	std::uniform_int_distribution<int> rand_x(0, map.rows);
    std::uniform_int_distribution<int> rand_y(0, map.cols);
    std::random_device rdx, rdy;

	std::vector<Point> config;
	config.reserve(num);
	while (config.size() < num) {
		Point pt = Point(rand_x(rdx), rand_y(rdy));
		if (not is_collision(map, pt)) config.push_back(pt);
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
