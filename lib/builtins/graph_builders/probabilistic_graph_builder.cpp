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
	size_t num = (_n > 0 ? _n : (map.rows * map.cols) / (25*25));
	Graph g;

	// sample random configurations in the graph
	std::vector<Point> config;
	config.reserve(num);
	while (config.size() < num) {
		Point pt = random_free_space_point(map);
		config.push_back(pt);
	}

	for (size_t i = 0; i < num; ++i) {
		for (size_t j = i+1; j < num; ++j) {
			Point p0 = config[i];
			Point p1 = config[j];
			Segment s(p0, p1);
			if (not is_collision(map, s))
				g.add_edge(p0, p1, distance(p0,p1));
		}
	}

	// if the graph is not connected
	num = (map.rows * map.cols) / (50*50);
	while (not g.connected()) {
		for (size_t i = 0; i < num; ++i) {
			Point pt = random_free_space_point(map);
			for (auto& e : g.vertex_list()) {
				if (not is_collision(map, Segment(pt, e)))
					g.add_edge(pt, e, distance(pt,e));
			}
		}
	}
	if (_n > 0 and _n < g.num_vertices())
		std::cout << "[voronoi_graph_builder] Extra nodes were added to make the graph connected (" << g.num_vertices() << ")" << std::endl;

	return g;
}
}
