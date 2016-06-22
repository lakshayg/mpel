#ifndef MPEL_BUILTINS_H
#define MPEL_BUILTINS_H

#include "types.hpp"
#include "planner.hpp"

namespace mpel {

// Distance metrics
struct euclidean_distance {
	double operator()(PointRef a, PointRef b);
};

struct manhattan_distance {
	double operator()(PointRef a, PointRef b);
};

struct chebychev_distance {
	double operator()(PointRef a, PointRef b);
};


// Graph builders
struct default_graph_builder {
	default_graph_builder();
	Graph operator()(MapRef map);
};

struct voronoi_graph_builder {
	voronoi_graph_builder(double eps = 10);
	Graph operator()(MapRef map);
private:
	double _eps; // parameter for approximating the workspace
};

struct probabilistic_graph_builder {
	probabilistic_graph_builder(size_t n = 0);
	Graph operator()(MapRef map);
private:
	size_t _n; // number of nodes in the graph 0 => automatically determined
};

// Grpah searches
struct default_search { // does nothing meaningful (only for debugging)
	default_search();
	Path operator()(GraphRef g, PointRef a, PointRef b);
};

struct a_star_search {
	a_star_search();
	Path operator()(GraphRef g, PointRef a, PointRef b);
};

struct dijkstra_search {
	dijkstra_search();
	Path operator()(GraphRef g, PointRef a, PointRef b);
};

struct breadth_first_search {
	breadth_first_search();
	Path operator()(GraphRef g, PointRef a, PointRef b);
};

struct depth_first_search {
	depth_first_search();
	Path operator()(GraphRef g, PointRef a, PointRef b);
};

// Interpolators
struct default_interpolator {
	default_interpolator();
	Path operator()(MapRef map, PathRef path);
};

struct bug2_interpolator {
	bug2_interpolator(double step = 2);
	Path operator()(MapRef map, PathRef path);
private:
	double _step;
};

struct potential_field_interpolator {
	potential_field_interpolator(double eps = 10);
	Path operator()(MapRef map, PathRef path);
private:
	double _eps;
};

// Planner configs
struct default_planner_config : Planner::Config {
	default_planner_config();
};

struct voronoi_planner_config : Planner::Config {
	voronoi_planner_config();
};

struct PRM_planner_config : Planner::Config {
	PRM_planner_config();
};
}
#endif
