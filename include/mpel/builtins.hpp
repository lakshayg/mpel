#ifndef MPEL_BUILTINS_H
#define MPEL_BUILTINS_H

#include "types.hpp"

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
struct voronoi_graph_builder {
	voronoi_graph_builder(double eps = 10);
	Graph operator()(MapRef map);
private:
	double _eps;
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
	Path operator()(PathRef path);
};

struct potential_field_interpolator {
	potential_field_interpolator();
	Path operator()(PathRef path);
};
}
#endif
