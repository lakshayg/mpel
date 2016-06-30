#include <mpel/builtins.hpp>
#include "types.hpp"
#include "common.hpp"

#include <stack>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/isotropy.hpp>
#include <boost/polygon/point_concept.hpp>
#include <boost/polygon/segment_concept.hpp>
#include <boost/polygon/rectangle_concept.hpp>

using namespace mpel;
using boost::polygon::voronoi_diagram;

typedef voronoi_diagram<double>::vertex_type& voronoi_vertex;
typedef voronoi_diagram<double>::edge_type& voronoi_edge;
typedef voronoi_diagram<double>::coordinate_type coordinate_type;
typedef voronoi_diagram<double>::vertex_type point_type;
typedef voronoi_diagram<double>::cell_type cell_type;
typedef voronoi_diagram<double>::edge_type segment_type;

// map mpel point type to boost polygon point_concept
namespace boost {
namespace polygon {

template <>
struct geometry_concept<Point> {
	typedef point_concept type;
};

template <>
struct point_traits<Point> {
	typedef int coordinate_type;

	static inline coordinate_type get(
	    PointRef point, orientation_2d orient) {
		return (orient == HORIZONTAL) ? point.x : point.y;
	}
};


template <>
struct geometry_concept<Segment> {
	typedef segment_concept type;
};

template <>
struct segment_traits<Segment> {
	typedef int coordinate_type;
	typedef Point point_type;

	static inline point_type get(const Segment& segment, direction_1d dir) {
		return dir.to_int() ? segment.p1 : segment.p0;
	}
};
}
}

namespace mpel {

voronoi_graph_builder::voronoi_graph_builder(double eps) : _eps(eps) {}

Graph voronoi_graph_builder::operator()(MapRef map) {
	// build voronoi diagram
	voronoi_diagram<double> vd;
	std::vector<Segment> segments = get_map_segments(map, _eps);
	construct_voronoi(segments.begin(), segments.end(), &vd);

	// build graph from diagram
	Graph g;
	for (auto& e : vd.edges()) {
		if (e.is_infinite()) continue;
		if (not e.is_primary()) continue;

		Point p0(e.vertex0()->x(), e.vertex0()->y());
		Point p1(e.vertex1()->x(), e.vertex1()->y());

		int nsize = 5;
		bool flag = false;
		for (int i = -nsize; i <= nsize and not flag; ++i)
			for (int j = -nsize; j <= nsize and not flag; ++j)
				flag = (is_collision(map, p0 + Point(i,j)) or is_collision(map, p1 + Point(i,j)));
		if (flag) continue;

		for (double k = 0; k <= 1 and not flag; k += 0.2)
			flag = is_collision(map, k * p0 + (1 - k) * p1);

		if (flag) continue;
		g.add_edge(p0, p1, distance(p0, p1));
	}
	return g;
}
}

