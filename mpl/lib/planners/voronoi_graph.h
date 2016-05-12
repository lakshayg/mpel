#ifndef MPL_PLANNERS_VORONOI_GRAPH_H
#define MPL_PLANNERS_VORONOI_GRAPH_H

#include <vector>
#include <boost/polygon/voronoi.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_map>
#include "workspace.h"

class myhash {
public:
	size_t operator()(const cv::Point& pt) const {
		return pt.x * 10000 + pt.y;
	}
};

using namespace boost;
// using boost::polygon::voronoi_diagram;

namespace mplib {
	class VoronoiGraph {
		typedef adjacency_list<setS, vecS, undirectedS, no_property, property<edge_weight_t, double> > mygraph_t;
		typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
		typedef mygraph_t::vertex_descriptor vertex;
		typedef mygraph_t::edge_descriptor edge_descriptor;
		typedef mygraph_t::vertex_iterator vertex_iterator;
		typedef std::pair<int, int> edge;
		typedef polygon::voronoi_diagram<double> diagram_t;

	public:
		VoronoiGraph(Workspace *ws, const diagram_t& vd);
		void print_graph(std::string filename);
		std::vector<cv::Point> find_path(const cv::Point&, const cv::Point&);

	private:
		int nvertex, nedge;

		// actual bgl graph
		WeightMap weightmap;
		mygraph_t boost_graph;

		// for constructing bgl graph
		std::vector<cv::Point> vertices;
		std::vector<edge> edges;
		std::vector<double> weights;

		unordered_map<cv::Point,vertex,myhash> vdesc;
	};
}
#endif
