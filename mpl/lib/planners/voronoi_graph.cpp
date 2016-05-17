#include "voronoi_graph.h"

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt
#include <cassert>

using namespace boost;
using namespace std;

// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType> {
public:
	typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(LocMap l, Vertex goal) : m_location(l), m_goal(goal) {}
	CostType operator()(Vertex u) {
		CostType dx = m_location[m_goal].x - m_location[u].x;
		CostType dy = m_location[m_goal].y - m_location[u].y;
		return sqrt(dx * dx + dy * dy);
	}
private:
	LocMap m_location;
	Vertex m_goal;
};


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor {
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph& g) {
		if (u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};


namespace mplib {

VoronoiGraph::VoronoiGraph(Workspace *ws, const diagram_t& vd) {
	nvertex = 0;
	nedge = 0;
	for (auto& v_edge : vd.edges()) {
		if (v_edge.is_infinite()) continue;
		if (not v_edge.is_primary()) continue;

		cv::Point p0((int) v_edge.vertex0()->x(), (int) v_edge.vertex0()->y());
		cv::Point p1((int) v_edge.vertex1()->x(), (int) v_edge.vertex1()->y());

		if (ws->collision(p0) || ws->collision(p1)) continue;
		if (ws->collision(0.5*p0 + 0.5*p1)) continue;

		edge e;

		// check if p0 is already in the graph, if not then insert
		std::pair<cv::Point, int> val0(p0, nvertex);
		auto ins0 = vdesc.insert(val0);
		if (ins0.second) { // if the vertex was not already present
			vertices.push_back(p0);
			e.first = nvertex;
			nvertex++;
		}
		else {
			auto tmp = vdesc.find(p0);
			assert(tmp != vdesc.end());
			e.first = tmp->second;
		}

		// check if p1 is already in the graph, if not then insert
		std::pair<cv::Point, int> val1(p1, nvertex);
		auto ins1 = vdesc.insert(val1);
		if (ins1.second) { // if the vertex was not already present
			vertices.push_back(p1);
			e.second = nvertex;
			nvertex++;
		}
		else {
			auto tmp = vdesc.find(p1);
			assert(tmp != vdesc.end());
			e.second = tmp->second;
		}

		// draw the edge
		std::vector<cv::Point> v = {p0, p1};
		ws->draw(v, cv::Scalar::all(128), 2);

		edges.push_back(e);
		cv::Point pt = p0 - p1;
		weights.push_back(sqrt(pt.x * pt.x + pt.y * pt.y));
	}
	boost_graph = mygraph_t(vertices.size());
	weightmap = get(edge_weight, boost_graph);
	for (std::size_t j = 0; j < edges.size(); ++j) {
		edge_descriptor e;
		bool inserted;
		boost::tie(e, inserted) = add_edge(edges[j].first, edges[j].second,
		                                   boost_graph);
		weightmap[e] = weights[j];
	}
}

void VoronoiGraph::print_graph(std::string filename) {
	std::ofstream dotfile(filename.c_str());
	boost::write_graphviz (dotfile, boost_graph);
}

std::vector<cv::Point> VoronoiGraph::find_path(const cv::Point& a,
        const cv::Point& b) {

	std::vector<cv::Point> path;
	vertex start = vdesc[a];
	vertex goal = vdesc[b];

	vector<mygraph_t::vertex_descriptor> p(nvertex);
	vector<double> d(nvertex);
	try {
		// call astar named parameter interface
		astar_search (boost_graph, start,
		              distance_heuristic<mygraph_t, double, cv::Point*>
		              (vertices.data(), goal),
		              predecessor_map(&p[0]).distance_map(&d[0]).
		              visitor(astar_goal_visitor<vertex>(goal)));
	}
	catch (found_goal fg) { // found a path to the goal
		vertex v;
		for (v = goal; p[v] != v; v = p[v]) {
			path.push_back(vertices[v]);
		}
		path.push_back(vertices[v]);
	}
	std::cout << "Length of graph_path: " << path.size() << std::endl;
	std::reverse(path.begin(), path.end());
	return path;
}

} // mplib
