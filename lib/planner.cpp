#include "planner.hpp"
#include "types.hpp"
#include "common.hpp"
#include <vector>
#include <functional>
#include <algorithm>

namespace mpel {

Planner::Planner(Planner::Config pc) : _pc(pc) {}

void Planner::load_workspace(const Workspace& ws) {
	_ws = ws;

	// generate the roadmap
	_g = _pc.graph_builder(_ws.map);
}

GraphRef Planner::roadmap() const {
	return _g;
}

MapRef Planner::map() const {
	return _ws.map;
}

template <typename Iter, typename Compare = std::less<typename Iter::value_type>>
std::vector<typename Iter::value_type> ksmallest(Iter begin, Iter end, size_t k, Compare comp = Compare()) {
	std::vector<typename Iter::value_type> out(begin, begin+k);
	std::make_heap(out.begin(), out.end(), comp);
	for (Iter it = begin+k; it != end; ++it) {
		if (comp(*it, out.front())) {
			std::pop_heap(out.begin(), out.end(), comp);
			out.back() = *it;
			std::push_heap(out.begin(), out.end(), comp);
		}
	}
	return out;
}

struct dcomp {
	Point _p;
	dcomp(PointRef p) : _p(p) {}
	bool operator()(PointRef a, PointRef b) {
		double dx1 = a.x - _p.x;
		double dy1 = a.y - _p.y;
		double dx2 = b.x - _p.x;
		double dy2 = b.y - _p.y;
		return dx1*dx1+dy1*dy1 < dx2*dx2+dy2*dy2;
	}
};

Path Planner::solve(ProblemDefinition pdef) {
	// find point closest to given points in graph
	if (is_collision(_ws.map, pdef.start)) return Path();
	if (is_collision(_ws.map, pdef.goal)) return Path();
	Path p;
	Point in, out;

	// find some vertices in graph closest to start and goal and connect them
	Graph tmp_g = _g;
	size_t nneigh = std::min((size_t) 5, tmp_g.vertex_list().size());
	std::vector<Point> start_neigh = ksmallest(tmp_g.vertex_list().begin(), tmp_g.vertex_list().end(), nneigh, dcomp(pdef.start));
	std::vector<Point> goal_neigh = ksmallest(tmp_g.vertex_list().begin(), tmp_g.vertex_list().end(), nneigh, dcomp(pdef.goal));
	for (size_t i = 0; i < nneigh; ++i) {
		Segment s1 = Segment(pdef.start, start_neigh[i]);
		if (not is_collision(_ws.map, s1))
			tmp_g.add_edge(pdef.start, start_neigh[i], distance(pdef.start, start_neigh[i]));

		Segment s2 = Segment(pdef.goal, goal_neigh[i]);
		if (not is_collision(_ws.map, s2))
			tmp_g.add_edge(pdef.goal, goal_neigh[i], distance(pdef.goal, goal_neigh[i]));
	}
	p = _pc.graph_search(tmp_g, pdef.start, pdef.goal);

	Path ret = _pc.interpolator(_ws.map, p);
	return ret;
}

}
