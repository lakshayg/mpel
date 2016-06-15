#include "planner.hpp"
#include "types.hpp"
#include "common.hpp"

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

	Path Planner::solve(ProblemDefinition pdef) {
		// find point closest to given points in graph
		Point in, out;
		double in_dist  = std::numeric_limits<double>::max();
		double out_dist = std::numeric_limits<double>::max();
		for (auto it = _g.vertex_list().begin(); it != _g.vertex_list().end(); ++it) {
			double d_in  = distance(*it,pdef.start);
			double d_out = distance(*it,pdef.goal);
			if (d_in < in_dist) {in = *it; in_dist = d_in; }
			if (d_out < out_dist) {out = *it; out_dist = d_out; }
		}

		Path p;
		p = _pc.graph_search(_g, in, out);
		//p.push_front(pdef.start);
		//p.push_back(pdef.goal);
		return p;
	}
	
}
