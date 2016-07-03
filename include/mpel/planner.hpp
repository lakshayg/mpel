#ifndef MPEL_PLANNER_H
#define MPEL_PLANNER_H

#include "types.hpp"

namespace mpel {

class Planner {

public:

	struct Config {
		GraphBuilderFn graph_builder;
		GraphSearchFn graph_search;
		InterpolatorFn interpolator;
	};
	
	Planner(Planner::Config pc = Planner::Config());
	
	void load_workspace(const Workspace& ws);
	
	Path solve(ProblemDefinition pdef);

	GraphRef roadmap() const;
	MapRef map() const;
	
private:

	Config _pc;
	
	Graph _g;
	
	Workspace _ws;
};
}
#endif
