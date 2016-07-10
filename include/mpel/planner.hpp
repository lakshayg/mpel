/**
 * \file planner.hpp
 * \brief This file defines the planner class.
 *
 * The planner class provides a skeleton for building the motion
 * planning algorithm. This class uses the provided components
 * for generating paths. New planners can be made either by changing
 * the input parameters or by deriving another class from this class
 */
#ifndef MPEL_PLANNER_H
#define MPEL_PLANNER_H

#include "types.hpp"

namespace mpel {

class Planner {

public:
    /// Paramters for planner configuration
    struct Config {
        GraphBuilderFn graph_builder;
        GraphSearchFn graph_search;
        InterpolatorFn interpolator;
    };

    Planner(Planner::Config pc = Planner::Config());

    /// Load a workspace into the planner
    virtual void load_workspace(const Workspace& ws);

    /// Problems are provided using a ProblemDefinition struct
    virtual Path solve(ProblemDefinition pdef);

    /// Returns the graph generated by graph builder
    virtual GraphRef roadmap() const;

    /// Returns the map that is a part of the workspace
    virtual MapRef map() const;

private:
    Config _pc;

    Graph _g;

    Workspace _ws;
};
}
#endif
