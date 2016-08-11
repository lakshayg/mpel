#include "planner.hpp"
#include "algorithms.hpp"
#include "common.hpp"
#include "types.hpp"
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <functional>
#include <vector>

using namespace std::chrono;

namespace mpel {

Planner::Planner(Planner::Config pc)
    : _pc(pc)
{
}

void Planner::load_workspace(const Workspace& ws)
{
    _ws = ws;

    // generate the roadmap
    auto t0 = high_resolution_clock::now();
    _g = _pc.graph_builder(_ws.map);
    auto t1 = high_resolution_clock::now();
    t_graph = duration_cast<microseconds>(t1 - t0).count();
}

GraphRef Planner::roadmap() const { return _g; }

MapRef Planner::map() const { return _ws.map; }

struct dcomp {
    Point _p;
    dcomp(PointRef p)
        : _p(p)
    {
    }
    bool operator()(PointRef a, PointRef b)
    {
        double dx1 = a.x - _p.x;
        double dy1 = a.y - _p.y;
        double dx2 = b.x - _p.x;
        double dy2 = b.y - _p.y;
        return dx1 * dx1 + dy1 * dy1 < dx2 * dx2 + dy2 * dy2;
    }
};

Path Planner::solve(ProblemDefinition pdef)
{
    // find point closest to given points in graph
    if (is_collision(_ws.map, pdef.start)) return Path();
    if (is_collision(_ws.map, pdef.goal)) return Path();
    Path p;
    Point in, out;

    // find some vertices in graph close to start and goal and connect them
    Graph tmp_g = _g;
    size_t nneigh = std::min((size_t)5, tmp_g.vertex_list().size());
    std::vector<Point> start_neigh
        = k_best(tmp_g.vertex_list().begin(), tmp_g.vertex_list().end(), nneigh, dcomp(pdef.start));
    std::vector<Point> goal_neigh
        = k_best(tmp_g.vertex_list().begin(), tmp_g.vertex_list().end(), nneigh, dcomp(pdef.goal));

    size_t in_segment = 0, out_segment = 0; // check the number of edges which could be connected
    for (size_t i = 0; i < nneigh; ++i) {
        Segment s1 = Segment(pdef.start, start_neigh[i]);
        if (not is_collision(_ws.map, s1)) {
            tmp_g.add_edge(pdef.start, start_neigh[i], distance(pdef.start, start_neigh[i]));
            in_segment++;
        }

        Segment s2 = Segment(pdef.goal, goal_neigh[i]);
        if (not is_collision(_ws.map, s2)) {
            tmp_g.add_edge(pdef.goal, goal_neigh[i], distance(pdef.goal, goal_neigh[i]));
            out_segment++;
        }
    }

    auto t0 = high_resolution_clock::now();

    // check if the terminal points could not be successfully inserted
    // it may happen that they could not be inserted because the graph
    // built by graph builder is not sufficiently dense or the user did
    // not use a graph builder at all due to which the graph is empty
    if (in_segment == 0 or out_segment == 0) {
        std::cout << "[Planner::solve] Terminal points could not be inserted in the graph, "
                  << "check if the graph is sufficiently dense" << std::endl;
        // we return a straight line joining start and goal hoping that the interpolator
        // will be able to find a path. If it too cannot find a path then the returned
        // path is a straight line between start and goal points.
        p.push_back(pdef.start);
        p.push_back(pdef.goal);
    } else {
        p = _pc.graph_search(tmp_g, pdef.start, pdef.goal);
    }

    auto t1 = high_resolution_clock::now();
    t_search = duration_cast<microseconds>(t1 - t0).count();

    t0 = high_resolution_clock::now();
    Path ret = _pc.interpolator(_ws.map, p);
    t1 = high_resolution_clock::now();
    t_interp = duration_cast<microseconds>(t1 - t0).count();

    if (ret.back() != pdef.goal) { // check if path was completely interpolated
        std::cout << "[Planner::solve] Could not find a path!" << std::endl;
    }

    return ret;
}

void Planner::show_stats() const
{
    printf("Graph builder: %9.3f ms\n", 1e-3 * t_graph);
    printf("Graph search : %9.3f ms\n", 1e-3 * t_search);
    printf("Interpolation: %9.3f ms\n", 1e-3 * t_interp);
}
}
