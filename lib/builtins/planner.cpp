#include "planner.hpp"
#include "types.hpp"
#include <mpel/builtins.hpp>

namespace mpel {
namespace builtin {
    voronoi_planner_config::voronoi_planner_config()
    {
        graph_builder = graph_builder::voronoi();
        graph_search = graph_search::dijkstra();
        interpolator = interpolator::none();
    }

    PRM_planner_config::PRM_planner_config()
    {
        graph_builder = graph_builder::probabilistic();
        graph_search = graph_search::dijkstra();
        interpolator = interpolator::none();
    }
}
}
