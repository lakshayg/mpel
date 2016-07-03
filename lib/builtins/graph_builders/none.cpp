#include "types.hpp"
#include <mpel/builtins.hpp>

namespace mpel {
namespace builtin {
    namespace graph_builder {
        /*
         * This graph builder does nothing on itself, it is
         * provided so that planners which only use the local
         * information for planning can be made without affecting
         * the general procedure followed when using this library
         * Use default_graph_builder and use an interpolator as planner
        */
        none::none() {}
        Graph none::operator()(MapRef map)
        {
            Graph g;
            return g;
        }
    }
}
}
