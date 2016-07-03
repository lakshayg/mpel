#include "common.hpp"
#include "types.hpp"
#include <mpel/builtins.hpp>
#include <queue>
#include <vector>

namespace mpel {
namespace builtin {
    namespace graph_search {
        // default search
        none::none() {}
        Path none::operator()(GraphRef g, PointRef a, PointRef b)
        {
            Path path;
            path.push_back(a);
            path.push_back(b);
            return path;
        }
    }
}
}
