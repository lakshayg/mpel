#include "common.hpp"
#include "types.hpp"
#include <mpel/builtins.hpp>

namespace mpel {
namespace builtin {
    namespace interpolator {
        none::none() {}
        Path none::operator()(MapRef map, PathRef path)
        {
            Path p(path.begin(), path.end());
            return p;
        }
    }
}
}
