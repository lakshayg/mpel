#include <mpel/builtins.hpp>
#include <vector>
#include <queue>
#include "types.hpp"
#include "common.hpp"

namespace mpel {
	namespace builtin {
namespace graph_search {
// default search
none::none() {}
Path none::operator()(GraphRef g, PointRef a, PointRef b) {
	Path path;
	path.push_back(a);
	path.push_back(b);
	return path;
}
}
}
}
