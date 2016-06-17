#include <mpel/builtins.hpp>
#include <vector>
#include <queue>
#include "types.hpp"
#include "common.hpp"

namespace mpel {

// default search
default_search::default_search() {}
Path default_search::operator()(GraphRef g, PointRef a, PointRef b) {
	Path path;
	path.push_back(a);
	path.push_back(b);
	return path;
}
}
