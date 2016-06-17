#include "types.hpp"
#include "common.hpp"
#include <mpel/builtins.hpp>

namespace mpel {

default_interpolator::default_interpolator() {}
Path default_interpolator::operator()(MapRef map, PathRef path) {
	Path p(path.begin(), path.end());
	return p;
}

}
