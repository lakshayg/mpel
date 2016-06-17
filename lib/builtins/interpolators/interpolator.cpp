#include "types.hpp"
#include "common.hpp"
#include <mpel/builtins.hpp>

namespace mpel {

potential_field_interpolator::potential_field_interpolator() {
	throw UnimplementedException();
}
Path potential_field_interpolator::operator()(MapRef map, PathRef path) {
	Path p(path.begin(), path.end());
	return p;
}
}
