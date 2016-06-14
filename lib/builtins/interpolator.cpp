#include "types.hpp"
#include "common.hpp"
#include <mpel/builtins.hpp>

namespace mpel {

	default_interpolator::default_interpolator() {}
	Path default_interpolator::operator()(PathRef path) {
		Path p(path.begin(), path.end());
		return p;
	}


	potential_field_interpolator::potential_field_interpolator() {
		throw UnimplementedException();
	}
	Path potential_field_interpolator::operator()(PathRef path) {
		Path p(path.begin(), path.end());
		return p;
	}
}
