#ifndef MPEL_COMMON_H
#define MPEL_COMMON_H
#include "types.hpp"
#include <exception>

namespace mpel {

struct UnimplementedException : public std::exception {
	const char *what() const throw() {
		return "Unimplemented Functionality";
	}
};

Map load_map_from_image(std::string filename);

double distance(PointRef a, PointRef b);

bool is_collision(MapRef map, PointRef pt);

std::vector<Segment> get_map_segments(MapRef map, double eps = 10);
}
#endif
