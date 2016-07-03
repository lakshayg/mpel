#ifndef MPEL_COMMON_H
#define MPEL_COMMON_H
#include "types.hpp"
#include <exception>
#include <opencv2/opencv.hpp>

namespace mpel {

struct UnimplementedException : public std::exception {
	const char *what() const throw() {
		return "Unimplemented Functionality";
	}
};

Map load_map_from_image(std::string filename);

double distance(PointRef a, PointRef b);
double distance(SegmentRef s, PointRef a);
double distance(PointRef a, SegmentRef s);

bool is_collision(MapRef map, PointRef pt);
bool is_collision(MapRef map, SegmentRef s);

bool on_segment(SegmentRef s, PointRef p);

std::vector<Segment> get_map_segments(MapRef map, double eps = 10);

Path subdivide_path(PathRef path, double len = 40);

Point random_free_space_point(const Map& map);
Point mark_point(const Workspace& ws);
}
#endif
