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

bool is_collision(MapRef map, PointRef pt);
bool is_collision(MapRef map, SegmentRef s);

std::vector<Segment> get_map_segments(MapRef map, double eps = 10);

static struct _mark_point {
public:
	static Point marked_point;
	Point operator()(const Workspace& ws);
private:
	static void callback(int event, int x, int y, int flags, void * userdata);
} mark_point;

}
#endif
