#include "types.hpp"
#include "common.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <mpel/builtins.hpp>

namespace mpel {
	namespace builtin {
namespace interpolator {
a_star::a_star() {
	throw UnimplementedException();
}

/* returns the norm of a point */
double norm(PointRef p) {
	return sqrt(p.x * p.x + p.y * p.y);
}

/* return the squared euler distance between two points */
double sqr_dist(PointRef p1, PointRef p2) {
	double dx = p1.x - p2.x;
	double dy = p1.y - p2.y;
	return dx * dx + dy * dy;
}

double heuristic(PointRef pt, PointRef goal) {
	return 0;
}

Path interpolate_segment(MapRef map, PointRef a, PointRef b) {
	std::vector<Point> path, open, closed;
	open.push_back(a);
	return path;
}

Path a_star::operator()(MapRef map, PathRef _path) {
	cv::distanceTransform(map, _dt, CV_DIST_L2, 5);
	_dt.convertTo(_dt, CV_64F);
	cv::normalize(_dt, _dt, 0, 1, cv::NORM_MINMAX);	



	Path p(_path.begin(), _path.end());
	return p;
}

}
}
}
