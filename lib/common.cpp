#include "common.hpp"
#include "types.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace mpel {

Map load_map_from_image(std::string filename) {	return cv::imread(filename, 0); }

double distance(PointRef a, PointRef b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(dx * dx + dy * dy);
}

bool is_collision(MapRef map, PointRef pt) { return map.at<uchar>(pt) < 100; }


std::vector<Segment> get_map_segments(MapRef map, double eps) {
	// eps is for approximating contours
	Map m = map.clone();
	std::vector<std::vector<Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(m, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

	std::vector<std::vector<Point> > approx;
	for (size_t i = 0; i < contours.size(); ++i) {
		std::vector<Point> tmp;
		cv::approxPolyDP(contours[i], tmp, eps, true);
		approx.push_back(tmp);
	}

	std::vector<Segment> segments;
	for (size_t i = 0; i < approx.size(); ++i) {
		Point prev = Point(-10, -10);
		int len = approx[i].size();
		if (len > 1) {
			prev = approx[i][0];
			for (int j = 1; j < len; ++j) {
				Point curr = approx[i][j];
				if (distance(prev, curr) < 3) continue;
				segments.push_back(Segment(prev, curr));
				prev = curr;
			}
			segments.push_back(Segment(prev, approx[i][0])); // close the curve
		}
	}
	return segments;
}


}
