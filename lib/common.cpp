#include "common.hpp"
#include "types.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <random>

std::random_device _mpel_random_device;

namespace mpel {

Map load_map_from_image(std::string filename) {
	cv::Mat im = cv::imread(filename, 0);
	cv::threshold(im, im, 125, 255, cv::THRESH_BINARY);
	return im;
}

double distance(PointRef a, PointRef b) {
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return sqrt(dx * dx + dy * dy);
}

double distance(PointRef p, SegmentRef s) {
	double len = distance(s.p0, s.p1);
	double cross = p.x*(s.p1.y-s.p0.y) - p.y*(s.p1.x-s.p0.x) + \
				   (s.p1.x*s.p0.y - s.p0.x*s.p1.y);
	return std::abs(cross/len);
}

double distance(SegmentRef s, PointRef p) {
	return distance(p, s);
}

bool on_segment(SegmentRef s, PointRef p) {
	double d1 = distance(s.p0, p);
	double d2 = distance(p, s.p1);
	double d  = distance(s.p0, s.p1);
	double delta = std::abs(d1 + d2 - d);
	return delta < 3.;
}

bool is_collision(MapRef map, PointRef pt) {
	if (pt.x >= map.cols or pt.x < 0) return true;
	if (pt.y >= map.rows or pt.y < 0) return true;
	return map.at<uchar>(pt) < 250;
}

bool is_collision(MapRef map, SegmentRef s) {
	if (is_collision(map, s.p0)) return true;
	if (is_collision(map, s.p1)) return true;
	double len = distance(s.p0, s.p1);
	double dx = (s.p1.x - s.p0.x) / len;
	double dy = (s.p1.y - s.p0.y) / len;
	for (double r = 0; r <= len; r += 4) {
		Point p(s.p0.x + dx * r, s.p0.y + dy * r);
		if (is_collision(map, p)) return true;
	}
	return false;
}

// return a random point from the free space
Point random_free_space_point(const Map& map) {
	std::uniform_int_distribution<int> randx(0, map.rows);
	std::uniform_int_distribution<int> randy(0, map.cols);
	Point pt;
	do {
		pt.x = randx(_mpel_random_device);
		pt.y = randy(_mpel_random_device);
	} while (is_collision(map, pt));
	return pt;
}

Path div(PathRef path) {
	Path p;
	for (size_t i = 0; i < path.size() - 1; ++i) {
		double len = distance(path[i], path[i+1]);
		double dx = (path[i+1].x - path[i].x)/len;
		double dy = (path[i+1].y - path[i].y)/len;
		double t = 0.0;
		while (t <= len) {
			p.push_back(path[i] + Point(dx*t,dy*t));
			t = t + 1.;
		}
	}
	p.push_back(path.back());
	return p;
}
Path subdivide_path(PathRef path, double len) {
	Path p = div(path);
	Path ans; ans.push_back(p.front());
	for (auto pt : p) {
		if (distance(pt, ans.back()) > len)
			ans.push_back(pt);
	}
	if (ans.back() != path.back())
		ans.push_back(path.back());
	return ans;
}

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

// mark point in a workspace
Point marked_point = Point(-1,-1);
void callback(int event, int x, int y, int flags, void * userdata) {
	if (event == cv::EVENT_LBUTTONDOWN) {
		marked_point = Point(x, y);
	}
}
Point mark_point(const Workspace& ws) {
	std::string name = "Mark Point";
	cv::namedWindow(name);
	cv::imshow(name, ws.map);
	marked_point = Point(-1, -1);
	cv::setMouseCallback(name, callback, NULL);
	while (marked_point == Point(-1, -1)) cv::waitKey(10);
	cv::setMouseCallback(name, NULL, NULL);
	cv::destroyWindow(name);
	return marked_point;
}
}
