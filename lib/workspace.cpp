#include "workspace.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

Point marked_point;

void callback(int event, int x, int y, int flags, void * userdata) {
	if (event == EVENT_LBUTTONDOWN) {
		marked_point = Point(x, y);
	}
}

namespace mplib {

Workspace::Workspace(const std::string& name) {
	this->name = name;
}

Workspace::Workspace(const std::string& name, const std::string& filename) {
	this->name = name;
	bitmap = imread(filename, 0);
	threshold(bitmap, bitmap, 100, 255, THRESH_BINARY);
	cvtColor(bitmap, disp, COLOR_GRAY2BGR);
}

Workspace::Workspace(const std::string& name, Workspace& ws) {
	this->name = name;
	bitmap = ws.get_bitmap();
	cvtColor(bitmap, disp, COLOR_GRAY2BGR);
}

Workspace::~Workspace() {
	bitmap.release();
	disp.release();
}

void Workspace::toggle_display() {
	display = not display;
	if (display) {
		namedWindow(name);
		update_display();
	}
	else {
		destroyWindow(name);
	}
}


void Workspace::read(const std::string& filename) {
	bitmap = imread(filename, 0);
	cvtColor(bitmap, disp, COLOR_GRAY2BGR);
}

void Workspace::draw(const Point& pt, int radius, const Scalar& color,
                     int thickness) {
	circle(disp, pt, radius, color, thickness);
	update_display();
}

void Workspace::draw(const vector<Point>& path, const Scalar& color,
                     int thickness) {
	if (path.empty()) return;
	for (auto it = path.begin(); it < path.end() - 1; ++it) {
		line(this->disp, *it, *(it+1), color, thickness);
	}
	update_display();
}

Mat Workspace::get_bitmap() {
	return bitmap;
}

void Workspace::update_display() {
	if (display) {
		imshow(name, disp);
		waitKey(1);
	}
}

bool Workspace::collision(const cv::Point& a) {
	cv::Rect r(cv::Point(0,0),bitmap.size());
	return (r.contains(a)) && (bitmap.at<uchar>(a) == 0);
}

void Workspace::clear() {
	cvtColor(bitmap, disp, COLOR_GRAY2BGR);
	update_display();
}

Point Workspace::mark_point() {
	marked_point = Point(-1,-1);
	setMouseCallback(name, callback, NULL);
	while (marked_point == Point(-1,-1))
		waitKey(10);
	setMouseCallback(name, NULL, NULL);
	Scalar color(255,0,0);
	if (collision(marked_point)) color = Scalar(0,0,255);
	draw(marked_point);
	return marked_point;
}
}
