#include "viz.hpp"
#include "types.hpp"
#include <string>

namespace mpel {

size_t View::nviews = 0;

View::View(std::string name) : _name(name) {
	cv::namedWindow(_name, 0);
	update();
	nviews++;
}

void View::close() {
	clear();
	cv::destroyWindow(_name);
	update();
	nviews--;
}

void View::clear() {
	_img.release();
	cv::destroyWindow(_name);
	cv::namedWindow(_name, 0);
	update();
}

void View::update() {
	if (not _img.empty())
		cv::imshow(_name, _img);
	cv::waitKey(10);
}

void View::stay(char key) {
	while (nviews > 0 and (char)cv::waitKey(10) != key);
}

void View::add_layer(MapRef map) {
	_img = map.clone();
	cv::cvtColor(_img, _img, cv::COLOR_GRAY2BGR);
	update();
}
void View::add_layer(PointRef pt) {
	circle(_img, pt, 4, COLOR_PINK, -1);
	update();
}
void View::add_layer(ProblemDefinition pdef) {
	circle(_img, pdef.start, 4, COLOR_RED, -1);
	circle(_img, pdef.goal, 4, COLOR_BLUE, -1);
	update();
}
void View::add_layer(PathRef path) {
	for (size_t i = 0; i < path.size() - 1; ++i) {
		line(_img, path[i], path[i + 1], COLOR_GREEN, 2);
		circle(_img, path[i], 2, COLOR_DARK_GREEN, -1);
	}
	update();
}
void View::add_layer(GraphRef g) {
	for (size_t i = 0; i < g.num_vertices(); ++i) {
		for (size_t j = 0; j < i; ++j) {
			if (g.weight(i,j) > 0)
				line(_img, g.vertex(i), g.vertex(j), COLOR_GRAY, 2);
		}
	}
	update();
}

}
