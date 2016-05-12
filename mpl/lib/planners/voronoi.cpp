#include "voronoi.h"
#include "workspace.h"
#include "simple.h"
#include "util.h"
#include "voronoi_graph.h"

#include <boost/polygon/voronoi.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <memory>

using namespace cv;
using namespace std;
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::voronoi_visual_utils;

typedef voronoi_diagram<double>::vertex_type& voronoi_vertex;
typedef voronoi_diagram<double>::edge_type& voronoi_edge;
typedef voronoi_diagram<double>::vertex_type point_type;
// typedef voronoi_diagram<double>::edge_type& voronoi_edge;

namespace mplib {
VoronoiPlanner::VoronoiPlanner(Workspace& ws, double eps) {
	this->ws = &ws;
	EPS = eps;
	construct_roadmap();
}

void VoronoiPlanner::construct_roadmap() {
	vector<Segment> segments = get_entities();
	construct_voronoi(segments.begin(), segments.end(), &vd);
	vg = std::unique_ptr<VoronoiGraph>(new VoronoiGraph(ws, vd));
	// vg->print_graph("voronoi.dot");
}

vector<Segment> VoronoiPlanner::get_entities() {
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(ws->get_bitmap(), contours, hierarchy,
	             CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	vector<vector<Point>> approx;

	for (size_t i = 0; i < contours.size(); i++) {
		vector<Point> tmp;
		approxPolyDP(contours[i], tmp, EPS, true);
		approx.push_back(tmp);
	}
	vector<Segment> segments;

	for (size_t i = 0; i < approx.size(); i++) {
		Point prev = Point(-10, -10);
		int len = approx[i].size();
		if (len > 1) {
			prev = approx[i][0];
			for (int j = 1; j < len; j++) {
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

vector<Point> VoronoiPlanner::find_path(const Point& start, const Point& goal) {
	// auto dist = [](const Point& a, const Point& b){return abs(a.x-b.x) + abs(a.y-b.y);};
	voronoi_diagram<double>::const_vertex_iterator it;

	// reset the color parameter of vd [required if the user will perform multiple queries]
	// find point closest to start in voronoi diagram [in]
	// find point closest to goal in voronoi diagram [out]
	Point in, out;
	// int in_idx, out_idx;
	double in_d = 1e10, out_d = 1e10;

	for (size_t i = 0; i < vd.vertices().size(); i++) {
		auto it = &vd.vertices()[i];
		Point pt = Point((int) it->x(), (int) it->y());
		ws->draw(pt);

		// cout << "Voronoi vertex: " << pt << endl;
		if (ws->collision(pt)) {
			// cout << "Collision!!!" << endl;
			continue;
		}

		double d0 = distance(start, pt);
		double d1 = distance(goal, pt);
		double d;

		d = d0;
		// cout << "d: " << d << endl;
		if (d < in_d) {
			// in_idx = i;
			in = pt;
			in_d = d;
		}
		d = d1;
		if (d < out_d) {
			// out_idx = i;
			out = pt;
			out_d = d;
		}
	}

	// find path from start to in
	// find path from out to goal
	SimplePlanner p(*ws);
	vector<Point> p0 = {start, in}; //p.find_path(start, in);

	// search the voronoi diagram for path between in and out
	vector<Point> p1 = vg->find_path(in, out);
	std::cout << "Length of path found in VoronoiGraph: " << p1.size() << std::endl;
	vector<Point> p2 = {out, goal}; //p.find_path(out, goal);

	// output the complete path as vector<Point>
	vector<Point> path;
	path.reserve(p0.size() + p1.size() + p2.size());
	path.insert(path.end(), p0.begin(), p0.end());
	path.insert(path.end(), p1.begin(), p1.end());
	path.insert(path.end(), p2.begin(), p2.end());

	ws->draw(p0, Scalar(0, 0, 255), 5);
	ws->draw(p1, Scalar(0, 255, 0), 5);
	ws->draw(p2, Scalar(255, 0, 0), 5);

	return path;
}

void VoronoiPlanner::update() {
	construct_roadmap();
}
}
