#include "simple.h"
#include "workspace.h"
#include "util.h"

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

namespace mplib {
	SimplePlanner::SimplePlanner(Workspace& ws) {
		this->ws = &ws;
		width = ws.get_bitmap().cols;
		height = ws.get_bitmap().rows;
		compute_cost();
	}

	void SimplePlanner::compute_cost() {
		distanceTransform(ws->get_bitmap(), dt, CV_DIST_L2, 5);
		normalize(dt, dt, 0, 1, NORM_MINMAX);
		dt.convertTo(dt, CV_64F);
		dt = 1 - dt;
	}

	vector<Point> SimplePlanner::neighbors(Point pt, Point goal) {
		vector<Point> neighbors;
		for (int i = 0; i < 9; i++) {
			if (i%2 == 0) continue;
			if (i==4) continue;
			int xi = (i%3) - 1;
			int yi = (i/3) - 1;

			// use a coarse grid for searching when far away from the goal
			if (abs(goal.x-pt.x) + abs(goal.y-pt.y) > 10) {
				xi *= 3;
				yi *= 3;
			}
			else if (abs(goal.x-pt.x) + abs(goal.y-pt.y) > 5) {
				xi *= 2;
				yi *= 2;
			}

			int x = pt.x;
			int y = pt.y;
			int X = x+xi;
			int Y = y+yi;
			Point pt = Point(X,Y);
			if (X>=0 and X<width and Y>=0 and Y<width and !ws->collision(pt))
				neighbors.push_back(pt);
		}
		return neighbors;
	}

	double SimplePlanner::heuristic(const Point& a, const Point& b) {
		double d = 0.01*((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
		if (d > 50) d += 1000*dt.at<double>(a);
		return d;
	}

	vector<Point> SimplePlanner::find_path(const Point& start, const Point& goal, bool draw) {

		vector<Point> path;
		vector<Point> open;
		vector<Point> closed;
		open.push_back(start);

		Mat fCost(height, width, CV_64F), gCost(height, width, CV_64F);
		gCost += 1e10;
		fCost += 1e10;

		gCost.at<double>(start) = 0;
		fCost.at<double>(start) = heuristic(start, goal);

		Point current;
		auto cmp = [fCost](const Point& a, const Point& b){return fCost.at<double>(a) > fCost.at<double>(b);};
		map<int,int> parent;

		while (not open.empty()) {
			sort(open.begin(), open.end(), cmp);
			current = open.back(); open.pop_back();

			if (current == goal) {
				while (current != start) {
					path.push_back(current);
					int hash = parent[width*current.y + current.x];
					current = Point(hash%width, hash/width);
				}
				return path;
			}

			closed.push_back(current);
			if (draw) ws->draw(current, 2, Scalar(255,0,0), 2);

			for (Point n : neighbors(current, goal)) {
				if (is_member(closed, n)) continue;
				double tmp_g = gCost.at<double>(current) + dt.at<double>(n);
				if (is_member(open, n)) {
					if (tmp_g < gCost.at<double>(n)) {
						parent[width*n.y + n.x] = width*current.y + current.x;
						gCost.at<double>(n) = tmp_g;
						fCost.at<double>(n) = tmp_g + heuristic(n, goal);
					}
				}
				else {
					open.push_back(n);
					parent[width*n.y + n.x] = width*current.y + current.x;
					gCost.at<double>(n) = tmp_g;
					fCost.at<double>(n) = tmp_g + heuristic(n, goal);
				}
			}
		}
		// std::reverse(path.begin(), path.end());
		return path;
	}
}
