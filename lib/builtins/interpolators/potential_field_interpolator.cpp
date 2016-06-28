#include "types.hpp"
#include "common.hpp"
#include <cmath>
#include <opencv2/opencv.hpp>
#include <mpel/builtins.hpp>

namespace mpel {

potential_field_interpolator::potential_field_interpolator() {
	//throw UnimplementedException();
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

/* construct the attractive potential at every location in the grid
 * rows, cols denotes the size of the workspace
 * goal denotes the point where we wish to reach
 * c is the constant scaling factor used for the field f(x) = c * ||x - xg||^2
 */
cv::Mat attractive_potential(size_t rows, size_t cols, Point goal, double c = 250) {
	cv::Mat pot = cv::Mat::zeros(rows, cols, CV_64FC1);
	for (size_t i = 0; i < rows; ++i) {
		for (size_t j = 0; j < cols; ++j) {
			Point pt(j, i);
			pot.at<double>(pt) = sqr_dist(goal, pt);
		}
	}
	cv::normalize(pot, pot, 0, c, cv::NORM_MINMAX);
	return pot;
}

/* repulsive field due to obstacles in the field
 * m is the 2D map of the workspace
 * d is the distance parameter used in constructing the repulsive potential
 * c is a constant scaling factor
 */
cv::Mat repulsive_potential(MapRef m, double d = 250, double c = 250) {
	// compute the distance transform of this image
	cv::Mat dt;
	cv::distanceTransform(m, dt, CV_DIST_L2, CV_DIST_MASK_PRECISE);
	dt.convertTo(dt, CV_64F);

	// compute the following function for repulsive potential
	//              /  1      1 \ 2
	// f(x) =  c * | ----- - --- |   for r(x) < d0
	//              \ r(x)    d /

	cv::Mat pot = cv::Mat::zeros(m.rows, m.cols, CV_64FC1);

	for (int i = 0; i < m.rows; ++i) {
		for (int j = 0; j < m.cols; ++j) {
			Point pt(j, i);
			double dist = dt.at<double>(pt);
			if (dist < d) {
				double p = (1./(dist+1) - 1./d) * (1./(dist+1) - 1./d);
				if (p > c) p = c;
				pot.at<double>(pt) = p;
			}
		}
	}
	cv::normalize(pot, pot, 0, c, cv::NORM_MINMAX);
	return pot;
}

/* return the direction of max descent in the given matrix
 * at the given location. The return value is a point of the
 * form [dx, dy] where dx and dy are the displacements in the
 * x and y directions respectively. n specifies the size of
 * the square neighborhood to consider
 */
Point max_descent_direction(cv::Mat m, PointRef pt, int n = 15) {
	double min_grad = 0;
	Point dir = Point(0, 0);
	double val = m.at<double>(pt);
	for (int i = -n; i <= n; ++i) {
		for (int j = -n; j <= n; ++j) {
			Point tmp = Point(j,i);
			double grad = (m.at<double>(pt+tmp) - val)/norm(tmp);
			if (grad <= min_grad) {
				min_grad = grad;
				dir = tmp;
			}
		}
	}
	return dir;
}

/* interpolate a single segment in the path using attraction and
 * repulsion terms provided as arguments to the function. The parameter
 * d specifies how close we need to get to the final goal.
 */
Path interpolate_segment(Point in, Point out, const cv::Mat& attr, const cv::Mat& rep, double d = 10) {
	Path p;
	cv::Mat pot_field = attr + rep;
	Point curr = in;
	p.push_back(curr);
	while (sqr_dist(curr, out) > d*d) {
		Point dir = max_descent_direction(pot_field, curr);
		/* highly improbable but print a debug message just in case
		 * we get stuck in a local minima and can't decide on the direction
		 * returns the partially interpolated path
		 */
		if (dir.x == 0 and dir.y == 0) {
			std::cout << "Stuck in a local minima :O" << std::endl;
			p.push_back(out);
			return p;
		}
		p.push_back(curr + dir);
		curr = curr + dir;
	}
	return p;
}

Path potential_field_interpolator::operator()(MapRef map, PathRef _path) {
	Path path = subdivide_path(_path, 80);
	Path p;
	Point curr = path.front();
	cv::Mat rep = repulsive_potential(map);
	/* interpolate all the segments in the path and return the result */
	for (size_t i = 1; i < path.size(); ++i) {
		Path tmp;
		Point goal = path[i];
		cv::Mat attr = attractive_potential(map.rows, map.cols, goal);
		if (i == path.size() - 1)
			tmp = interpolate_segment(curr, goal, attr, rep);
		else
			tmp = interpolate_segment(curr, goal, attr, rep, 3);
		p.insert(p.end(), tmp.begin(), tmp.end());
		curr = p.back();
	}
	p.push_back(_path.back());

	return p;
}

}
