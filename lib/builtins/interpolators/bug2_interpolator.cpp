#include "types.hpp"
#include "common.hpp"
#include <opencv2/opencv.hpp>
#include <mpel/builtins.hpp>
#include <cassert>

namespace mpel {

	bug2_interpolator::bug2_interpolator(double step) {
		_step = step;
		//throw UnimplementedException();
	}

	// distance between a point and contour
	double dist_pc(const std::vector<Point>& cnt, PointRef p) {
		double dist = std::numeric_limits<double>::max();
		for (auto& e : cnt) {
			dist = std::min(dist, distance(e, p));
		}
		return dist;
	}

	// index of the contour nearest to the given point
	size_t nearest_contour_idx(const std::vector<std::vector<Point> >& cnt, PointRef p) {
		double min_dist = std::numeric_limits<double>::max();
		size_t min_idx = -1;
		for(size_t i = 0; i < cnt.size(); ++i) {
			double d = dist_pc(cnt[i], p);
			if (d < min_dist) {
				min_dist = d;
				min_idx = i;
			}
		}
		return min_idx;
	}

	// find the path joining points a and b by using contours
	Path join(PointRef a, PointRef b, const std::vector<std::vector<Point> >& cnt) {
		Path p;
		size_t idx = nearest_contour_idx(cnt, a);
		std::vector<Point> wall = cnt[idx];

		double min_dist_a = std::numeric_limits<double>::max();
		double min_dist_b = std::numeric_limits<double>::max();
		size_t idx_a, idx_b;

		for (size_t i = 0; i < wall.size(); ++i) {
			double da = distance(a, wall[i]);
			double db = distance(b, wall[i]);
			if (da < min_dist_a) {
				idx_a = i;
				min_dist_a = da;
			}
			if (db < min_dist_b) {
				idx_b = i;
				min_dist_b = db;
			}
		}

		//if (std::abs((int)idx_a - (int)idx_b) > wall.size()/2)
			for (size_t i = idx_a; i != idx_b; i = (i+1)%wall.size())
				p.push_back(wall[i]);
		//else
			//for (size_t i = idx_a; i != idx_b; i = (i-1)%wall.size())
				//p.push_back(wall[i]);

		return p;
	}


	Path bug2_interpolator::operator()(MapRef map, PathRef path) {
		Path p; p.push_back(path.front());

		// calculate contours of the map
		cv::Mat m = map.clone();
		std::vector<std::vector<Point> > contours;
		cv::findContours(m, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

		for (size_t i = 0; i < path.size() - 1; ++i) {
			Segment s(path[i], path[i+1]);
			Point p0(-1,-1), p1(-1,-1);

			// traverse the segment and find colliding parts
			//std::vector<Point> cseg; // parts of the segment that are colliding
			double len = distance(s.p0, s.p1);
			double dx = (s.p1.x - s.p0.x)/len;
			double dy = (s.p1.y - s.p0.y)/len;
			double l = 0;
			while (l < len) {
				Point pt(s.p0.x + l * dx, s.p0.y + l * dy);
				while ((l < len) and (not is_collision(map, pt))) {
					l += _step;
					pt = Point(s.p0.x + l * dx, s.p0.y + l * dy);
				}
				if (pt != s.p1) {
					p0 = pt;
					//cseg.push_back(pt);
				}
				while (l < len and is_collision(map, pt)) {
					l += _step;
					pt = Point(s.p0.x + l * dx, s.p0.y + l * dy);
				}
				if (pt != s.p1) {
					p1 = pt;
					//cseg.push_back(pt);
				}

				if (p0 != Point(-1,-1) and p1 != Point(-1,-1)) {
					Path tmp = join(p0, p1, contours);
					p.insert(p.end(), tmp.begin(), tmp.end());
				}
			}
		}
		p.push_back(path.back());
		return p;
	}
}

