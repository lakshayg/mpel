#include "common.hpp"
#include "types.hpp"
#include <cassert>
#include <iostream>
#include <mpel/builtins.hpp>
#include <opencv2/opencv.hpp>
#define LOG(x) {std::cout << #x": " << x << std::endl;}


/*
 * The Bug2 interpolator is meant to be used with the
 * default_graph_builder and default_graph_search ONLY
 * You may use it with other components as well but it
 * may give strange result sometimes.
 */

double distance_pc(std::vector<mpel::Point> cnt, mpel::PointRef p) {
	assert(cnt.size() > 0);
	double d = std::numeric_limits<double>::max();
	for (auto& e : cnt) {
		d = std::min(d, mpel::distance(e,p));
	}
	return d;
}

size_t nearest_contour_idx(std::vector<std::vector<mpel::Point> >& cnts, mpel::PointRef p) {
	assert(cnts.size() > 0);
	double d = std::numeric_limits<double>::max();	
	size_t idx = 0;
	for(size_t i = 0; i < cnts.size(); ++i) {
		double dist = distance_pc(cnts[i], p);
		if (dist < d) {
			d = dist;
			idx = i;
		}
	}
	return idx;
}

size_t nearest_point_idx(std::vector<mpel::Point>& cnt, mpel::PointRef p) {
	assert(cnt.size() > 0);
	size_t idx = 0;
	double d = std::numeric_limits<double>::max();
	for (size_t i = 0; i < cnt.size(); ++i) {
		double dist = mpel::distance(cnt[i], p);
		if (dist < d) {
			d = dist;
			idx = i;
		}
	}
	return idx;
}

namespace mpel {
	namespace builtin { namespace interpolator {

bug2::bug2(double step) {
  _step = step;
			}

			Path bug2::operator()(MapRef map, PathRef path) {
				// sanity checks
				for (auto& e : path)
					assert(not is_collision(map, e));
				assert(path.size() > 1);
				if (path.size() != 2)
					std::cerr << "Are you sure you wanted to use the bug2_interpolator?" << std::endl;

				Path p; // path of the bug
				cv::Mat m = map.clone();

				std::vector<std::vector<Point> > contours;
				cv::findContours(m, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

				for (size_t i = 0; i < path.size() - 1; ++i) {
					Point p0 = path[i];
                                        Point p1 = path[i + 1];
                                        double len = distance(p0, p1);
					double dx = (p1.x - p0.x)/len;
					double dy = (p1.y - p0.y)/len;
					double l = 0;

					// bug moves from p0 to p1
					Point bug = p0;
					p.push_back(bug);
					while (distance(bug, p1) > _step) {
						l = distance(bug, p0);
						while (not is_collision(map, bug) and l < len) {
							// move till the bug reaches a wall
							l = l + _step;
							bug = Point(p0.x + dx*l, p0.y + dy*l);
						}

						p.push_back(bug);
						if (l >= len) { break; }

						// bug has reached the wall
						// 1. find the nearest point on the contour and push it in the path vector
						// 2. follow the contour till bug reaches the m-line again
						double thresh = 20.;
						size_t wall_idx = nearest_contour_idx(contours, bug);
						size_t  bug_idx = nearest_point_idx(contours[wall_idx], bug);
						Point tmp = contours[wall_idx][bug_idx];
						//Path asdf;
						do {
							bug = contours[wall_idx][bug_idx];
							p.push_back(bug);
							bug_idx = (bug_idx + 1)%contours[wall_idx].size();
						} while (distance(tmp, bug) < thresh or // ensure that the bug has moved some distance
								not on_segment(Segment(p0,p1), bug)); // and bug is not back on the m-line
					}
				}
				return p;
			}
		}
	}
}
