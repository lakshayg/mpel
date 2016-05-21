#ifndef MPL_UTIL_H
#define MPL_UTIL_H

#include <opencv2/opencv.hpp>
#include <cmath>

// check if a STL container [v] contains a given element [p]
#define is_member(v, p) (!v.empty() && (find(v.begin(), v.end(), p) != v.end()))
// #define max(a, b) ((a)>(b) ? (a):(b))
// #define min(a, b) ((a)<(b) ? (a):(b))
#define abs(x) ((x) < 0 ? -(x):(x))
#define sqr(x) ((x)*(x))

typedef const cv::Point& const_pt;

double distance_manhattan(const_pt, const_pt);
double distance_euclid(const_pt, const_pt);
double distance_euclid_sq(const_pt, const_pt);
double distance_chebychev(const_pt, const_pt);
double distance(const_pt, const_pt); // same as manhattan distance

#endif
