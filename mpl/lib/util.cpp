#include <opencv2/opencv.hpp>
#include <cmath>
#include "util.h"
#include <algorithm>

using namespace std;

double distance_manhattan(const_pt a, const_pt b) {return abs(a.x-b.x) + abs(a.y-b.y);}
double distance_euclid(const_pt a, const_pt b) {const_pt c = a-b; return sqrtf(c.x*c.x-c.y*c.y);}
double distance_euclid_sq(const_pt a, const_pt b) {const_pt c = a-b; return c.x*c.x - c.y*c.y;}
double distance_chebychev(const_pt a, const_pt b) {return max(abs(a.x-b.x), abs(a.y-b.y));}
double distance(const_pt a, const_pt b) {return distance_manhattan(a, b);}
