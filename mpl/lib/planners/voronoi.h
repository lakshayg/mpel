#ifndef MPL_PLANNER_VORONOI_H
#define MPL_PLANNER_VORONOI_H

#include "workspace.h"
#include "voronoi_boost.h"
#include "voronoi_graph.h"
#include <memory>
#include <opencv2/opencv.hpp>
#include <boost/polygon/voronoi.hpp>
#include <unordered_map>

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

namespace mplib {
	class VoronoiPlanner {
	public:
		VoronoiPlanner(Workspace& ws, double eps = 10);
		void update();
		std::vector<cv::Point> find_path(const cv::Point& a, const cv::Point& b);

	private:
		double EPS; // parameter for approximating obstacles as
		            // polygons when constructing voronoi roadmap
		Workspace *ws;
		voronoi_diagram<double> vd;

		std::unique_ptr<VoronoiGraph> vg;

		std::vector<Segment> get_entities();
		void construct_roadmap();
	};
}

#endif
