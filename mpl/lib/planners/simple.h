#ifndef MPL_PLANNER_SIMPLE_H
#define MPL_PLANNER_SIMPLE_H

#include "workspace.h"
#include <opencv2/core/core.hpp>

namespace mplib {
	class SimplePlanner {
	public:
		SimplePlanner(Workspace& ws);
		std::vector<cv::Point> find_path(const cv::Point& start, const cv::Point& goal, bool draw = false);
	private:
		Workspace *ws;
		cv::Mat dt; // distance transform
		void compute_cost();
		double heuristic(const cv::Point& a, const cv::Point& b);
		std::vector<cv::Point> neighbors(cv::Point pt, cv::Point goal);
		int width, height;
	};
}
#endif
