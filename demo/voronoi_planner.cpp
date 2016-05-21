#include "mpl.h"
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace mplib;

int main() {
	Workspace ws("Workspace", "../../workspace/004.bmp");
	ws.toggle_display();

	Point start = ws.mark_point();
	Point goal = ws.mark_point();

	VoronoiPlanner vroni(ws, 20);
	vector<Point> p = vroni.find_path(start, goal);

	cv::waitKey();
	return 0;
}
