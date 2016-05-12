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

    // Robot roby("Roby", 0);

    // SimplePlanner planner(ws);
    // vector<Point> path = planner.find_path(start, goal, true);
    // ws.draw(path, Scalar(255,0,0), 6);
    // ws.draw(path, Scalar(255,255,0), 2);

    // cout << "Complete" << endl;

    VoronoiPlanner vroni(ws, 20);
    vector<Point> p = vroni.find_path(start, goal);
    cout << p << endl;
    // ws.draw(p, Scalar(0,0,255), 5);
    // ws.draw(p, Scalar(0,255,0), 2);

    cv::waitKey();
    return 0;
}
