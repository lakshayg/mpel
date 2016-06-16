#ifndef MPEL_VIZ_H
#define MPEL_VIZ_H

#include <opencv2/opencv.hpp>

#define COLOR_BLUE cv::Scalar(255,0,0)
#define COLOR_GREEN cv::Scalar(0,255,0)
#define COLOR_RED cv::Scalar(0,0,255)
#define COLOR_DARK_GREEN cv::Scalar(0,100,0)
#define COLOR_PINK cv::Scalar(211,0,246)
#define COLOR_GRAY cv::Scalar(150,150,150)

#include "planner.hpp"
#include "types.hpp"
#include <string>

namespace mpel {
class View {
public:
	View(std::string name = "View: " + std::to_string(nviews));

	void close();

	void clear();

	void add_layer(MapRef map);
	void add_layer(PointRef pt);
	void add_layer(ProblemDefinition pdef);
	void add_layer(PathRef path);	
	void add_layer(GraphRef graph);
	void add_layer(const Planner& p);

	void save(std::string filename = "view.png");
	
	static void stay(char key = ' ');

private:
	cv::Mat _img;
	
	std::string _name;

	void update();

	static size_t nviews;
};


}
#endif
