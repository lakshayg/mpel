#ifndef MPL_WORKSPACE_H
#define MPL_WORKSPACE_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

namespace mplib {
class Workspace {
public:
	Workspace(const std::string& name);
	Workspace(const std::string& name, const std::string& filename);
	Workspace(const std::string& name, Workspace& ws);

	~Workspace();

	// the workspace display is turned off initially, use this
	// method to toggle the display state
	void toggle_display();

	// load workspace information from a file
	void read(const std::string& filename);

	void draw(const cv::Point& pt, int radius = 3,
	          const cv::Scalar& color = cv::Scalar(0,0,0), int thickness = 3);
	void draw(const std::vector<cv::Point>& path,
	          const cv::Scalar& color = cv::Scalar(0,0,0), int thickness = 3);

	cv::Mat get_bitmap();
	bool collision(const cv::Point& a);
	void clear();
	cv::Point mark_point();
private:
	cv::Mat bitmap;
	cv::Mat disp;
	std::string name;
	bool display;
	void update_display();
};
}

#endif
