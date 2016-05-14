#ifndef MPL_ROBOT_H
#define MPL_ROBOT_H

#include <string>

namespace mplib {
class Robot {
public:
	Robot(const std::string& name, double radius = 0);
	double radius;
private:
	std::string name;
};
}
#endif
