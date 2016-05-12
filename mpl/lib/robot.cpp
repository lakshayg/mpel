#include "robot.h"
#include <string>

using namespace std;

namespace mplib {
	Robot::Robot(const string& name, double radius) {
		this->name = name;
		this->radius = radius;
	}
}
