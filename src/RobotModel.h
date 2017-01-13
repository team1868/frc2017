#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"

class RobotModel {
public:
	RobotModel();
	~RobotModel();
	void ResetTimer();
	double GetTime();

private:
	Timer *timer;
};

#endif /* SRC_ROBOTMODEL_H_ */
