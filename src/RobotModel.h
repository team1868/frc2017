#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include "CANTalon.h"
#include "AHRS.h"

class RobotModel {
public:
	RobotModel();
	~RobotModel();
	void ResetTimer();
	double GetTime();

private:
    AHRS *ahrs;             // navX MXP
	Timer *timer;
};

#endif /* SRC_ROBOTMODEL_H_ */
