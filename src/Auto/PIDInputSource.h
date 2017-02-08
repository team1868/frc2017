/*
   * PIDInputSource.h
 *
 *  Created on: Jan 31, 2017
 *      Author: Lynn D
 */

#ifndef SRC_PIDINPUTSOURCE_H_
#define SRC_PIDINPUTSOURCE_H_

#include "RobotModel.h"

class NavxPIDSource : public frc::PIDSource {
public:
	NavxPIDSource(RobotModel *robot);
	double PIDGet();
	double CalculateAccumulatedYaw();
	void ResetAccumulatedYaw();
	~NavxPIDSource();
private:
	double currYaw_, lastYaw_, deltaYaw_, accumulatedYaw_;
	RobotModel *robot_;
};

#endif /* SRC_PIDINPUTSOURCE_H_ */
