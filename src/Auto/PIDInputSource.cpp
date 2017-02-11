/*
 * PIDInputSource.cpp
 *
 *  Created on: Jan 31, 2017
 *      Author: Lynn D
 */

#include <Auto/PIDInputSource.h>

NavxPIDSource::NavxPIDSource(RobotModel *robot) {
	robot_ = robot;
	ResetAccumulatedYaw();
}

double NavxPIDSource::PIDGet() {
	CalculateAccumulatedYaw();
	return accumulatedYaw_;
}

double NavxPIDSource::CalculateAccumulatedYaw() {
	lastYaw_ = currYaw_;
	currYaw_ = robot_->GetNavxYaw();
	deltaYaw_ = currYaw_ - lastYaw_;

	if (deltaYaw_ < -180) {			// going clockwise (from 180 to -180)
		accumulatedYaw_ += (180 - lastYaw_) + (180 + currYaw_);
	} else if (deltaYaw_ > 180) {	// going counterclockwise (from -180 to 180)
		accumulatedYaw_ -= (180 + lastYaw_) + (180 - currYaw_);
	} else {
		accumulatedYaw_ += deltaYaw_;
	}

	//SmartDashboard::PutNumber("Accumulated Angle", accumulatedYaw_);
	return accumulatedYaw_;
}

void NavxPIDSource::ResetAccumulatedYaw() {
	accumulatedYaw_ = 0.0;
	currYaw_ = robot_->GetNavxYaw();
	lastYaw_ = currYaw_;
	deltaYaw_ = 0.0;
}
