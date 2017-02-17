/*
   * PIDInputSource.h
 *
 *  Created on: Jan 31, 2017
 *      Author: Lynn D
 */

#ifndef SRC_PIDINPUTSOURCE_H_
#define SRC_PIDINPUTSOURCE_H_

#include "RobotModel.h"
#include "WPILib.h"

/*---------------------- NAVX PID SOURCE ---------------------- */

class NavXPIDSource : public frc::PIDSource {
public:
	/**
	 * Assigns robot and also resets accumulated yaw
	 * @param robot a RobotModel
	 */
	NavXPIDSource(RobotModel *robot);
	/**
	 * Calculates and returns AccumulatedYaw
	 * @return AccumulatedYaw
	 */
	double PIDGet();
	/**
 	 * Updates currYaw, calculates deltaYaw, calculates accumulatedYaw.
 	 * @return AccumulatedYaw
	 */
	double CalculateAccumulatedYaw();
	/**
	 * Sets AccumulatedYaw and deltaYaw to zero and updates currYaw and lastYaw
	 */
	void ResetAccumulatedYaw();
	virtual ~NavXPIDSource();
private:
	double currYaw_, lastYaw_, deltaYaw_, accumulatedYaw_;
	RobotModel *robot_;
};

/*---------------------- TALON ENCODER PID SOURCE ---------------------- */

class TalonEncoderPIDSource : public frc::PIDSource {
public:
	TalonEncoderPIDSource(RobotModel *robot);
	double PIDGet();
	virtual ~TalonEncoderPIDSource();
private:
	RobotModel *robot_;
	double averageTalonDistance_;
};

#endif /* SRC_PIDINPUTSOURCE_H_ */
