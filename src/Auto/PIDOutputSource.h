/*
 * PIDOutput.h
 *
 *  Created on: Feb 20, 2017
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_PIDOUTPUTSOURCE_H_
#define SRC_AUTO_PIDOUTPUTSOURCE_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "Auto/PIDInputSource.h"

class AnglePIDOutput : public frc::PIDOutput {
public:
	AnglePIDOutput();
	void PIDWrite(double output);
	double GetPIDOutput();
	virtual ~AnglePIDOutput();
private:
	double pidOutput_;
};

class DistancePIDOutput : public frc::PIDOutput {
public:
	DistancePIDOutput();
	void PIDWrite(double output);
	double GetPIDOutput();
	virtual ~DistancePIDOutput();
private:
	double pidOutput_;
};
#endif /* SRC_AUTO_PIDOUTPUTSOURCE_H_ */
