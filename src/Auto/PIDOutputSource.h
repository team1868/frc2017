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
	/**
	 * Initializes pidOutput_ to 0
	 */
	AnglePIDOutput();

	/**
	 * Sets pidoutput_ to the output from the PID loop
	 */
	void PIDWrite(double output);

	/**
	 * @return pidOutput_;
	 */
	double GetPIDOutput();

	/**
	 * Destructor
	 */
	virtual ~AnglePIDOutput();
private:
	/**
	 * The output from PID loop
	 */
	double pidOutput_;
};

class DistancePIDOutput : public frc::PIDOutput {
public:
	/**
	 * Initializes pidOutput to 0
	 */
	DistancePIDOutput();

	/**
	 * Gets the ouptut from the PID loop and assigns it to pidOuput_
	 */
	void PIDWrite(double output);

	/**
	 * @return pidOutput_
	 */
	double GetPIDOutput();

	/**
	 * Destructor
	 */
	virtual ~DistancePIDOutput();
private:
	/**
	 * The output from PID loop
	 */
	double pidOutput_;
};
#endif /* SRC_AUTO_PIDOUTPUTSOURCE_H_ */
