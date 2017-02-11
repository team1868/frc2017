/*
 * PivotCommand.h
 *
 *  Created on: Feb 2, 2017
 *      Author: Lynn D
 */

#ifndef SRC_PIVOTCOMMAND_H_
#define SRC_PIVOTCOMMAND_H_

#include "Auto/Commands/AutoCommand.h"
#include "Auto/PIDInputSource.h"
#include "RobotModel.h"

extern bool global_pivotCommandIsDone;

class PivotPIDTalonOutput : public frc::PIDOutput {
public:
	PivotPIDTalonOutput(RobotModel *robot);
	void PIDWrite(double output);
	virtual ~PivotPIDTalonOutput();
	double GetOutput();
private:
	RobotModel *robot_;
	double output_;
};
/**
 * A class implementing Pivot PID the WPILibrary PID Controller
 */
class PivotCommand : public AutoCommand, RobotModel {
public:
	/**
	 * PivotCommand constructor that gives the desired turn and whether or not it is absolute position
	 * @param robot a RobotModel
	 * @param desiredAngle a double that is the angle of the turn
	 * @param isAbsolutePosition a bool that represents whether the angle is absolute position or delta angle
	 * @param navxSource a NavxPIDSource
	 */
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavxPIDSource* navxSource);
	void Init();
	void Reset();
	void RefreshIni();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

	virtual ~PivotCommand();
private:
	/**
	 * Calculating the value of delta angle if the angle is absolute position.
	 * @param desiredAngle a double is the original angle passed in from constructor.
	 * @return deltaAngle which is the amount we have to turn to get to the absolute position
	 */
	double CalculateDeltaAngle(double desiredAngle); // for absolute position

	double pFac_, iFac_, dFac_;
	double desiredDeltaAngle_;
	double initYaw_;
	bool isDone_;

	RobotModel *robot_;
	PIDController *pivotPID_;
	NavxPIDSource *navxSource_;
	PivotPIDTalonOutput *talonOutput_;

	double pivotCommandStartTime_;
	double minDrivePivotOutput_;
};

#endif /* SRC_PIVOTCOMMAND_H_ */
