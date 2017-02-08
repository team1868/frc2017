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

class PivotPIDTalonOutput : public frc::PIDOutput {
public:
	PivotPIDTalonOutput(RobotModel *robot);
	void PIDWrite(double output);
	virtual ~PivotPIDTalonOutput();
private:
	RobotModel *robot_;
};

class PivotCommand : public AutoCommand, RobotModel {
public:
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavxPIDSource* navxSource);
	void Init();
	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

	virtual ~PivotCommand();
private:
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
};

#endif /* SRC_PIVOTCOMMAND_H_ */
