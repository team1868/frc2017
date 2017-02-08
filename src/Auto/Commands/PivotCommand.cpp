/*
 * PivotCommand.cpp
 *
 *  Created on: Feb 2, 2017
 *      Author: Lynn D
 */

#include <Auto/Commands/PivotCommand.h>

PivotCommand::PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavxPIDSource* navxSource) {
//	pFac_ = 0.;
//	iFac_ = 0.0;
//	dFac_ = 0.0;

//	pFac_ = 0.75;
//	iFac_ = 0.001;
//	dFac_ = 3.7;
//	pFac_ = 0.2;
//	iFac_ = 0.0;
//	dFac_ = 0.0;

	pFac_ = 0.0075;
	iFac_ = 0.00008;
	dFac_ = 0.0021;

	printf("p: %f, i: %f, d: %f\n", pFac_, iFac_, dFac_);

	navxSource_ = navxSource;
	initYaw_ = navxSource_->CalculateAccumulatedYaw();

	if (isAbsolutePosition){
		desiredDeltaAngle_ = CalculateDeltaAngle(desiredAngle);
	} else {
		desiredDeltaAngle_ = desiredAngle;
	}
	isDone_ = false;

	robot_ = robot;
	talonOutput_ = new PivotPIDTalonOutput(robot_);

	pivotCommandStartTime_ = robot_->GetTime();
}

void PivotCommand::Init() {
	initYaw_ = navxSource_->PIDGet();
	pivotPID_ = new PIDController(pFac_, iFac_, dFac_, navxSource_, talonOutput_);
	pivotPID_->SetSetpoint(initYaw_ + desiredDeltaAngle_);
	pivotPID_->SetContinuous(false);
	pivotPID_->SetOutputRange(-0.8, 0.8);	// TODO probably lessen bc too OP
	//pivotPID_->SetTolerance(.9);	// TODO should check
	pivotPID_->SetAbsoluteTolerance(1.0);	// TODO should check
	pivotPID_->Enable();

	SmartDashboard::PutNumber("Initial yaw", initYaw_);
	printf("Desired Delta Angle: %f\n", pivotPID_->GetSetpoint());
}

void PivotCommand::Reset() {
	pivotPID_->Reset();
	pivotPID_->Disable();
	isDone_ = true;
	free(pivotPID_);

	printf("IS DONE FROM RESET\n");
}

void PivotCommand::Update(double currTimeSec, double deltaTimeSec) {
	SmartDashboard::PutNumber("Pivot Error", pivotPID_->GetError());
	SmartDashboard::PutNumber("Setpoint", pivotPID_->GetSetpoint());
	SmartDashboard::PutNumber("Delta setpoint", pivotPID_->GetDeltaSetpoint());
	SmartDashboard::PutBoolean("Is done", isDone_);

	bool timeOut = (robot_->GetTime() - pivotCommandStartTime_ > 5.0);

	SmartDashboard::PutBoolean("Timed out", timeOut);

	if (!isDone_) {
		if (pivotPID_->OnTarget() || timeOut) {
			pivotPID_->Reset();
			pivotPID_->Disable();
			isDone_ = true;

			printf("IS DONE \n");
			if (timeOut) {
				printf("FROM TIME OUT\n");
			}
		}
	}
}

bool PivotCommand::IsDone() {
	return isDone_;
}

double PivotCommand::CalculateDeltaAngle(double desiredAngle) {
	double currYaw = fmod(initYaw_, 360.0);
	return desiredAngle - currYaw;
}

PivotCommand::~PivotCommand() {
	pivotPID_->Reset();
	pivotPID_->Disable();
	isDone_ = true;
	free(pivotPID_);

	printf("IS DONE FROM DECONSTRUCTOR\n");
}

PivotPIDTalonOutput::PivotPIDTalonOutput(RobotModel *robot){
//	leftMaster_->SetInverted(false);
//	rightMaster_->SetInverted(false);
	robot_ = robot;
}

void PivotPIDTalonOutput::PIDWrite(double output){
	// one side is already mechanically inverted, so we don't need to negate the output of the wheels
	robot_->SetDriveValues(RobotModel::kLeftWheels, -output);
	robot_->SetDriveValues(RobotModel::kRightWheels, output);

	SmartDashboard::PutNumber("left output", -output);
	SmartDashboard::PutNumber("right output", output);
}

PivotPIDTalonOutput::~PivotPIDTalonOutput(){

}
