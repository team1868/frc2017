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

//	pFac_ = 0.03;
////	iFac_ = 0.00008;
////	dFac_ = 0.0021;
//	iFac_ = 0.0;
//	dFac_ = 0.0;


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

	GetIniValues();
}

void PivotCommand::GetIniValues() {
	//std::cout << robot_->pini->Ini("robot.ini");
	pFac_ = robot_->pini->getf("PIVOT PID", "pFac", 0.0);
	iFac_ = robot_->pini->getf("PIVOT PID", "iFac", 0.0);
	dFac_ = robot_->pini->getf("PIVOT PID", "dFac", 0.0);
	minDrivePivotOutput_ = robot_->pini->getf("PIVOT PID", "minDrivePivotOutput", 0.0);
	printf("p: %f, i: %f, d: %f\n", pFac_, iFac_, dFac_);
	SmartDashboard::PutNumber("P_fac", pFac_);
	SmartDashboard::PutNumber("I_fac", iFac_);
	SmartDashboard::PutNumber("D_fac", dFac_);
}

void PivotCommand::Init() {
	robot_->RefreshIni();
	initYaw_ = navxSource_->PIDGet();
	pivotPID_ = new PIDController(pFac_, iFac_, dFac_, navxSource_, talonOutput_);
	pivotPID_->SetSetpoint(initYaw_ + desiredDeltaAngle_);
	pivotPID_->SetContinuous(false);
	pivotPID_->SetOutputRange(-0.8, 0.8);	// TODO probably lessen bc too OP
	//pivotPID_->SetTolerance(.9);	// TODO should check
	//pivotPID_->SetAbsoluteTolerance(1.0);	// TODO should check
	pivotPID_->SetAbsoluteTolerance(1.0);	// TODO should check
	pivotPID_->Enable();

	SmartDashboard::PutNumber("Initial yaw", initYaw_);
	global_pivotCommandIsDone = false;
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
		if ((pivotPID_->OnTarget() && (fabs(talonOutput_->GetOutput()) < minDrivePivotOutput_))) {
			//	|| timeOut) {
			pivotPID_->Reset();
			pivotPID_->Disable();
			isDone_ = true;
			global_pivotCommandIsDone = true;

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
	output_ = 0.0;
}

void PivotPIDTalonOutput::PIDWrite(double myOutput){
	output_ = myOutput;
	// one side is already mechanically inverted, so we don't need to negate the output of the wheels
	robot_->SetDriveValues(RobotModel::kLeftWheels, -output_);
	robot_->SetDriveValues(RobotModel::kRightWheels, output_);

//	printf("in PID write\n");
//	printf("left output: %f\n", -output);
//	printf("right output: %f\n", output);

	SmartDashboard::PutNumber("left output", -output_);
	SmartDashboard::PutNumber("right output", output_);
}

double PivotPIDTalonOutput::GetOutput() {
	return output_;
}

PivotPIDTalonOutput::~PivotPIDTalonOutput(){

}
