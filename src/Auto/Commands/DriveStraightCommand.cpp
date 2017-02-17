/*
 * DriveStraightCommand.cpp
 *
 *  Created on: Feb 9, 2017
 *      Author: Lynn D
 */

#include <Auto/Commands/DriveStraightCommand.h>
#include "WPILib.h"

const double WHEEL_DIAMETER = 6.05 / 12.0; // in feet
const double ENCODER_COUNT_PER_ROTATION = 256.0;

AnglePIDOutput::AnglePIDOutput(){
	pidOutput_ = 0.0;
}

void AnglePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double AnglePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

AnglePIDOutput::~AnglePIDOutput() {

}

DistancePIDOutput::DistancePIDOutput(){
	pidOutput_ = 0.0;
}

void DistancePIDOutput::PIDWrite(double output) {
	pidOutput_ = output;
}

double DistancePIDOutput::GetPIDOutput() {
	return pidOutput_;
}

DistancePIDOutput::~DistancePIDOutput() {

}

DriveStraightCommand::DriveStraightCommand(NavxPIDSource* navxSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance) {
	navxSource_ = navxSource;
	talonEncoderSource_ = talonEncoderSource;
	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;
	robot_ = robot;

//	//TODO tune these values pls
//	// PID Gains for rotation
//	rPFac_ = 0.0001;
//	rIFac_ = 0.0;
//	rDFac_ = 0.0;
//
//	// PID Gains for distance
//	dPFac_ = 0.0005;
//	dIFac_ = 0.0;
//	dDFac_ = 0.0;

	initialAngle_ = navxSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();

	// Convert feet to encoder values and adding the difference to the initial to know how far we want to go
	//desiredDistance_ = ; //  * (ENCODER_COUNT_PER_ROTATION * 4)/ (WHEEL_DIAMETER * M_PI);
	desiredDistance_ = desiredDistance;
	desiredTotalAvgDistance_ = desiredDistance_ + initialAvgDistance_;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;

	GetIniValues();

	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navxSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);
}

void DriveStraightCommand::Init() {
	isDone_= false;
	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	robot_->SetPercentVDrive();

	initialAngle_ = navxSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetSetpoint(initialAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(false);
	distancePID_->SetContinuous(false);

	anglePID_->SetOutputRange(-0.2, 0.2);
	distancePID_->SetOutputRange(-0.8, 0.8);

	// TODO fix this later
	anglePID_->SetAbsoluteTolerance(1.0);
	distancePID_->SetAbsoluteTolerance(0.07);

	anglePID_->Enable();
	distancePID_->Enable();
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("Drive straight command update\n");
	SmartDashboard::PutNumber("Left Motor Output", leftMotorOutput_);
	SmartDashboard::PutNumber("Right Motor Output", rightMotorOutput_);

	SmartDashboard::PutBoolean("Is Distance Met", distancePID_->OnTarget());
	SmartDashboard::PutBoolean("Is Angle Met", anglePID_->OnTarget());
	SmartDashboard::PutBoolean("Is Done", (anglePID_->OnTarget()) && (distancePID_->OnTarget()));

	if ((anglePID_->OnTarget()) && (distancePID_->OnTarget())) {
		anglePID_->Reset();
		distancePID_->Reset();
		isDone_= true;

		// To ensure that the wheels are off
		leftMotorOutput_ = 0.0;
		rightMotorOutput_ = 0.0;
	} else {
		// Getting the output values from the PID Controllers and send them to motor outputs
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		// Might want to check
		rightMotorOutput_ = dOutput - rOutput;
		leftMotorOutput_ = dOutput + rOutput;

//		rightMotorOutput_ = dOutput;
//		leftMotorOutput_ = dOutput;

		// Getting the max value of the outputs and scale the down
		double maxOutput = fmax(fabs(rightMotorOutput_), fabs(leftMotorOutput_));
		SmartDashboard::PutNumber("Max output", maxOutput);
//		if (maxOutput > 1.0) {
//			rightMotorOutput_ = ( rightMotorOutput_ / maxOutput );
//			leftMotorOutput_ = ( leftMotorOutput_ / maxOutput );
//		}

		SmartDashboard::PutNumber("Angle Error", anglePID_->GetError());
		SmartDashboard::PutNumber("Desired Angle", initialAngle_);
		SmartDashboard::PutNumber("Encoder Error Feet", distancePID_->GetError());
		SmartDashboard::PutNumber("Desired Total Feet", desiredTotalAvgDistance_);
		SmartDashboard::PutNumber("Desired Difference Feet", desiredDistance_);
	}
	robot_->SetDriveValues(RobotModel::kLeftWheels, leftMotorOutput_);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightMotorOutput_);
}

bool DriveStraightCommand::IsDone() {
	return isDone_;
}

void DriveStraightCommand::GetIniValues() {
	//std::cout << robot_->pini->Ini("robot.ini");
	rPFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
//	rPFac_ = 0.001;
	rIFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	rDFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);

	dPFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "dPFac", 0.0);
	//dPFac_ = 0.1;
	dIFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	dDFac_ = robot_->pini->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
}

DriveStraightCommand::~DriveStraightCommand() {
	// TODO Auto-generated destructor stub
}

