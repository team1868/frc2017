#include <Auto/Commands/DriveStraightCommand.h>
#include "WPILib.h"

DriveStraightCommand::DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
		AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
		double desiredDistance) {
	robot_ = robot;

	navXSource_ = navXSource;
	talonEncoderSource_ = talonEncoderSource;

	anglePIDOutput_ = anglePIDOutput;
	distancePIDOutput_ = distancePIDOutput;

	initialAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();

	// Convert feet to encoder values and adding the difference to the initial to know how far we want to go
	desiredDistance_ = desiredDistance;
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;
	isDone_ = false;
	maxDriveTime_ = robot_->GetTime();

	GetIniValues();

	anglePID_ = new PIDController(rPFac_, rIFac_, rDFac_, navXSource_, anglePIDOutput_);
	distancePID_ = new PIDController(dPFac_, dIFac_, dDFac_, talonEncoderSource_, distancePIDOutput_);
}

void DriveStraightCommand::Init() {
	robot_->SetPercentVBusDriveMode();

	isDone_= false;

	leftMotorOutput_ = 0.0;
	rightMotorOutput_ = 0.0;

	initialAngle_ = navXSource_->PIDGet();
	initialAvgDistance_ = talonEncoderSource_->PIDGet();
	desiredTotalAvgDistance_ = initialAvgDistance_ + desiredDistance_;

	anglePID_->SetSetpoint(initialAngle_);
	distancePID_->SetSetpoint(desiredTotalAvgDistance_);

	anglePID_->SetContinuous(false);
	distancePID_->SetContinuous(false);

	anglePID_->SetOutputRange(-0.2, 0.2);
	distancePID_->SetOutputRange(-0.8, 0.8);

	// TODO fix this later
	anglePID_->SetAbsoluteTolerance(2.0);			// 1 degree
	distancePID_->SetAbsoluteTolerance(2.5/12.0);	// 1 inch

	anglePID_->Enable();
	distancePID_->Enable();

	maxDriveTime_ = robot_->GetTime();
}

void DriveStraightCommand::Update(double currTimeSec, double deltaTimeSec) {
	printf("Drive straight command update\n");
	SmartDashboard::PutNumber("Left Motor Output", leftMotorOutput_);
	SmartDashboard::PutNumber("Right Motor Output", rightMotorOutput_);

	SmartDashboard::PutBoolean("Is Distance Met", distancePID_->OnTarget());
	SmartDashboard::PutBoolean("Is Angle Met", anglePID_->OnTarget());
	SmartDashboard::PutBoolean("Is Done", (anglePID_->OnTarget()) && (distancePID_->OnTarget()));

	maxDriveTime_ = robot_->GetTime() - maxDriveTime_;
	if ((anglePID_->OnTarget() && (distancePID_->OnTarget())) || (maxDriveTime_ > 4.0)) {
		anglePID_->Reset();
		distancePID_->Reset();

		// To ensure that the wheels are off
		leftMotorOutput_ = 0.0;
		rightMotorOutput_ = 0.0;

		isDone_= true;
	} else {
		// Getting the output values from the PID Controllers and send them to motor outputs
		double dOutput = distancePIDOutput_->GetPIDOutput();
		double rOutput = anglePIDOutput_->GetPIDOutput();

		// TODO might want to check
		rightMotorOutput_ = dOutput + rOutput;
		leftMotorOutput_ = dOutput - rOutput;

//		rightMotorOutput_ = dOutput;
//		leftMotorOutput_ = dOutput;

		// TODO put this maxOutput back in
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
	rPFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rPFac", 0.0);
	rIFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rIFac", 0.0);
	rDFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "rDFac", 0.0);
//	rPFac_ = 0.0;
//	rIFac_ = 0.0;
//	rDFac_ = 0.0;

	dPFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dPFac", 0.2);
	dIFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dIFac", 0.0);
	dDFac_ = robot_->pini_->getf("DRIVESTRAIGHT PID", "dDFac", 0.0);
//	dPFac_ = 0.3;
//	dIFac_ = 0.0;
//	dDFac_ = 0.0;
}

DriveStraightCommand::~DriveStraightCommand() {

}

