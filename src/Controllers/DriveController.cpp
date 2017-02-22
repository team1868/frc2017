#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl, NavXPIDSource *navX) {
	robot_ = robot;
	humanControl_ = humanControl;
	navX_ = navX;
	anglePIDOutput_ = new AnglePIDOutput();

	pFac_ = 0.01;
	iFac_ = 0.0;
	dFac_ = 0.0;

	driveStraight_ = new PIDController(pFac_, iFac_, dFac_, navX_, anglePIDOutput_);
	isDone_ = false;
	isDriveStraightStarted_ = false;
	desiredAngle_ = robot_->GetNavXYaw();
	angleOutput_ = 0.0;

	driveStraight_->SetOutputRange(-1.0, 1.0);
	driveStraight_->SetContinuous(false);
	driveStraight_->SetAbsoluteTolerance(2);

	currState_ = kInitialize;
	nextState_ = kInitialize;
}

void DriveController::Reset() {

}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	SmartDashboard::PutNumber("Left encoder", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	SmartDashboard::PutNumber("Right encoder", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));

	PrintDriveValues();

	switch (currState_) {
		case (kInitialize):
			nextState_ = kTeleopDrive;
			break;
		case (kTeleopDrive) :
			robot_->SetPercentVBusDrive();

			// Getting joystick values
			double leftJoyY, rightJoyY, rightJoyX;
			leftJoyY = -humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);	// was neg
			rightJoyY = -humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);	// was neg
			rightJoyX = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);

			if (humanControl_->GetGearShiftDesired()) {
				robot_->SetHighGear();
			} else {
				robot_->SetLowGear();
			}

			if (humanControl_->GetQuickTurnDesired()) {
				printf("Quick turning\n");
				QuickTurn(rightJoyX);
			} else if (humanControl_->GetArcadeDriveDesired()) {
				printf("Arcade driving\n");
				ArcadeDrive(rightJoyX, leftJoyY);
			} else {
				printf("Tank driving\n");
				TankDrive(leftJoyY, rightJoyY);
			}

			nextState_ = kTeleopDrive;
			break;
	}

	currState_ = nextState_;
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Drive direction", DriveDirection());
	SmartDashboard::PutNumber("Get state", GetDriveState());
}

void DriveController::ArcadeDrive(double myX, double myY) {
	PrintDriveValues();

	double thrustValue = myY * DriveDirection();
	double rotateValue = myX;
	double leftOutput = 0.0;
	double rightOutput = 0.0;

	// If thrust is less than 0.1, do not rotate
	if (fabs(thrustValue) < 0.1) {
		rotateValue = 0.0;
	}

	leftOutput = thrustValue;
	rightOutput = thrustValue;

	if (fabs(rotateValue) > 0.1) {	 // If we want turn
		driveStraight_->Disable();
		isDriveStraightStarted_ = false;

		leftOutput += rotateValue;
		rightOutput -= rotateValue;

	} else { // If we want straight
/*		if (!isDriveStraightStarted_) {
			desiredAngle_ = robot_->GetNavXYaw();
			driveStraight_->SetSetpoint(desiredAngle_);
			driveStraight_->Enable();

			angleOutput_ = anglePIDOutput_->GetPIDOutput();
		} else {
			printf("Driving Straight \n");
			angleOutput_ = anglePIDOutput_->GetPIDOutput();
		}

		leftOutput += angleOutput_;
		rightOutput -= angleOutput_;
		SmartDashboard::PutNumber("Angle Error", driveStraight_->GetError());*/
	}

	// Finding the max output
	double maxOutput = fmax(fabs(leftOutput), fabs(rightOutput));

	// If the maxOutput exceeds 1.0, scale them down by maxOutput
	if (maxOutput > 1.0) {
		leftOutput /= maxOutput;
		rightOutput /= maxOutput;
	}

	// 	TODO ask about sensitivity of the joysticks

	if (fabs(thrustValue) < 0.2) {			// TODO change if necessary
		leftOutput = 0.0;
		rightOutput = 0.0;
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);

	SmartDashboard::PutNumber("Left motor output", leftOutput );
	SmartDashboard::PutNumber("Right motor output", rightOutput);
}

void DriveController::TankDrive(double left, double right) {
	PrintDriveValues();
	double leftOutput = left * DriveDirection();
	double rightOutput = right * DriveDirection();

	// 	TODO ask about sensitivity of the joysticks

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);
}

void DriveController::QuickTurn(double myRight) {
	robot_->SetDriveValues(RobotModel::kLeftWheels, myRight);
	robot_->SetDriveValues(RobotModel::kRightWheels, -myRight);
}

int DriveController::DriveDirection() {
	if (humanControl_->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

int DriveController::GetDriveState() {
	return currState_;
}

bool DriveController::IsDone() {
	return isDone_;
}

DriveController::~DriveController() {

}
