#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl) {
	robot_ = robot;
	humanControl_ = humanControl;

	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.7, 0.02, 0.1, 1.38329);
	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.7, 0.02, 0.1, 1.30254);

//	leftExample_ = new MotionProfileExample(*leftMaster_);
//	rightExample_ = new MotionProfileExample(*rightMaster_);

	isDone_ = false;

	currState_ = kInitialize;
	nextState_ = kInitialize;
}

void DriveController::Reset() {
//	leftExample_->hasStarted = false;
//	rightExample_->hasStarted = false;

//	leftMaster_->ClearMotionProfileTrajectories();
//	leftMaster_->SetControlMode(CANTalon::kPercentVbus);
//	leftMaster_->Set( 0 );
	/* clear our buffer and put everything into a known state */
//	leftExample_->reset();

//	leftMaster_->SetPosition(0);
//
//	rightMaster_->SetControlMode(CANTalon::kPercentVbus);
//	rightMaster_->Set( 0 );
//	/* clear our buffer and put everything into a known state */
//	rightExample_->reset();
//
//	rightMaster_->SetPosition(0);
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (currState_) {
		case (kInitialize):
			nextState_ = kTeleopDrive;
			break;
		case (kTeleopDrive) :
			robot_->SetPercentVDrive();

			// Getting joystick values
			double leftJoyY, rightJoyY, rightJoyX;
			leftJoyY = -humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);	// was neg
			rightJoyY = -humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);	// was neg
			rightJoyX = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);

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

//		case (kMotionProfile) :
//			UpdateMotionProfile();
	}

	currState_ = nextState_;
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Left encoder", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	SmartDashboard::PutNumber("Right encoder", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
	SmartDashboard::PutNumber("Drive direction", DriveDirection());
	SmartDashboard::PutNumber("Get state", GetDriveState());

//    SmartDashboard::PutNumber("Left error", leftMaster_->GetClosedLoopError());
//    SmartDashboard::PutNumber("Right error", leftMaster_->GetClosedLoopError());
}
/*
void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength) {

	leftExample_->control();
	rightExample_->control();

//	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
//	rightMaster_->SetControlMode(CANTalon::kMotionProfile);

	CANTalon::SetValueMotionProfile leftSetOutput = leftExample_->getSetValue();
	CANTalon::SetValueMotionProfile rightSetOutput = rightExample_->getSetValue();

//	leftMaster_->Set(leftSetOutput);
//	rightMaster_->Set(rightSetOutput);

	if (!leftExample_->hasStarted && !rightExample_->hasStarted) {	// may not need
		leftExample_->start(true);
		rightExample_->start(false);
	}

//	printf("Setup trajectory\n");
//	// Setting up left motors for motion profiling
//	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
//
//	// Setting up right motors for motion profiling
//	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
//
//	// Setting up trajectories for update		// MAYBE HAVE TO MOVE TO UPDATE
//	if (!leftExample_->hasStarted && !rightExample_->hasStarted) {	// may not need
////		leftExample_->start(leftTrajectory, trajectoryLength);
////		rightExample_->start(rightTrajectory, trajectoryLength);
//		leftExample_->start(true);
//		rightExample_->start(false);
//	}
//
//	// TODO doesn't work now
//
//	printf("in DriveController UpdateMotionProfile\n");
//	leftExample_->control();
//	rightExample_->control();
//
//	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
//	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
//
//	CANTalon::SetValueMotionProfile leftSetOutput = leftExample_->getSetValue();
//	CANTalon::SetValueMotionProfile rightSetOutput = rightExample_->getSetValue();
//
//	leftMaster_->Set(leftSetOutput);
//	rightMaster_->Set(rightSetOutput);

	PrintDriveValues();
}

void DriveController::UpdateMotionProfile() {
//	// TODO doesn't work now
//
//	printf("in DriveController UpdateMotionProfile\n");
//	leftExample_->control();
//	rightExample_->control();
//
//	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
//	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
//
//	CANTalon::SetValueMotionProfile leftSetOutput = leftExample_->getSetValue();
//	CANTalon::SetValueMotionProfile rightSetOutput = rightExample_->getSetValue();
//
//	leftMaster_->Set(leftSetOutput);
//	rightMaster_->Set(rightSetOutput);

//	SmartDashboard::PutNumber("Left encoder", _leftMaster.GetEncPosition());
//	SmartDashboard::PutNumber("Right encoder", _rightMaster.GetEncPosition());
//	SmartDashboard::PutNumber("Left error", _leftMaster.GetClosedLoopError());
//	SmartDashboard::PutNumber("Right error", _rightMaster.GetClosedLoopError());
//	SmartDashboard::PutNumber("Left speed", _leftMaster.GetSpeed());
//	SmartDashboard::PutNumber("Right speed", _rightMaster.GetSpeed());

	if (!leftExample_->hasStarted && !rightExample_->hasStarted) {
//		leftExample_->start(leftTrajectory);		// takes in isLeft
//		rightExample_->start();
	}

//	printf("Motion profiling\n");
//	leftExample_->control();
//	rightExample_->control();
//	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
//	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
//	CANTalon::SetValueMotionProfile setLeftOutput = leftExample_->getSetValue();
//	CANTalon::SetValueMotionProfile setRightOutput = rightExample_->getSetValue();
//	leftMaster_->Set(setLeftOutput);
//	rightMaster_->Set(setRightOutput);

	PrintDriveValues();
	//if (!(leftExample_->isDone() && rightExample_->isDone())) {
//		printf("Setting motion profiling\n");
//		leftExample_->control();
//		rightExample_->control();
//		CANTalon::SetValueMotionProfile setLeftOutput = leftExample_->getSetValue();
//		CANTalon::SetValueMotionProfile setRightOutput = rightExample_->getSetValue();
//		leftMaster_->Set(setLeftOutput);
//		rightMaster_->Set(setRightOutput);
//	} else {
//		printf("Done with motion profiling\n");
//		isDone_ = true;
//	}
}
*/
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

	if (fabs(rotateValue) > 0.1) {	 // TODO possibly add PID loop to keep it straight
		leftOutput += rotateValue;
		rightOutput -= rotateValue;
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
