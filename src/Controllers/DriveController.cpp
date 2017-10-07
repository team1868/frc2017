#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoderSource) {
	robot_ = robot;
	humanControl_ = humanControl;
	isDone_ = false;

	thrustSensitivity_ = 0.0;
	rotateSensitivity_ = 0.0;
	quickTurnSensitivity_ = 0.0;

	// For AlignWithPeg
	alignWithPegStarted_ = false;
	pegCommand_ = NULL;
	navXSource_ = navX;
	talonEncoderSource_ = talonEncoderSource;

	currState_ = kTeleopDrive;
	nextState_ = kTeleopDrive;
}

void DriveController::Reset() {
	robot_->SetPercentVBusDriveMode();

	thrustSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "thrustSensitivity", 0.3);
	rotateSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "rotateSensitivity", 0.5);
	quickTurnSensitivity_ = robot_->pini_->getf("TELEOP DRIVING", "quickTurnSensitivity", 0.5);
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (currState_) {

	case (kTeleopDrive) :
		robot_->SetPercentVBusDriveMode();

		// Getting joystick values
		double leftJoyY, leftJoyZ, rightJoyY, rightJoyX, rightJoyZ;
		leftJoyY = -humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);	// was neg
		leftJoyZ = humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kZ);
		rightJoyY = -humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);	// was neg
		rightJoyX = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);
		rightJoyZ = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kZ);

		// so leftJoyZ and rightJoyZ are from -1 to 1
		thrustSensitivity_ = (leftJoyZ + 1.0) / 2.0;
		rotateSensitivity_ = (rightJoyZ + 1.0) / 2.0;
//		leftJoyZ = 0.3;		// TODO READ FROM INI!
//		rightJoyZ = 0.5;

		SmartDashboard::PutNumber("Thrust z", thrustSensitivity_);
		SmartDashboard::PutNumber("Rotate z", rotateSensitivity_);

		if (humanControl_->GetHighGearDesired()) {
			SmartDashboard::PutString("Gear", "High");
			robot_->SetHighGear();
		} else {
			SmartDashboard::PutString("Gear", "Low");
			robot_->SetLowGear();
		}

		if (humanControl_->GetAlignWithPegDesired()) {
			nextState_ = kAlignWithPeg;
		} else if (humanControl_->GetQuickTurnDesired()) {
			QuickTurn(rightJoyX, 0.0);
			nextState_ = kTeleopDrive;
		} else {
			ArcadeDrive(rightJoyX, leftJoyY, thrustSensitivity_, rotateSensitivity_);
			nextState_ = kTeleopDrive;
		}

		break;

	case (kAlignWithPeg) :
		if (!alignWithPegStarted_){
			printf("Initializing AlignWithPeg teleop");
			//Profiler startAlignPegProfiler(robot_, "kAlignWithPeg");
			pegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonEncoderSource_, false);
			pegCommand_->Init();
			alignWithPegStarted_ = true;
			nextState_ = kAlignWithPeg;
		}

		if (!pegCommand_->IsDone()){
			pegCommand_->Update(currTimeSec, deltaTimeSec);
			nextState_ = kAlignWithPeg;
		} else {
			printf("Done with AlignWithPeg teleop\n");
			alignWithPegStarted_ = false;
			nextState_ = kTeleopDrive;
		}

		printf("In AlignWithPeg\n");
		break;
	}

	currState_ = nextState_;
}

void DriveController::ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity) {
	SmartDashboard::PutString("Drive Mode", "Arcade Drive");

	double thrustValue = myY * GetDriveDirection();
	double rotateValue = myX;
//	double rotateValue = myX * GetDriveDirection(); // What reverse drive should actually be, test this when possible
	double leftOutput = 0.0;
	double rightOutput = 0.0;

	// Account for small joystick jostles (deadband)
	thrustValue = HandleDeadband(thrustValue, 0.1);	// 0.02 was too low
	rotateValue = HandleDeadband(rotateValue, 0.06);

	// Sensitivity adjustment
	rotateValue = GetCubicAdjustment(rotateValue, rotateSensitivity);
	rotateValue *= fabs(thrustValue);

	thrustValue = GetCubicAdjustment(thrustValue, thrustSensitivity);

	leftOutput = thrustValue + rotateValue;
	rightOutput = thrustValue - rotateValue;

    if (leftOutput > 1.0) {
        leftOutput = 1.0;
    } else if (rightOutput > 1.0) {
    	rightOutput = 1.0;
    } else if (leftOutput < -1.0) {
    	leftOutput = -1.0;
    } else if (rightOutput < -1.0) {
        rightOutput = -1.0;
    }

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);

	SmartDashboard::PutNumber("Left motor output", leftOutput );
	SmartDashboard::PutNumber("Right motor output", rightOutput);
}

void DriveController::TankDrive(double left, double right) {		// Currently not in use
	SmartDashboard::PutString("Drive Mode", "Tank Drive");
	double leftOutput = left * GetDriveDirection();
	double rightOutput = right * GetDriveDirection();

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);
}

void DriveController::QuickTurn(double myRight, double myRotateZ) {
	SmartDashboard::PutString("Drive Mode", "Quick Turn");

	double rotateValue = GetCubicAdjustment(myRight, myRotateZ);

	robot_->SetDriveValues(RobotModel::kLeftWheels, rotateValue);
	robot_->SetDriveValues(RobotModel::kRightWheels, -rotateValue);
}

int DriveController::GetDriveDirection() {
	if (humanControl_->GetReverseDriveDesired()) {
		return -1;
	} else {
		return 1;
	}
}

double DriveController::HandleDeadband(double value, double deadband) {
	if (fabs(value) < deadband) {
		return 0.0;
	} else {
		return value;
	}
}

// Rotation sensitivity adjustment: when z == 0, output = output; when z==1, output = output^3
double DriveController::GetCubicAdjustment(double value, double adjustmentConstant) {
	return adjustmentConstant * std::pow(value, 3.0) + (1 - adjustmentConstant) * value;
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Drive direction", GetDriveDirection());
	SmartDashboard::PutNumber("Get state", currState_);
	SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
	SmartDashboard::PutNumber("Left drive distance", robot_->GetLeftDistance());
	SmartDashboard::PutNumber("Right drive distance", robot_->GetRightDistance());
//	SmartDashboard::PutNumber("Left drive encoder value", robot_->leftMaster_->GetEncPosition());
//	SmartDashboard::PutNumber("Right drive encoder value", robot_->rightMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Left drive encoder value", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	SmartDashboard::PutNumber("Right drive encoder value", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
}

bool DriveController::IsDone() {
	return isDone_;
}

DriveController::~DriveController() { }
