#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoderSource) {
	robot_ = robot;
	humanControl_ = humanControl;
	navXSource_ = navX;
	talonEncoderSource_ = talonEncoderSource;
	anglePIDOutput_ = new AnglePIDOutput();
	alignWithPegStarted_ = false;

	// TODO THIS SHOULD READ FROM INI FILE (IN A SEPARATE HEADER)
	pFac_ = 0.01;
	iFac_ = 0.0;
	dFac_ = 0.0;

	driveStraightPIDController_ = new PIDController(pFac_, iFac_, dFac_, navXSource_, anglePIDOutput_);
	isDone_ = false;
	isDriveStraightStarted_ = false;
	desiredAngle_ = robot_->GetNavXYaw();
	angleOutput_ = 0.0;

	driveStraightPIDController_->SetOutputRange(-1.0, 1.0);
	driveStraightPIDController_->SetContinuous(false);
	driveStraightPIDController_->SetAbsoluteTolerance(2.0);

	pegCommand_ = NULL;

	currState_ = kInitialize;
	nextState_ = kInitialize;
}

void DriveController::Reset() {
	robot_->SetPercentVBusDriveMode();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (currState_) {
		case (kInitialize):
			if (humanControl_->GetAlignWithPegDesired()) {
				nextState_ = kAlignWithPeg;
			} else {
				nextState_ = kTeleopDrive;
			}

			break;

		case (kTeleopDrive) :
			robot_->SetPercentVBusDriveMode();

			// Getting joystick values
			double leftJoyY, rightJoyY, rightJoyX;
			leftJoyY = -humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);	// was neg
			rightJoyY = -humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);	// was neg
			rightJoyX = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);

			if (humanControl_->GetHighGearDesired()) {
				printf("Set high gear\n");
				robot_->SetHighGear();
			} else {
				printf("Set low gear\n");
				robot_->SetLowGear();
			}

			nextState_ = kTeleopDrive;

			if (humanControl_->GetAlignWithPegDesired()) {
				printf("Going to alignwithpeg\n");
				nextState_ = kAlignWithPeg;
			} else if (humanControl_->GetQuickTurnDesired()) {
				printf("Quick turning\n");
				QuickTurn(rightJoyX);
			} else if (humanControl_->GetArcadeDriveDesired()) {
				printf("Arcade driving\n");
				ArcadeDrive(rightJoyX, leftJoyY);
			} else if (!humanControl_->GetArcadeDriveDesired()){
				printf("Tank driving\n");
				TankDrive(leftJoyY, rightJoyY);
			} else {
				printf("SOMETHING IS WRONG\n");
				ArcadeDrive(rightJoyX, leftJoyY);		// Default to arcade drive
			}
			break;

		case (kAlignWithPeg) :
			if (!alignWithPegStarted_){
				pegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonEncoderSource_);
				pegCommand_->Init();
				alignWithPegStarted_ = true;
				nextState_ = kAlignWithPeg;
			}

			if (!pegCommand_->IsDone()){
				pegCommand_->Update(currTimeSec, deltaTimeSec);
				nextState_ = kAlignWithPeg;
			} else {
				alignWithPegStarted_ = false;
				nextState_ = kTeleopDrive;
			}

			printf("IN ALIGN WITH PEG COMMAND\n");
			break;
	}

	currState_ = nextState_;
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Drive direction", DriveDirection());
	SmartDashboard::PutNumber("Get state", GetDriveState());
	SmartDashboard::PutNumber("NavX angle", robot_->GetNavXYaw());
	SmartDashboard::PutNumber("Left drive distance", robot_->GetLeftDistance());
	SmartDashboard::PutNumber("Right drive distance", robot_->GetRightDistance());
//	SmartDashboard::PutNumber("Left drive encoder value", robot_->leftMaster_->GetEncPosition());
//	SmartDashboard::PutNumber("Right drive encoder value", robot_->rightMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Left drive encoder value", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
	SmartDashboard::PutNumber("Right drive encoder value", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));
}

void DriveController::ArcadeDrive(double myX, double myY) {
	SmartDashboard::PutString("Drive Mode", "Arcade Drive");
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
		driveStraightPIDController_->Disable();
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

	if (fabs(thrustValue) < 0.15) {			// TODO change if necessary
		leftOutput = 0.0;
		rightOutput = 0.0;
	}

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);

	SmartDashboard::PutNumber("Left motor output", leftOutput );
	SmartDashboard::PutNumber("Right motor output", rightOutput);
}

void DriveController::TankDrive(double left, double right) {
	SmartDashboard::PutString("Drive Mode", "Tank Drive");
	double leftOutput = left * DriveDirection();
	double rightOutput = right * DriveDirection();

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightOutput);
}

void DriveController::QuickTurn(double myRight) {
	SmartDashboard::PutString("Drive Mode", "Quick Turn");
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
