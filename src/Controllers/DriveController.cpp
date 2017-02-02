#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl) {
//	robot_ = robot;
	humanControl_ = humanControl;

	// TODO move all Talons to RobotModel
	leftMaster_ = new CANTalon(LEFT_DRIVE_MASTER_ID);
	leftSlave_ = new CANTalon(LEFT_DRIVE_SLAVE_ID);
	rightMaster_ = new CANTalon(RIGHT_DRIVE_MASTER_ID);
	rightSlave_ = new CANTalon(RIGHT_DRIVE_SLAVE_ID);

	// Set encoders for Talons
	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMaster_->ConfigEncoderCodesPerRev(256);
	leftMaster_->SetPosition(0);
	leftMaster_->SetSensorDirection(true);				 // TODO check
	//rightMaster_->SetInverted(false);					 // TODO check
	//rightMaster_->SetClosedLoopOutputDirection(false); // TODO check

	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMaster_->ConfigEncoderCodesPerRev(256);
	rightMaster_->SetPosition(0);
	rightMaster_->SetSensorDirection(false); 			// TODO check
	rightMaster_->SetInverted(true);					// TODO check
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check

	// Set left and right slaves
	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

	// Set PID constants for left	// TODO
	leftMaster_->SetF(0.0);
	leftMaster_->SetP(0.1);
	leftMaster_->SetI(0.0);
	leftMaster_->SetD(0.0);

	// Set PID constants for right
	rightMaster_->SetF(0.0);
	rightMaster_->SetP(0.1);
	rightMaster_->SetI(0.0);
	rightMaster_->SetD(0.0);

	leftExample_ = new MotionProfileExample(*leftMaster_);
	rightExample_ = new MotionProfileExample(*rightMaster_);

	isDone_ = false;

	currState_ = kInitialize;
	nextState_ = kInitialize;
}

void DriveController::Reset() {
	leftMaster_->ClearMotionProfileTrajectories();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	PrintDriveValues();

	switch (currState_) {
		case (kInitialize):
			nextState_ = kTeleopDrive;
			break;
		case (kTeleopDrive) :
			leftMaster_->SetControlMode(CANTalon::kPercentVbus);
			rightMaster_->SetControlMode(CANTalon::kPercentVbus);

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

		case (kMotionProfile) :
			UpdateMotionProfile();
	}

	currState_ = nextState_;
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Left encoder", leftMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Right encoder", rightMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Drive direction", DriveDirection());
	SmartDashboard::PutNumber("Get state", GetDriveState());

    SmartDashboard::PutNumber("Left error", leftMaster_->GetClosedLoopError());
    SmartDashboard::PutNumber("Right error", leftMaster_->GetClosedLoopError());
}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength) {
	printf("Setup trajectory\n");
	// Setting up left motors for motion profiling
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);

	// Setting up right motors for motion profiling
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);

	// Setting up trajectories for update
	leftExample_->start(leftTrajectory, trajectoryLength);
	rightExample_->start(rightTrajectory, trajectoryLength);
}

void DriveController::UpdateMotionProfile() {
	// TODO doesn't work now
	printf("Motion profiling\n");
	leftExample_->control();
	rightExample_->control();
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
	CANTalon::SetValueMotionProfile setLeftOutput = leftExample_->getSetValue();
	CANTalon::SetValueMotionProfile setRightOutput = rightExample_->getSetValue();
	leftMaster_->Set(setLeftOutput);
	rightMaster_->Set(setRightOutput);

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

void DriveController::ArcadeDrive(double myX, double myY) {
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

	leftMaster_->Set(leftOutput);
	rightMaster_->Set(rightOutput);
}

void DriveController::TankDrive(double left, double right) {
	double leftOutput = left * DriveDirection();
	double rightOutput = right * DriveDirection();

	// 	TODO ask about sensitivity of the joysticks

	leftMaster_->Set(leftOutput);
	rightMaster_->Set(rightOutput);
}

void DriveController::QuickTurn(double myRight) {
	leftMaster_->Set(myRight);
	rightMaster_->Set(-myRight);
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
