#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* robot, ControlBoard* humanControl) {
//	robot_ = robot;
	humanControl_ = humanControl;

	leftMaster_ = new CANTalon(LEFT_DRIVE_MASTER_ID);
	leftSlave_ = new CANTalon(LEFT_DRIVE_SLAVE_ID);
	rightMaster_ =  new CANTalon(RIGHT_DRIVE_MASTER_ID);
	rightSlave_ = new CANTalon(RIGHT_DRIVE_SLAVE_ID);

	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMaster_->ConfigEncoderCodesPerRev(360);
	leftMaster_->SetPosition(0);
	leftMaster_->SetSensorDirection(true);		// TODO check
	//rightMaster_->SetInverted(false);			// TODO check
	//rightMaster_->SetClosedLoopOutputDirection(false); // TODO check

	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMaster_->ConfigEncoderCodesPerRev(360);
	rightMaster_->SetPosition(0);
	rightMaster_->SetSensorDirection(false); 	// TODO check
	//rightMaster_->SetInverted(true);			// TODO check
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check

	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

	// set PID constants for left	// TODO
	leftMaster_->SetF(0.0);
	leftMaster_->SetP(0.1);
	leftMaster_->SetI(0.0);
	leftMaster_->SetD(0.0);

	//	 set PID constants for right
	rightMaster_->SetF(0.0);
	rightMaster_->SetP(0.1);
	rightMaster_->SetI(0.0);
	rightMaster_->SetD(0.0);

	leftExample_ = new MotionProfileExample(*leftMaster_);
	rightExample_ = new MotionProfileExample(*rightMaster_);

	isDone = false;
}

void DriveController::Init() {
	leftMaster_->ClearMotionProfileTrajectories();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	if (!(leftExample_->isDone() && rightExample_->isDone())) {
		leftExample_->control();
		rightExample_->control();
		CANTalon::SetValueMotionProfile setLeftOutput = leftExample_->getSetValue();
		CANTalon::SetValueMotionProfile setRightOutput = rightExample_->getSetValue();
		leftMaster_->Set(setLeftOutput);
		rightMaster_->Set(setRightOutput);

		PrintDriveValues();
	} else {
		isDone = true;
	}

	// Getting joystick values
	double leftJoyY = -humanControl_->GetJoystickValue(RemoteControl::kLeftJoy, RemoteControl::kY);
//	double rightJoyY = -humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kY);
	double rightJoyX = humanControl_->GetJoystickValue(RemoteControl::kRightJoy, RemoteControl::kX);
//	TankDrive(leftJoyY, rightJoyY);	// TODO check if we need this really
	ArcadeDrive(leftJoyY, rightJoyX);
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Left encoder", leftMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Right encoder", rightMaster_->GetEncPosition());
}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength) {
	// Setting up left motors for motion profiling
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);

	// Setting up right motors for motion profiling
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);

	// Setting up trajectories for update
	leftExample_->start(leftTrajectory, trajectoryLength);
	rightExample_->start(rightTrajectory, trajectoryLength);
}

void DriveController::ArcadeDrive(double x, double y) {
	double thrustValue = y;			// TODO multiply by drive direction
	double rotateValue = x;
	double leftOutput = 0.0;
	double rightOutput = 0.0;

	leftMaster_->SetControlMode(CANTalon::kPercentVbus);
	rightMaster_->SetControlMode(CANTalon::kPercentVbus);

	// if thrust is less than 0.1, do not rotate
	if (fabs(thrustValue) < 0.1) {
		rotateValue = 0.0;
	}

	leftOutput = thrustValue;
	rightOutput = thrustValue;

	if (fabs(rotateValue) < 0.1) {			// TODO add the PID loop to keep it straight
	} else {
		leftOutput += rotateValue;
		rightOutput -= rotateValue;
	}

	// Finding the max output
	double maxOutput = 0.0;
	if (fabs(leftOutput) > maxOutput) {
		maxOutput = fabs(leftOutput);
	}
	if (fabs(rightOutput) > maxOutput) {
		maxOutput = fabs(rightOutput);
	}

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
	double leftOutput = 0.0;
	double rightOutput = 0.0;

	// 	TODO ask about sensitivity of the joysticks

	leftMaster_->Set(leftOutput);
	rightMaster_->Set(rightOutput);
}
bool DriveController::IsDone() {
	return isDone;
}

DriveController::~DriveController() {

}
