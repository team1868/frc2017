#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* myRobot) {
//	robot = myRobot;
	leftMaster_ = new CANTalon(LEFT_DRIVE_MASTER_ID);
	leftSlave_ = new CANTalon(LEFT_DRIVE_SLAVE_ID);
	rightMaster_ = new CANTalon(RIGHT_DRIVE_MASTER_ID);
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
}

void DriveController::PrintDriveValues() {
	SmartDashboard::PutNumber("Left encoder", leftMaster_->GetEncPosition());
	SmartDashboard::PutNumber("Right encoder", rightMaster_->GetEncPosition());
}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength) {
	// Setting up left motors for motion profiling
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(LEFT_DRIVE_MASTER_ID); 	// port number of the leftMaster

	// Setting up right motors for motion profiling
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID); 	// port number of the rightMaster

	// Setting up trajectories for update
	leftExample_->start(leftTrajectory, trajectoryLength);
	rightExample_->start(rightTrajectory, trajectoryLength);
}

void DriveController::TankDrive(double myLeft, double myRight) {

}

bool DriveController::IsDone() {
	return isDone;
}

DriveController::~DriveController() {

}
