#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* myRobot) {
//	robot = myRobot;
	leftMaster_ = new CANTalon(5);
	leftSlave_ = new CANTalon(4);
	rightMaster_ = new CANTalon(2);
	rightSlave_ = new CANTalon(3);

//	leftEncoder = new Encoder(0, 0, true);		// TODO
//	rightEncoder = new Encoder(0, 0, true);

	leftMaster_->SetFeedbackDevice(CANTalon::AnalogEncoder);
	rightMaster_->SetFeedbackDevice(CANTalon::AnalogEncoder);

//	leftMaster_->SetSensorDirection(true);	// TODO check
//	leftSlave_->SetSensorDirection(true);
//	rightMaster_->SetSensorDirection(true);
//	rightSlave_->SetSensorDirection(true);


//	// set PID constants for left	// TODO
//	leftMaster->SetF(0.0);
//	leftMaster->SetP(0.1);
//	leftMaster->SetI(0.0);
//	leftMaster->SetD(0.0);

	// set PID constants for right
//	rightMaster->SetF(0.0);
//	rightMaster->SetP(0.1);
//	rightMaster->SetI(0.0);
//	rightMaster->SetD(0.0);

	leftExample_ = new MotionProfileExample(*leftMaster_);
	rightExample_ = new MotionProfileExample(*rightMaster_);

	isDone = false;
}

void DriveController::Init() {
	leftMaster_->ClearMotionProfileTrajectories();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	leftExample_->control();
	rightExample_->control();
	CANTalon::SetValueMotionProfile setLeftOutput = leftExample_->getSetValue();
	CANTalon::SetValueMotionProfile setRightOutput = rightExample_->getSetValue();
	leftMaster_->Set(setLeftOutput);
	rightMaster_->Set(setRightOutput);
}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength) {
	// Setting up left motors for motion profiling
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(5); // should be the port number of the leftMaster

	// Setting up right motors for motion profiling
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(2); // should be the port number of the rightMaster

	// Setting up trajectories for update
	leftExample_->start(leftTrajectory, trajectoryLength);
	rightExample_->start(rightTrajectory, trajectoryLength);
}

bool DriveController::IsDone() {
	return isDone;		// TODO set isDone somewhere
}

DriveController::~DriveController() {

}
