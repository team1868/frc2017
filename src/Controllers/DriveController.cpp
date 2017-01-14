#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* myRobot) {
//	robot = myRobot;
	leftMaster = new CANTalon(0);	// TODO get deviceNumber
	leftSlave = new CANTalon(1);
	rightMaster = new CANTalon(2);
	rightSlave = new CANTalon(3);

//	leftEncoder = new Encoder(0, 0, true);		// TODO
//	rightEncoder = new Encoder(0, 0, true);

	leftMaster->SetFeedbackDevice(CANTalon::AnalogEncoder);
	rightMaster->SetFeedbackDevice(CANTalon::AnalogEncoder);

	leftMaster->SetSensorDirection(true);	// TODO check
	leftSlave->SetSensorDirection(true);
	rightMaster->SetSensorDirection(true);
	rightSlave->SetSensorDirection(true);


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

	leftExample = new MotionProfileExample(*leftMaster);
	rightExample = new MotionProfileExample(*rightMaster);
}

void DriveController::Init() {
	leftMaster->ClearMotionProfileTrajectories();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {
	leftExample->control();
	rightExample->control();
	CANTalon::SetValueMotionProfile setLeftOutput = leftExample->getSetValue();
	CANTalon::SetValueMotionProfile setRightOutput = rightExample->getSetValue();
	leftMaster->Set(setLeftOutput);
	rightMaster->Set(setRightOutput);
}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int myLength) {
	// Setting up left motors for motion profiling
	leftMaster->SetControlMode(CANTalon::kMotionProfile);
	leftSlave->SetControlMode(CANTalon::kFollower);
	leftSlave->Set(0); // should be the port number of the leftMaster

	// Setting up right motors for motion profiling
	rightMaster->SetControlMode(CANTalon::kMotionProfile);
	rightSlave->SetControlMode(CANTalon::kFollower);
	rightSlave->Set(2); // should be the port number of the rightMaster

	// Setting up trajectories for update
	leftExample->start(leftTrajectory, myLength);
	rightExample->start(rightTrajectory, myLength);

}

bool DriveController::IsDone() {
	return true;		// TODO change
}

DriveController::~DriveController() {

}
