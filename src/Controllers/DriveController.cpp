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

//	// set PID constants for all
//	leftMaster->SetF(0.0);
//	leftMaster->SetP(0.1);
//	leftMaster->SetI(0.0);
//	leftMaster->SetD(0.0);
	example = new MotionProfileExample(*leftMaster);
}

void DriveController::Init() {
	leftMaster->ClearMotionProfileTrajectories();
}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {

}

void DriveController::SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory) {
	leftMaster->SetControlMode(CANTalon::kMotionProfile);
//	leftSlave->SetControlMode(CANTalon::kFollower);
//	leftSlave->Set(0); // should be the port number of the leftMaster
	example->control();
	CANTalon::SetValueMotionProfile setOutput = example->getSetValue();
	leftMaster->Set(setOutput);
	example->start();
}

bool DriveController::IsDone() {
	return true;		// TODO change
}

DriveController::~DriveController() {

}
