#include "Controllers/DriveController.h"
#include "WPILib.h"

DriveController::DriveController(RobotModel* myRobot) {
	robot = myRobot;
	leftMaster = new CANTalon(0);	// TODO get deviceNumber
	leftSlave = new CANTalon(0);
	rightMaster = new CANTalon(0);
	rightSlave = new CANTalon(0);

	leftEncoder = new Encoder(0, 0, true);		// TODO
	rightEncoder = new Encoder(0, 0, true);

	leftMaster->SetFeedbackDevice(CANTalon::AnalogEncoder);
	rightMaster->SetFeedbackDevice(CANTalon::AnalogEncoder);

//	// set PID constants for all
//	leftMaster->SetF(0.0);
//	leftMaster->SetP(0.1);
//	leftMaster->SetI(0.0);
//	leftMaster->SetD(0.0);
}

void DriveController::Init() {

}

void DriveController::Update(double currTimeSec, double deltaTimeSec) {

}

void DriveController::FollowTrajectory(TrajectoryCandidate trajectoryCandidate) {
	int length = trajectoryCandidate.length;
	Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));

	pathfinder_generate(&trajectoryCandidate, trajectory);

	Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
	Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);

	double wheelbase_width = 0.6;			// CHECK THIS
	pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);
	free(trajectory);

	leftMaster->SetControlMode(CANTalon::kMotionProfile);

	// to finish
}

bool DriveController::IsDoneFollowingTrajectory() {
	return true;
}

DriveController::~DriveController() {

}
