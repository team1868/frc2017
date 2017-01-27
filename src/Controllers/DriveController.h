#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "WPILib.h"
#include "RobotModel.h"
#include "Controllers/MotionProfileExample.h"
#include "ControlBoard.h"

class DriveController {
public:
	DriveController(RobotModel* robot, ControlBoard *humanControl);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	void ArcadeDrive(double x, double y);				// TODO, currently empty
	void TankDrive(double left, double right);
	void SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength);	// gets called in PathCommand::Update(double, double)
	bool IsDone();											// gets called in PathCommand::IsDone()
	~DriveController();

private:
	// TODO add switch statement for drive states
//	RobotModel *robot;
	ControlBoard *controlBoard;
	CANTalon *leftMaster_, *leftSlave_, *rightMaster_, *rightSlave_;		// TODO move all talon and encoder stuff into RobotModel (or not)
//	Encoder *leftEncoder, *rightEncoder;
	MotionProfileExample *leftExample_, *rightExample_;
	ControlBoard* humanControl_;
	bool isDone;

	void PrintDriveValues();
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
