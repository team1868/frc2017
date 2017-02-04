#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "WPILib.h"
#include "RobotModel.h"
#include <Controllers/MotionProfileExample.h>
#include "DriverStation/ControlBoard.h"

class DriveController {
public:
	DriveController(RobotModel* robot, ControlBoard *humanControl);
	void Reset();
	void SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength);	// Gets called in PathCommand::Update(double, double)
	void UpdateMotionProfile();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();											// Gets called in PathCommand::IsDone()

	void PrintDriveValues();

	~DriveController();

	enum DriveState {
		kInitialize, kTeleopDrive, kMotionProfile
	};

private:
	void ArcadeDrive(double myX, double myY);
	void TankDrive(double myLeft, double myRight);
	void QuickTurn(double myRight);
	int DriveDirection();
	int GetDriveState();

//	RobotModel *robot;
	ControlBoard* humanControl_;
	CANTalon *leftMaster_, *leftSlave_, *rightMaster_, *rightSlave_; // TODO move all talon stuff into RobotModel (or not)
	MotionProfileExample *leftExample_, *rightExample_;
	bool isDone_;

	uint32_t currState_;
	uint32_t nextState_;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
