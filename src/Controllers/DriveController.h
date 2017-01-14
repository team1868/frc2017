#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "Controllers/MotionProfileExample.h"

class DriveController {
public:
	DriveController(RobotModel* robot);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);	// TODO, currently empty
	void ArcadeDrive(double x, double y);				// TODO, currently empty
	void SetupTrajectory(Segment *leftTrajectory, Segment *rightTrajectory, int trajectoryLength);	// gets called in PathCommand::Update(double, double)
	bool IsDone();											// gets called in PathCommand::IsDone()
	~DriveController();

private:
	// TODO add switch statement for drive states
//	RobotModel *robot;
	CANTalon *leftMaster_, *leftSlave_, *rightMaster_, *rightSlave_;		// TODO move all talon and encoder stuff into RobotModel (or not)
//	Encoder *leftEncoder, *rightEncoder;
	MotionProfileExample *leftExample_, *rightExample_;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
