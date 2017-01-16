/*
 * OfflinePathCommand.h
 *
 *  Created on: Jan 14, 2017
 *      Author: Lynn D
 */

#ifndef SRC_AUTO_COMMANDS_OFFLINEPATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_OFFLINEPATHCOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"
#include "Controllers/DriveController.h"
extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "Auto/Commands/LeftMotionProfile.h"
#include "Auto/Commands/RightMotionProfile.h"

class OfflinePathCommand : public AutoCommand {
public:
	OfflinePathCommand(DriveController *driveController, Waypoint *points, int pointLength);
	~OfflinePathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	void FillTrajectory(const double motionProfile[][3], Segment *trajectory, int length);
	Segment *leftTrajectory_, *rightTrajectory_;
	DriveController *driveController_;
	Waypoint *points_;
	int pointLength_;
};

#endif /* SRC_AUTO_COMMANDS_OFFLINEPATHCOMMAND_H_ */
