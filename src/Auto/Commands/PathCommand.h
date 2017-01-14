#ifndef SRC_AUTO_COMMANDS_PATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_PATHCOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"
#include "Controllers/DriveController.h"
extern "C" {
#include <pathfinder/pathfinder.h>
}

class PathCommand : public AutoCommand {
public:
	PathCommand(DriveController *myDriveController, Waypoint *myPoints, int myPointLength);
	~PathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	Segment *leftTrajectory, *rightTrajectory;
	DriveController *driveController;
	Waypoint *points;
	int pointLength;
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */
