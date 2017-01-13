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
	PathCommand(DriveController *myDriveController, Waypoint *points);
	~PathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	TrajectoryCandidate trajectoryCandidate;
	DriveController *driveController;
	Waypoint *points;
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */
