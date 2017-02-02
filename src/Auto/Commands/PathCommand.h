#ifndef SRC_AUTO_COMMANDS_PATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_PATHCOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"
#include "Controllers/DriveController.h"
//extern "C" {
//#include "../ext/pathfinder/pathfinder.h"
//}
extern "C" {
#include <pathfinder/pathfinder.h>
}

class PathCommand : public AutoCommand {
public:
	PathCommand(DriveController *driveController, Waypoint *points, int pointLength);
	~PathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	Segment *leftTrajectory_, *rightTrajectory_;
	DriveController *driveController_;
	Waypoint *points_;
	int pointLength_;
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */
