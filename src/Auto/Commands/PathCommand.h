#ifndef SRC_AUTO_COMMANDS_PATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_PATHCOMMAND_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "Path.h"

class PathCommand {
public:
	PathCommand(RobotModel *myRobot, Path *myPath);
	virtual ~PathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */
