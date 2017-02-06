#ifndef SRC_AUTO_COMMANDS_PATHCOMMAND_H_
#define SRC_AUTO_COMMANDS_PATHCOMMAND_H_

#include <Auto/MotionProfiling/MotionProfileExecutor.h>
#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"
extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "Auto/MotionProfiling/MotionProfile.h"
#include "Auto/MotionProfiling/LiftOne_MotionProfile.h"
#include "Auto/MotionProfiling/LiftTwo_MotionProfile.h"
#include "Auto/MotionProfiling/LiftThree_MotionProfile.h"
#include "RobotModel.h"

class PathCommand : public AutoCommand {
public:
	enum Path { kLiftOne, kLiftTwo, kLiftThree };
	PathCommand(RobotModel *robot, Path path);
	~PathCommand();
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	RobotModel *robot_;
	Path path_;
	int lengthOfLeftMotionProfile_, lengthOfRightMotionProfile_;
	double leftMotionProfile_[][3];
	double rightMotionProfile_[][3];
	MotionProfileExecutor *leftMotionProfileExecutor_, *rightMotionProfileExecutor_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_PATHCOMMAND_H_ */
