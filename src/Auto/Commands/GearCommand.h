#ifndef SRC_AUTO_COMMANDS_GEARCOMMAND_H_
#define SRC_AUTO_COMMANDS_GEARCOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"

#include "RobotModel.h"

class GearCommand : public AutoCommand {
public:
	GearCommand(RobotModel *robot);
	~GearCommand() {};

	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	RobotModel *robot_;
	bool isDone_;
};

#endif /* SRC_AUTO_COMMANDS_GEARCOMMAND_H_ */
