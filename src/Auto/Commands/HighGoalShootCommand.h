/*
 * HighGoalShootCommand.h
 *
 *  Created on: Feb 4, 2017
 *      Author: bili
 */

#ifndef SRC_AUTO_COMMANDS_HIGHGOALSHOOTCOMMAND_H_
#define SRC_AUTO_COMMANDS_HIGHGOALSHOOTCOMMAND_H_

#include "WPILib.h"
#include "AutoCommand.h"
#include "Controllers/SuperstructureController.h"

class HighGoalShootCommand : public AutoCommand {
public:
	HighGoalShootCommand(SuperstructureController* mySuperstructure);
	~HighGoalShootCommand() {};
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	SuperstructureController* superstructure_;
	bool isDone_;

};

#endif /* SRC_AUTO_COMMANDS_HIGHGOALSHOOTCOMMAND_H_ */
