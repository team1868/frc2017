/*
 * IntakeCommand.h
 *
 *  Created on: Feb 19, 2017
 *      Author: alisha
 */

#ifndef SRC_AUTO_COMMANDS_INTAKECOMMAND_H_
#define SRC_AUTO_COMMANDS_INTAKECOMMAND_H_

#include "WPILib.h"
#include "Auto/Commands/AutoCommand.h"

#include "RobotModel.h"

class IntakeCommand : public AutoCommand {
public:
	IntakeCommand(SuperstructureController *mySuperstructure, double seconds);
	~IntakeCommand() {};

	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();

private:
	SuperstructureController *superstructure_;
	bool isDone_;
	double seconds_;
};

#endif /* SRC_AUTO_COMMANDS_INTAKECOMMAND_H_ */
