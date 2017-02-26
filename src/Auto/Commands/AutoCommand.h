#ifndef AUTOCOMMAND_H
#define AUTOCOMMAND_H

#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "DriverStation/RemoteControl.h"
#include "Controllers/DriveController.h"
#include "Controllers/SuperstructureController.h"
#include "DriverStation/ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>

using namespace std;

class AutoCommand {		// TODO parallel auto commands
public:
	/**
	 * AutoCommand, if extended, allows other commands to implement these methods
	 */
	AutoCommand() {
		nextCommand = NULL;
	}

	virtual ~AutoCommand() {}

	virtual void Init() = 0;

	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;

	virtual bool IsDone() = 0;

	AutoCommand* GetNextCommand() {
		return nextCommand;
	}

	void SetNextCommand(AutoCommand* myNextCommand) {
		nextCommand = myNextCommand;
	}

private:
	AutoCommand *nextCommand;
};

#endif /* AUTOCOMMAND_H */
