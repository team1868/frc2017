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

class AutoCommand {
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

class ParallelAutoCommand : public AutoCommand {
public:
	ParallelAutoCommand(AutoCommand* myFirst, AutoCommand* mySecond) {
		first = myFirst;
		second = mySecond;
		firstDone = false;
		secondDone = false;
		done = false;
	}

	virtual void Init() {
		first->Init();
		second->Init();
	}

	virtual void Update(double currTimeSec, double deltaTimeSec) {
		firstDone = first->IsDone();
		secondDone = second->IsDone();
		if (firstDone && secondDone) {
			done = true;
		} else if (firstDone && !secondDone) {
			second->Update(currTimeSec, deltaTimeSec);
		} else if (!firstDone && secondDone) {
			first->Update(currTimeSec, deltaTimeSec);
		} else {
			first->Update(currTimeSec, deltaTimeSec);
			second->Update(currTimeSec, deltaTimeSec);
		}
	}

	virtual bool IsDone() {
		return done;
	}

	virtual ~ParallelAutoCommand() {}

private:
	AutoCommand *first, *second;
	bool firstDone, secondDone, done;
};

#endif /* AUTOCOMMAND_H */
