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
	AutoCommand() {}
	virtual ~AutoCommand() {}
	virtual void Init() = 0;
	virtual void Update(double currTimeSec, double deltaTimeSec) = 0;
	virtual bool IsDone() = 0;

//	virtual AutoCommand* GetNextCommand() = 0;
};

#endif /* AUTOCOMMAND_H */
