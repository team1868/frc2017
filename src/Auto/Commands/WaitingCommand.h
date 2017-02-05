#include "WPILib.h"
#include "Debugging.h"
#include "RobotModel.h"
#include "RemoteControl.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "ControlBoard.h"
#include <vector>
#include <string>
#include <iostream>
#include "DriveController.h"
#include "AutoCommand.h"
//#include "CameraController.h"

class WaitingCommand: public AutoCommand {
public:
	WaitingCommand(double myWaitTimeSec);
	~WaitingCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	double waitTimeSec;
	Timer *timer;
	bool isDone;
};
