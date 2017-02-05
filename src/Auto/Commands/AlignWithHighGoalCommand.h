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

class AlignWithHighGoalCommand: public AutoCommand {
public:
	AlignWithHighGoalCommand();
	~AlignWithHighGoalCommand() {}
	virtual void Init();
	virtual void Update(double currAngle, double deltaAngle);
	virtual bool IsDone();
private:
	double angleInDegrees;
	bool isDone;
};
