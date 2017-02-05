#ifndef AUTOCONTROLLER_H
#define AUTOCONTROLLER_H

#include "Auto/Modes/AutoMode.h"
#include "AutoCommand.h"
//#include "Auto/Commands/DriveCommands.h"
//#include "Auto/Commands/SuperstructureCommands.h"
#include "DriveController.h"
#include "SuperstructureController.h"
#include "RemoteControl.h"
//#include "CameraController.h"
#include <vector>
#include <string>
#include <iostream>

class AutoController {
public:
	enum AutoMode { kTestAuto = 0,
					kBlankAuto = 1,
					kReachAuto = 2,
					kOneGearAuto = 3,
					kHighShootAuto = 4,
					kHopperShootAuto = 5,
					kOneGearShootAuto = 6,
					kGearHopperShootAuto = 7,
					kTwoGearAuto = 8,
					kHopperAuto = 9};
	AutoController(RobotModel* myRobot, DriveController* myDrive,
				   SuperstructureController* mySuperstructure,
				   RemoteControl* myHumanControl); //add controllers as we create them as parameters of the constructor, add "CameraController* myCamera," later
	~AutoController() {}

	void StartAutonomous();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();
	void RefreshIni();

private:
	void CreateQueue();
//	void AddtoQueue(AutoCommand* myNewAutoCommand, SimpleAutoCommand* myLastAutoCommand);
	AutoCommand* firstCommand;
	AutoCommand* nextCommand;
	AutoCommand* currentCommand;
	RobotModel* robot;
	DriveController* drive;
	SuperstructureController* superstructure;
//	CameraController* camera;
	RemoteControl* humanControl;
	unsigned int autoMode;
	unsigned int autoStart;
	bool hardCodeShoot;
	bool hardCodeGear;
	double timeFinished;
};

#endif /* AUTOCONTROLLER_H */
