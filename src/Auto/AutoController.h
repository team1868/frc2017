#ifndef SRC_AUTO_AUTOCONTROLLER_H_
#define SRC_AUTO_AUTOCONTROLLER_H_

#include "Auto/Modes/AutoMode.h"

class AutoController {
public:
	AutoController();
	AutoController(AutoMode *autoMode);
	virtual ~AutoController() {}
	void SetAutonomousMode(AutoMode *autoMode);
	void StartAutonomous();		// TODO
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();	// TODO
	bool IsDone();

private:
	AutoMode *autoMode;
};

#endif /* SRC_AUTO_AUTOCONTROLLER_H_ */


//#ifndef AUTOCONTROLLER_H
//#define AUTOCONTROLLER_H
//
//#include "Auto/Modes/AutoMode.h"
//#include "Auto/Commands/AutoCommand.h"
////#include "Auto/Commands/DriveCommands.h"
////#include "Auto/Commands/SuperstructureCommands.h"
//#include "Controllers/DriveController.h"
//#include "Controllers/SuperstructureController.h"
//#include "DriverStation/RemoteControl.h"
//#include "Auto/Modes/OneGearMode.h"
////#include "CameraController.h"
//#include <vector>
//#include <string>
//#include <iostream>
//
//class AutoController {
//public:
//	enum AutoMode { kTestAuto = 0,
//					kBlankAuto = 1,
//					kReachAuto = 2,
//					kOneGearAuto = 3,
//					kHighShootAuto = 4,
//					kHopperShootAuto = 5,
//					kOneGearShootAuto = 6,
//					kGearHopperShootAuto = 7,
//					kTwoGearAuto = 8,
//					kHopperAuto = 9};
//	AutoController(RobotModel* myRobot, DriveController* myDrive,
//				   SuperstructureController* mySuperstructure,
//				   RemoteControl* myHumanControl); //add controllers as we create them as parameters of the constructor, add "CameraController* myCamera," later
//	~AutoController() {}
//
//	void StartAutonomous();
//	void Update(double currTimeSec, double deltaTimeSec);
//	void Reset();
//	void RefreshIni();
//
//private:
//	void CreateQueue();
////	void AddtoQueue(AutoCommand* myNewAutoCommand, SimpleAutoCommand* myLastAutoCommand);
//	AutoCommand *firstCommand_, *nextCommand_, *currentCommand_;
//	RobotModel *robot_;
//	DriveController *drive_;
//	SuperstructureController *superstructure_;
//	RemoteControl* humanControl_;
//	unsigned int autoMode_;
//	unsigned int autoStart_;
//	bool hardCodeShoot_;
//	bool hardCodeGear_;
//	double timeFinished_;
//};
//
//#endif /* AUTOCONTROLLER_H */
