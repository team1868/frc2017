#include "AutoController.h"
#include "AutoCommand.h"
#include "Debugging.h"

AutoController::AutoController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure,
		/*CameraController* myCamera,*/	RemoteControl* myHumanControl) {
	robot = myRobot;
	firstCommand = NULL;
	nextCommand = NULL;
	currentCommand = NULL;
	autoMode = 0;
	autoStart = 0;
	drive = myDrive;
	superstructure = mySuperstructure;
//	camera = myCamera;
	humanControl = myHumanControl;
	timeFinished = 0.0;
	hardCodeShoot = true;
	hardCodeGear = true;
}

void AutoController::StartAutonomous() {
	humanControl->ReadControls(); //CHECK CHECK CHECK CHECK
	CreateQueue();
	currentCommand = firstCommand;
	if (currentCommand != NULL) {
		currentCommand->Init();
	}
}

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	if (currentCommand != NULL) {
		if (currentCommand->IsDone()) {
			DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
//			currentCommand = currentCommand->GetNextCommand();
			if (currentCommand != NULL) {
				currentCommand->Init();
			} else {
				timeFinished = currTimeSec;
			}
		} else {
			currentCommand->Update(currTimeSec, deltaTimeSec);
		}
	} else {
		DO_PERIODIC(100, printf("Queue finished at: %f \n", timeFinished));
	}
}

void AutoController::Reset() {
	firstCommand = NULL;
	currentCommand = NULL;
	humanControl->ReadControls();

//	robot->ShiftToLowGear();
//	robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
}

void AutoController::RefreshIni() {

}

void AutoController::CreateQueue() {
	firstCommand = NULL;
	printf("AutoMode: %i \n", autoMode);
//	if (humanControl->GetStopAutoDesired()) {
//		autoMode = kBlankAuto;
//	}

	switch (autoMode) {
	case (kTestAuto): {
		printf("kTestAuto ------------------\n");
//		DUMP("TEST AUTO", 0.0);
	}
	case (kBlankAuto): {
		printf("kBlankAuto ----------------------\n");
//		DUMP("BLANK AUTO", 0.0);
		break;
	}
	case (kReachAuto): {
		/*
		 * Assumption: starting position is back of robot at alliance wall
		 * Length of robot is FILL IN FILL IN ft
		 * Distance from wall to airship is FILL IN FILL IN
		 */
		printf("kReachAuto ------------------------\n");
//		DUMP("REACH AUTO", 0.0);
//		DriveStraightCommand* reachDrive = new DriveStraightCommand(robot, 4.0);

		break;
	}
	case (kOneGearAuto): {
		break;
	}
	case (kHighShootAuto): {
		break;
	}
	case (kHopperShootAuto): {
		break;
	}
	case (kOneGearShootAuto): {
		break;
	}
	case (kGearHopperShootAuto): {
		break;
	}
	case (kTwoGearAuto): {
		break;
	}
	case (kHopperAuto): {
		break;
	}
	}
}
