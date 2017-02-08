#include <Auto/AutoController.h>

AutoController::AutoController() {
	autoMode = nullptr;
}

AutoController::AutoController(AutoMode *myAutoMode){
	autoMode = myAutoMode;
}

void AutoController::SetAutonomousMode(AutoMode *myAutoMode) {
	autoMode = myAutoMode;
}

void AutoController::Init() {
	autoMode->CreateQueue();
	autoMode->Init();
}

void AutoController::Update(double currTimeSec, double deltaTimeSec) {
	autoMode->Update(currTimeSec, deltaTimeSec);
}

void AutoController::Reset() {

}

bool AutoController::IsDone() {
	return autoMode->IsDone();
}

//#include "AutoController.h"
//#include "Auto/Commands/AutoCommand.h"
//#include "Debugging.h"
//
//AutoController::AutoController(RobotModel* myRobot, DriveController* myDrive, SuperstructureController* mySuperstructure,
//		/*CameraController* myCamera,*/	RemoteControl* myHumanControl) {
//	robot_ = myRobot;
//	firstCommand_ = NULL;
//	nextCommand_ = NULL;
//	currentCommand_ = NULL;
//	autoMode_ = 0;
//	autoStart_ = 0;
//	drive_ = myDrive;
//	superstructure_ = mySuperstructure;
////	camera = myCamera;
//	humanControl_ = myHumanControl;
//	timeFinished_ = 0.0;
//	hardCodeShoot_ = true;
//	hardCodeGear_ = true;
//}
//
//void AutoController::StartAutonomous() {
//	humanControl_->ReadControls(); //CHECK CHECK CHECK CHECK
//	CreateQueue();
//	currentCommand_ = firstCommand_;
//	if (currentCommand_ != NULL) {
//		currentCommand_->Init();
//	}
//}
//
//void AutoController::Update(double currTimeSec, double deltaTimeSec) {
//	if (currentCommand_ != NULL) {
//		if (currentCommand_->IsDone()) {
//			DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
////			currentCommand = currentCommand->GetNextCommand();
//			if (currentCommand_ != NULL) {
//				currentCommand_->Init();
//			} else {
//				timeFinished_ = currTimeSec;
//			}
//		} else {
//			currentCommand_->Update(currTimeSec, deltaTimeSec);
//		}
//	} else {
//		DO_PERIODIC(100, printf("Queue finished at: %f \n", timeFinished_));
//	}
//}
//
//void AutoController::Reset() {
//	firstCommand_ = NULL;
//	currentCommand_ = NULL;
//	humanControl_->ReadControls();
//
////	robot->ShiftToLowGear();
////	robot->SetWheelSpeed(RobotModel::kAllWheels, 0.0);
//}
//
//void AutoController::RefreshIni() {
//
//}
//
//void AutoController::CreateQueue() {
//	firstCommand_ = NULL;
//	printf("AutoMode: %i \n", autoMode_);
////	if (humanControl->GetStopAutoDesired()) {
////		autoMode = kBlankAuto;
////	}
//
//	switch (autoMode_) {
//	case (kTestAuto): {
//		printf("kTestAuto ------------------\n");
////		DUMP("TEST AUTO", 0.0);
//	}
//	case (kBlankAuto): {
//		printf("kBlankAuto ----------------------\n");
////		DUMP("BLANK AUTO", 0.0);
//		break;
//	}
//	case (kReachAuto): {
//		/*
//		 * Assumption: starting position is back of robot at alliance wall
//		 * Length of robot is FILL IN FILL IN ft
//		 * Distance from wall to airship is FILL IN FILL IN
//		 */
//		printf("kReachAuto ------------------------\n");
////		DUMP("REACH AUTO", 0.0);
////		DriveStraightCommand* reachDrive = new DriveStraightCommand(robot, 4.0);
//
//		break;
//	}
//	case (kOneGearAuto): {
//		break;
//	}
//	case (kHighShootAuto): {
//		break;
//	}
//	case (kHopperShootAuto): {
//		break;
//	}
//	case (kOneGearShootAuto): {
//		break;
//	}
//	case (kGearHopperShootAuto): {
//		break;
//	}
//	case (kTwoGearAuto): {
//		break;
//	}
//	case (kHopperAuto): {
//		break;
//	}
//	}
//}
