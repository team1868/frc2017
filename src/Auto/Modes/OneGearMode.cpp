#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot) {
	robot_ = robot;
	firstCommand_ = NULL;
//	liftPath_ = new PathCommand(robot_, PathCommand::kLiftTwo);	// to put this as input to OneGearMode?
//
//	NavxPIDSource *navxSource = new NavxPIDSource(robot_);
//	pivotCommand_ = new PivotCommand(robot_, 90.0, false, navxSource);
//
//	navxSource->ResetAccumulatedYaw();
//	liftPath2_ = new PathCommand(robot_, PathCommand::kLiftOne); 	// TO REPLACE THIS WITH PIVOT COMMAND
	alignWithPegCommand_ = new AlignWithPegCommand(robot_);
	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
//	firstCommand_ = liftPath_;
//	liftPath_->SetNextCommand(pivotCommand_);
	firstCommand_ = alignWithPegCommand_;

//	firstCommand_ = pivotCommand_;

	currentCommand = firstCommand_;
}

void OneGearMode::Init() {
//	liftPath_->Init();		// DON'T FORGET TO INIT THE FIRST COMMAND
	alignWithPegCommand_->Init();
//	pivotCommand_->Init();
//	pivotCommand_->Init();
//	liftPath2_->Init();
	currentCommand = firstCommand_;
}

void OneGearMode::RefreshIni() {
	//autoMode = robot->pini->geti("AUTONOMOUS","AutoMode",0);
	//hardCodeShoot = robot->pini->getbool("AUTONOMOUS", "HardCodeShoot", true);
	//secondDefensePos = robot->pini->geti("AUTONOMOUS", "SecondDefense", 0);
	//useSallyPort = robot->pini->getbool("AUTONOMOUS", "UseSallyPort", true);
	printf("in one gear mode refresh ini\n");
	alignWithPegCommand_->RefreshIni();		// TODO move
}

//bool OneGearMode::IsDone() {
//	return liftPath_->IsDone();
//}

OneGearMode::~OneGearMode() {

}
