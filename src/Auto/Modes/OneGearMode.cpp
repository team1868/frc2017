#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot) {
	robot_ = robot;
	firstCommand_ = NULL;
	liftPath_ = new PathCommand(robot_, PathCommand::kLiftTwo);	// to put this as input to OneGearMode?

	NavxPIDSource *navxSource = new NavxPIDSource(robot_);
	pivotCommand_ = new PivotCommand(robot_, 90.0, false, navxSource);

	navxSource->ResetAccumulatedYaw();
//	liftPath2_ = new PathCommand(robot_, PathCommand::kLiftOne); 	// TO REPLACE THIS WITH PIVOT COMMAND
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
	firstCommand_ = liftPath_;
	liftPath_->SetNextCommand(pivotCommand_);

	currentCommand = firstCommand_;
}

void OneGearMode::Init() {
//	liftPath_->Init();
//	pivotCommand_->Init();
//	liftPath2_->Init();
//	currentCommand = firstCommand_;
}

//bool OneGearMode::IsDone() {
//	return liftPath_->IsDone();
//}

OneGearMode::~OneGearMode() {

}
