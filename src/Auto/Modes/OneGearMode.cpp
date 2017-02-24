#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;
	//liftPath_ = new PathCommand(robot_, PathCommand::kLiftTwo);	// to put this as input to OneGearMode?
	liftPath_ = new PathCommand(robot_, PathCommand::kLiftTwo);
	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_);

	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
	//firstCommand_ = alignWithPegCommand_;
//	alignWithPegCommand_->SetNextCommand(liftPath_);
	firstCommand_ = liftPath_;
	liftPath_->SetNextCommand(alignWithPegCommand_);
//	firstCommand_ = alignWithPegCommand_;
	currentCommand = firstCommand_;
}

void OneGearMode::Init() {
	firstCommand_->Init();
	currentCommand = firstCommand_;
}

void OneGearMode::RefreshIni() {
	printf("in one gear mode refresh ini\n");
}

bool OneGearMode::IsDone() {
	return false;		// TODO
}

OneGearMode::~OneGearMode() {

}
