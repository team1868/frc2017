#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;

	// TODO CHANGE
	int autoMode = robot_->pini_->geti("AUTO MODE", "autoMode", 0);

	if (autoMode == 2) {
		liftPath_ = new PathCommand_OLD(robot_, PathCommand_OLD::kLeftLift);
	} else if (autoMode == 3) {
		liftPath_ = new PathCommand_OLD(robot_, PathCommand_OLD::kMiddleLift);
	} else if (autoMode == 4) {
		liftPath_ = new PathCommand_OLD(robot_, PathCommand_OLD::kRightLift);
	}

	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_);

	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
	firstCommand_ = liftPath_;
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
