#include <Auto/Modes/OneGearHighShootMode.h>

OneGearHighShootMode::OneGearHighShootMode(RobotModel *robot, SuperstructureController *superstructure, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	robot_ = robot;
	superstructure_ = superstructure;

	navXSource_ = navXSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;

	// IF SWITCH SIDES, LIFT ONE		// TODO ADD AS INPUT
	liftPath_ = new PathCommand(robot_, PathCommand::kLiftThree);
	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_);
	gearCommand_ = new GearCommand(robot_);
	// IF SWITCH SIDES, KHIGHGOALAFTERRIGHTLIFT
	highGoalPath_ = new PathCommand(robot_, PathCommand::kHighGoalAfterRightLift);
	highGoalShootCommand_ = new HighGoalShootCommand(superstructure_);
	alignWithHighGoalCommand_ = new AlignWithHighGoalCommand(robot_, navXSource, talonSource_);
	printf("in one gear high shoot mode constructor\n");
}

void OneGearHighShootMode::CreateQueue() {
	printf("Creating queue\n");
	firstCommand_ = liftPath_;
	liftPath_->SetNextCommand(highGoalPath_);
	highGoalPath_->SetNextCommand(alignWithHighGoalCommand_);
	alignWithHighGoalCommand_->SetNextCommand(highGoalShootCommand_);
//	liftPath_->SetNextCommand(alignWithPegCommand_);
//	alignWithPegCommand_->SetNextCommand(gearCommand_);
//	gearCommand_->SetNextCommand(highGoalPath_);
//	highGoalPath_->SetNextCommand(alignWithHighGoalCommand_);
//	alignWithHighGoalCommand_->SetNextCommand(highGoalShootCommand_);

	currentCommand = firstCommand_;
}

void OneGearHighShootMode::Init() {
	firstCommand_->Init();
	currentCommand = firstCommand_;
}

void OneGearHighShootMode::RefreshIni() {
}

bool OneGearHighShootMode::IsDone() {
	return false;		// TODO
}

OneGearHighShootMode::~OneGearHighShootMode() {

}
