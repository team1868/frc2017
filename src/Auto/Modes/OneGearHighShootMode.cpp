#include <Auto/Modes/OneGearHighShootMode.h>

OneGearHighShootMode::OneGearHighShootMode(RobotModel *robot, SuperstructureController *superstructure, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, bool isLeft) {
	robot_ = robot;
	superstructure_ = superstructure;

	navXSource_ = navXSource;
	talonSource_ = talonSource;

	firstCommand_ = NULL;

	if (isLeft) {
		liftPath_ = new PathCommand(robot_, PathCommand::kLeftLift);
		highGoalPath_ = new PathCommand(robot_, PathCommand::kHighGoalAfterLeftLift);
	} else {
		liftPath_ = new PathCommand(robot_, PathCommand::kRightLift);
		highGoalPath_ = new PathCommand(robot_, PathCommand::kHighGoalAfterRightLift);
	}

	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_, true);
	gearCommand_ = new GearCommand(robot_);
	waitingCommand_ = new WaitingCommand(2.0);

	// IF SWITCH SIDES, KHIGHGOALAFTERRIGHTLIFT
	highGoalShootCommand_ = new HighGoalShootCommand(superstructure_);
	alignWithHighGoalCommand_ = new AlignWithHighGoalCommand(robot_, navXSource_, talonSource_);

	// TODO ADD PARALLEL AUTO COMMAND

	highGoalPathAndShootCommand_ = new ParallelAutoCommand(highGoalPath_, highGoalShootCommand_);

	printf("in one gear high shoot mode constructor\n");
}

void OneGearHighShootMode::CreateQueue() {
	printf("Creating queue\n");

	firstCommand_ = liftPath_;
	liftPath_->SetNextCommand(waitingCommand_);
	waitingCommand_->SetNextCommand(highGoalPathAndShootCommand_);
	//waitingCommand_->SetNextCommand(highGoalPath_);
	//highGoalPath_->SetNextCommand(highGoalShootCommand_);

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
