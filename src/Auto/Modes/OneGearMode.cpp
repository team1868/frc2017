#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot, NavxPIDSource *navxSource, TalonEncoderPIDSource *talonSource) {
	robot_ = robot;
	navxSource_ = navxSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;
	liftPath_ = new PathCommand(robot_, PathCommand::kLiftTwo);	// to put this as input to OneGearMode?
	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navxSource_, talonSource_);
	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
//	firstCommand_ = alignWithPegCommand_;
//	alignWithPegCommand_->SetNextCommand(liftPath_);
	firstCommand_ = liftPath_;
	liftPath_->SetNextCommand(alignWithPegCommand_);
	currentCommand = firstCommand_;
}

void OneGearMode::Init() {
	firstCommand_->Init();
	currentCommand = firstCommand_;
}

void OneGearMode::RefreshIni() {
	//autoMode = robot->pini->geti("AUTONOMOUS","AutoMode",0);
	//hardCodeShoot = robot->pini->getbool("AUTONOMOUS", "HardCodeShoot", true);
	//secondDefensePos = robot->pini->geti("AUTONOMOUS", "SecondDefense", 0);
	//useSallyPort = robot->pini->getbool("AUTONOMOUS", "UseSallyPort", true);
	printf("in one gear mode refresh ini\n");
	//alignWithPegCommand_->RefreshIni();		// TODO move
}

bool OneGearMode::IsDone() {
	return false;		// TODO
}

OneGearMode::~OneGearMode() {

}
