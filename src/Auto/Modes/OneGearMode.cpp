#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource) {
	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;

	// TODO CHANGE
	autoMode_ = robot_->pini_->geti("AUTO MODE", "autoMode", 0);

	double desiredDistance = 0.0;
	double desiredAngle = 0.0;

	if (autoMode_ == 2) {
		desiredDistance = -6.0;		// CHANGE THESE NUMBERS (MAYBE PUT IN INI??)
		desiredAngle = -60.0;
//		liftPath_ = new PathCommand(robot_, PathCommand::kLeftLift);
	} else if (autoMode_ == 3) {
		desiredDistance = -6.0;
		desiredAngle = 0.0;
//		liftPath_ = new PathCommand(robot_, PathCommand::kMiddleLift);
	} else if (autoMode_ == 4) {
		desiredDistance = -6.0;
		desiredAngle = 60.0;
//		liftPath_ = new PathCommand(robot_, PathCommand::kRightLift);
	} else {
		// SOMETHING IS WRONG print something
	}

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_, robot_, desiredDistance);
	pivotCommand_ = new PivotCommand(robot_, desiredAngle, false, navXSource_);

	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_);

	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");
//	firstCommand_ = liftPath_;
	firstCommand_ = driveStraightCommand_;

	if (autoMode_ == 2 || autoMode_ == 4) {
		driveStraightCommand_->SetNextCommand(pivotCommand_);
		pivotCommand_->SetNextCommand(alignWithPegCommand_);
	}

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
