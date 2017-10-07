#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, int kAutoMode) {
	robot_ = robot;
	navXSource_ = navXSource;
	talonSource_ = talonSource;
	firstCommand_ = NULL;

	//autoMode_ = robot_->pini_->geti("AUTO MODE", "autoMode", 0);

	double desiredDistance = 0.0;
	double desiredAngle = 0.0;

	if (kAutoMode == 2) {
		SmartDashboard::PutString("Auto Mode", "LEFT LIFT AUTO");
		desiredDistance = robot_->pini_->getf("DESIRED DISTANCES", "leftLiftDesiredDistance", -7.5);
		desiredAngle = -60.0;
		printf("Side Gear Distance: %f\n", desiredDistance);
//		liftPath_ = new PathCommand(robot_, PathCommand::kLeftLift);
	} else if (kAutoMode == 3) {
		SmartDashboard::PutString("Auto Mode", "MIDDLE LIFT AUTO");
		desiredDistance = robot_->pini_->getf("DESIRED DISTANCES", "middleLiftDesiredDistance", -6.0);
		desiredAngle = 0.0;
//		liftPath_ = new PathCommand(robot_, PathCommand::kMiddleLift);
	} else if (kAutoMode == 4) {
		SmartDashboard::PutString("Auto Mode", "RIGHT LIFT AUTO");
		desiredDistance = robot_->pini_->getf("DESIRED DISTANCES", "rightLiftDesiredDistance", -7.5);
		desiredAngle = 60.0;
		printf("Side Gear Distance: %f\n", desiredDistance);
		//		liftPath_ = new PathCommand(robot_, PathCommand::kRightLift);
	} else {
		// SOMETHING IS WRONG print something
	}

	angleOutput_ = new AnglePIDOutput();
	distanceOutput_ = new DistancePIDOutput();

	driveStraightCommand_ = new DriveStraightCommand(navXSource_, talonSource_, angleOutput_, distanceOutput_, robot_, desiredDistance);
	pivotCommand_ = new PivotCommand(robot_, desiredAngle, false, navXSource_);

	alignWithPegCommand_ = new AlignWithPegCommand(robot_, navXSource_, talonSource_, true);

	printf("in one gear mode constructor\n");
}

void OneGearMode::CreateQueue() {
	printf("Creating queue\n");

	firstCommand_ = driveStraightCommand_;

	if (autoMode_ == 2 || autoMode_ == 4) {
		driveStraightCommand_->SetNextCommand(pivotCommand_);
		pivotCommand_->SetNextCommand(alignWithPegCommand_);
	}

//	driveStraightCommand_->SetNextCommand(pivotCommand_);
//	pivotCommand_->SetNextCommand(alignWithPegCommand_);
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
