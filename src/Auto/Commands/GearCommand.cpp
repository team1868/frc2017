#include <Auto/Commands/GearCommand.h>

GearCommand::GearCommand(RobotModel *robot) {
	robot_ = robot;
	isDone_ = false;
}

void GearCommand::Init() {
	robot_->SetGearInRobot(true);
	isDone_ = false;
}

void GearCommand::Update(double currTimeSec, double deltaTimeSec) {
	robot_->GearUpdate();
	if (robot_->GetGearInRobot()) {
		isDone_ = false;
	} else {
		isDone_ = true;
	}
}

bool GearCommand::IsDone() {
	return isDone_;
}

