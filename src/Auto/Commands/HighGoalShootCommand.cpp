#include <Auto/Commands/HighGoalShootCommand.h>

HighGoalShootCommand::HighGoalShootCommand(SuperstructureController* mySuperstructure) {
	superstructure_ = mySuperstructure;
	isDone_ = false;
}

void HighGoalShootCommand::Init() {
	superstructure_->SetAutoFlywheelDesired(true);
}

void HighGoalShootCommand::Update(double currTimeSec, double deltaTimeSec) {
	superstructure_->Update(currTimeSec, deltaTimeSec);
}

bool HighGoalShootCommand::IsDone() {
	return isDone_;
}
