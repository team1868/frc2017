#include <Auto/Modes/OneGearMode.h>

OneGearMode::OneGearMode(RobotModel *robot) {
	robot_ = robot;
	liftOnePath_ = new PathCommand(robot_, PathCommand::kLiftOne);
}

void OneGearMode::Init() {
	liftOnePath_->Init();
}

void OneGearMode::Update(double currTimeSec, double deltaTimeSec) {
	liftOnePath_->Update(currTimeSec, deltaTimeSec);
}

bool OneGearMode::IsDone() {
	return liftOnePath_->IsDone();
}

OneGearMode::~OneGearMode() {

}
