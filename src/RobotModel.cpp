#include "RobotModel.h"
#include "WPILib.h"

RobotModel::RobotModel() {
	timer = new Timer();
	timer->Start();
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

double RobotModel::GetTime() {
	return timer->Get();
}

RobotModel::~RobotModel() {
}
