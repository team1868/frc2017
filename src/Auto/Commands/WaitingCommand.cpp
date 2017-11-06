#include "AutoCommand.h"
#include "WaitingCommand.h"

#define PI 3.14159265358979

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec_ = myWaitTimeSec;
	timer_ = new Timer();
	isDone_ = false;
}

void WaitingCommand::Init() {
	timer_->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone_ = (timer_->Get() >= waitTimeSec_);
	if(isDone_) {
		printf("done! :) %f", currTimeSec);
	}
}

bool WaitingCommand::IsDone() {
	return isDone_;
}
