#include "AutoCommand.h"
#include "WaitingCommand.h"

#define PI 3.14159265358979

WaitingCommand::WaitingCommand(double myWaitTimeSec) {
	waitTimeSec = myWaitTimeSec;
	timer = new Timer();
	isDone = false;
}

void WaitingCommand::Init() {
	timer->Start();
}

void WaitingCommand::Update(double currTimeSec, double deltaTimeSec) {
	isDone = (timer->Get() >= waitTimeSec);
	if(isDone) {
		printf("done! :) %f", currTimeSec);
	}
}

bool WaitingCommand::IsDone() {
	return isDone;
}
