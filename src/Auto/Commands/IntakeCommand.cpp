/*
 * IntakeCommand.cpp
 *
 *  Created on: Feb 19, 2017
 *      Author: alisha
 */

#include <Auto/Commands/IntakeCommand.h>

IntakeCommand::IntakeCommand(SuperstructureController *mySuperstructure, double seconds) {
	superstructure_ = mySuperstructure;
	seconds_ = seconds;
	isDone_ = false;
}

void IntakeCommand::Init() {
//	superstructure_->SetAutoTimeIntakeDesired(true);
//	superstructure_->SetAutoIntakeTime(seconds_);
	isDone_ = false;
}

void IntakeCommand::Update(double currTimeSec, double deltaTimeSec) {
//	if (!superstructure_->GetAutoFinishedIntake()) {
//		superstructure_->Update(currTimeSec, deltaTimeSec);
//	} else {
//		isDone_ = true;
//		superstructure_->SetAutoTimeIntakeDesired(false);
//		superstructure_->SetAutoFinishedIntake(false);
//	}
}

bool IntakeCommand::IsDone() {
	return isDone_;
}
