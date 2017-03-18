#include "Controllers/SuperstructureController.h"
#include "WPILib.h"
#include <cmath>

SuperstructureController::SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	currState_ = kInit;
	nextState_ = kInit;

	adjustedFlywheelVelocity_ = 0.0;
	desiredFlywheelVelocity_ = 11.5; // ft/sec, tune value
	// TODO put all this in the ini file
	// TODO add dial for changing velocity
	expectedFlywheelMotorOutput_ = 0.7; // Tune value
	feederMotorOutput_ = 0.85;
	climberMotorOutput_ = 0.9;
	intakeMotorOutput_ = 0.4; // postive for comp
	flywheelStartTime_ = 0.0;

	pFac_ = 0.0;
	iFac_ = 0.0;
	dFac_ = 0.0;
	fFac_ = expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_;
	// m_result = m_D * m_error + m_P * m_totalError + CalculateFeedForward();		// In line 123 of PIDController.cpp
	RefreshIni();
	flywheelController_ = new PIDController(pFac_, iFac_, dFac_, fFac_, robot_->GetFlywheelEncoder(), robot_->GetFlywheelMotor(), 0.02);

	flywheelController_->SetSetpoint(desiredFlywheelVelocity_);
	flywheelController_->SetOutputRange(0.0, 1.0);
	flywheelController_->SetAbsoluteTolerance(2.0);
	flywheelController_->SetContinuous(false);

	gearMechPos_ = false;

	flywheelStarted_ = false;
	flywheelStartTime_ = 0.0;

	autoFlywheelDesired_ = false;
	autoTimeIntakeDesired_ = false;
	autoStartedIntake_ = false;
	autoFinishedIntake_ = false;

	autoIntakeTime_ = 0.0;
	autoIntakeStartTime_ = 0.0;

	currState_ = kInit;
	nextState_ = kInit;
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

	flywheelController_->Reset();
	robot_->SetIntakeOutput(0.0);
	robot_->SetFeederOutput(0.0);
	robot_->SetClimberOutput(0.0);

	feederMotorOutput_ = -fabs(feederMotorOutput_);
	intakeMotorOutput_ = -fabs(intakeMotorOutput_); // positive for comp

	flywheelStarted_ = false;
	flywheelStartTime_ = 0.0;

	autoFlywheelDesired_ = false;
	autoTimeIntakeDesired_ = false;
	autoStartedIntake_ = false;
	autoFinishedIntake_ = false;

	autoIntakeTime_ = 0.0;
	autoIntakeStartTime_ = 0.0;

	currState_ = kInit;
	nextState_ = kInit;
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	robot_->GearUpdate();
	SetOutputs();
	if (humanControl_->GetGearMechOutDesired()) {
		gearMechPos_ = !gearMechPos_;
		robot_->SetGearMech(gearMechPos_);
	}

	switch(currState_) {
	case kInit:
			SmartDashboard::PutString("State", "kInit");
			flywheelController_->Disable();
			robot_->SetIntakeOutput(0.0);
			robot_->SetFeederOutput(0.0);
			robot_->SetClimberOutput(0.0);
			nextState_ = kIdle;
			break;
	case kIdle:
			SmartDashboard::PutString("State", "kIdle");
			nextState_ = kIdle;
			if (humanControl_->GetIntakeDesired()) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kIntake;
			} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
				flywheelController_->Enable();
				nextState_ = kFeederAndFlywheel;
			} else if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			} else if (autoTimeIntakeDesired_) {
				nextState_ = kTimeIntake;
			}
			break;
	case kIntake:
			SmartDashboard::PutString("State", "kIntake");
			SmartDashboard::PutNumber("Intake Output:", intakeMotorOutput_);
			if (humanControl_->GetIntakeDesired()) {
				printf("in intake desired: %f\n", intakeMotorOutput_);
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kIntake;
			} else {
				robot_->SetIntakeOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case kFeederAndFlywheel:
			SmartDashboard::PutString("State", "kFeederAndFlywheel");
			SmartDashboard::PutNumber("Flywheel Output", robot_->GetFlywheelMotorOutput());
			SmartDashboard::PutNumber("Flywheel Error", flywheelController_->GetError());
			SmartDashboard::PutNumber("Flywheel Velocity", robot_->GetFlywheelEncoder()->GetRate());
			SmartDashboard::PutNumber("Flywheel Distance", robot_->GetFlywheelEncoder()->GetDistance());
			SmartDashboard::PutNumber("Flywheel Pulses", robot_->GetFlywheelEncoder()->GetRaw());

			RefreshIni();
			flywheelController_->SetPID(pFac_, iFac_, dFac_, fFac_);
			if (!flywheelStarted_) {
				flywheelStartTime_ = robot_->GetTime();
				flywheelStarted_ = true;
				nextState_ = kFeederAndFlywheel;
			} else if (robot_->GetTime() - flywheelStartTime_ < 3.0) {
				nextState_ = kFeederAndFlywheel;
				printf("time - flywheelStartTime: %f\n", robot_->GetTime() - flywheelStartTime_);
			} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
				printf("IN FEEDER\n");
				robot_->SetFeederOutput(feederMotorOutput_);
//				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else {
				robot_->SetFeederOutput(0.0);
				robot_->SetIntakeOutput(0.0);
				flywheelController_->Disable();
				flywheelStarted_ = false;
				nextState_ = kIdle;
			}
			break;
	case kClimber:
			SmartDashboard::PutString("State", "kClimber");
			if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			} else {
				robot_->SetClimberOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case kTimeIntake:
			SmartDashboard::PutString("State", "kTimeIntake");
			if (!autoStartedIntake_) {
				autoIntakeStartTime_ = robot_->GetTime();
				robot_->SetIntakeOutput(intakeMotorOutput_);
				autoStartedIntake_ = true;
				nextState_ = kTimeIntake;
			} else if (robot_->GetTime() - autoIntakeStartTime_ < autoIntakeTime_) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kTimeIntake;
			} else {
				robot_->SetIntakeOutput(0.0);
				autoStartedIntake_ = false;
				autoFinishedIntake_ = true;
				nextState_ = kIdle;
			}
	}
	currState_ = nextState_;
}

bool SuperstructureController::GetAutoFlywheelDesired() {
	return autoFlywheelDesired_;
}

bool SuperstructureController::GetAutoIntakeDesired() {
	return autoTimeIntakeDesired_;
}

bool SuperstructureController::GetAutoFinishedIntake() {
	return autoFinishedIntake_;
}

void SuperstructureController::SetAutoFlywheelDesired(bool desired) {
	autoFlywheelDesired_ = desired;
}

void SuperstructureController::SetAutoTimeIntakeDesired(bool desired) {
	autoTimeIntakeDesired_ = desired;
}

void SuperstructureController::SetAutoIntakeTime(int seconds) {
	autoIntakeTime_ = seconds;
}

void SuperstructureController::SetAutoFinishedIntake(bool finished) {
	autoFinishedIntake_ = finished;
}

void SuperstructureController::SetOutputs() {
	if (humanControl_->GetReverseIntakeDesired()) {
		intakeMotorOutput_ = -intakeMotorOutput_;
	}
	if (humanControl_->GetReverseFeederDesired()) {
		feederMotorOutput_ = -feederMotorOutput_;
	}
	SmartDashboard::PutNumber("Feeder Velocity", feederMotorOutput_);
	SmartDashboard::PutNumber("Intake Velocity", intakeMotorOutput_);
	SmartDashboard::PutNumber("Flywheel Dial Value", humanControl_->GetFlywheelVelAdjust());
	adjustedFlywheelVelocity_ = desiredFlywheelVelocity_ + (desiredFlywheelVelocity_ * 0.2 * humanControl_->GetFlywheelVelAdjust());
	SmartDashboard::PutNumber("Adjusted Velocity", adjustedFlywheelVelocity_);
	flywheelController_->SetSetpoint(adjustedFlywheelVelocity_);
	fFac_ = expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_;
	flywheelController_->SetPID(pFac_, iFac_, dFac_, fFac_);
}

void SuperstructureController::RefreshIni() {
	pFac_ = robot_->pini_->getf("VELOCITY PID", "pFac", 0.0);
	iFac_ = robot_->pini_->getf("VELOCITY PID", "iFac", 0.0);
	dFac_ = robot_->pini_->getf("VELOCITY PID", "dFac", 0.1);
	fFac_ = expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_;
	desiredFlywheelVelocity_ = robot_->pini_->getf("VELOCITY PID", "flywheelVelocity", 11.5);
}

SuperstructureController::~SuperstructureController() {

}
