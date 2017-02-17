#include <Controllers/SuperstructureController.h>
#include "WPILib.h"

SuperstructureController::SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	m_stateVal_ = kInit;
	nextState_ = kInit;

	desiredIntakeVelocity_ = 100.0;
	desiredFlywheelVelocity_ = 250.0;

	expectedIntakeMotorOutput_ = 0.7;
	expectedFlywheelMotorOutput_ = 0.7;

	feederMotorOutput_ = 0.7;
	climberMotorOutput_ = 0.7;

	intakeController_ = new PIDController(0.0, 0.0, 0.0, (expectedIntakeMotorOutput_ / desiredIntakeVelocity_), robot_->GetIntakeEncoder(), robot_->GetIntakeMotor(), 0.02);
	intakeController_->SetSetpoint(desiredIntakeVelocity_);
	intakeController_->SetOutputRange(-1.0, 1.0);
	intakeController_->SetAbsoluteTolerance(5.0);
	intakeController_->SetContinuous(false);

	flywheelController_ = new PIDController(0.0, 0.0, 0.0, (expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_), robot_->GetFlywheelEncoder(), robot_->GetFlywheelMotor(), 0.02);
	flywheelController_->SetSetpoint(desiredFlywheelVelocity_);
	flywheelController_->SetOutputRange(-1.0, 1.0);
	flywheelController_->SetAbsoluteTolerance(5.0);
	flywheelController_->SetContinuous(false);


	autoFlywheelDesired_ = false;
}

void SuperstructureController::Reset() {
	m_stateVal_ = kInit;
	nextState_ = kInit;

	intakeController_->Reset();
	flywheelController_->Reset();
	robot_->SetFeederOutput(0.0);
	robot_->SetClimberOutput(0.0);

	autoFlywheelDesired_ = false;
}
void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	robot_->SetGearInRobot();
	switch(m_stateVal_) {
	case (kInit):
			intakeController_->Disable();
			flywheelController_->Disable();
			robot_->SetFeederOutput(0.0);
			robot_->SetClimberOutput(0.0);
			nextState_ = kIdle;
			break;
	case (kIdle):
			nextState_ = kIdle;
			if (humanControl_->GetIntakeDesired()) {
				intakeController_->Enable();
				nextState_ = kIntake;
			} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
				flywheelController_->Enable();
				robot_->SetFeederOutput(feederMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			}
			break;
	case (kIntake):
			if (humanControl_->GetIntakeDesired() || autoFlywheelDesired_) {
				nextState_ = kIntake;
			} else {
				intakeController_->Disable();
				nextState_ = kIdle;
			}
			break;
	case (kFeederAndFlywheel):
			if (humanControl_->GetFlywheelDesired()) {
				robot_->SetFeederOutput(0.7);
				nextState_ = kFeederAndFlywheel;
			} else {
				flywheelController_->Disable();
				robot_->SetFeederOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kClimber):
			if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(0.7);
				nextState_ = kClimber;
			} else {
				robot_->SetClimberOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	}
	m_stateVal_ = nextState_;
}

bool SuperstructureController::GetAutoFlywheelDesired() {
	return autoFlywheelDesired_;
}

void SuperstructureController::SetAutoFlywheelDesired(bool desired) {
	autoFlywheelDesired_ = desired;
}

SuperstructureController::~SuperstructureController() {

}

