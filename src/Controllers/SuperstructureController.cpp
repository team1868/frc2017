#include <Controllers/SuperstructureController.h>
#include "WPILib.h"

SuperstructureController::SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	m_stateVal_ = kInit;
	nextState_ = kInit;

	desiredFlywheelVelocity_ = -125.0;

	expectedFlywheelMotorOutput_ = -0.96;

	feederMotorOutput_ = 0.7;
	climberMotorOutput_ = 0.7;
	intakeMotorOutput_ = 0.85;

	flywheelController_ = new PIDController(0.0, 0.0, 0.3, (expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_), robot_->GetFlywheelEncoder(), robot_->GetFlywheelMotor(), 0.02);
	flywheelController_->SetSetpoint(desiredFlywheelVelocity_);
	flywheelController_->SetOutputRange(-1.0, 1.0);
	flywheelController_->SetAbsoluteTolerance(2.0);
	flywheelController_->SetContinuous(false);


	autoFlywheelDesired_ = false;
}

void SuperstructureController::Reset() {
	m_stateVal_ = kInit;
	nextState_ = kInit;

	flywheelController_->Reset();
	robot_->SetIntakeOutput(0.0);
	robot_->SetFeederOutput(0.0);
	robot_->SetClimberOutput(0.0);

	autoFlywheelDesired_ = false;
}
void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
	robot_->GearUpdate();
	SetOutput();
	switch(m_stateVal_) {
	case (kInit):
			SmartDashboard::PutNumber("State", 1.0);
			flywheelController_->Disable();
			robot_->SetIntakeOutput(0.0);
			robot_->SetFeederOutput(0.0);
			robot_->SetClimberOutput(0.0);
			nextState_ = kIdle;
			break;
	case (kIdle):
			SmartDashboard::PutNumber("State", 2.0);
			nextState_ = kIdle;
			if (humanControl_->GetIntakeDesired()) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kIntake;
			} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
				flywheelController_->Enable();
				robot_->SetFeederOutput(feederMotorOutput_);
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			}
			break;
	case (kIntake):
			SmartDashboard::PutNumber("State", 3.0);
			if (humanControl_->GetIntakeDesired()) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kIntake;
			} else {
				robot_->SetIntakeOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kFeederAndFlywheel):
			SmartDashboard::PutNumber("State", 4.0);;
			if (humanControl_->GetFlywheelDesired() || GetAutoFlywheelDesired()) {
				SmartDashboard::PutNumber("Feeder", feederMotorOutput_);
				robot_->SetFeederOutput(feederMotorOutput_);
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else {
				flywheelController_->Disable();
				robot_->SetFeederOutput(0.0);
				robot_->SetIntakeOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kClimber):
			SmartDashboard::PutNumber("State", 5.0);
			if (humanControl_->GetClimberDesired()) {
				SmartDashboard::PutNumber("Climber", climberMotorOutput_);
				robot_->SetClimberOutput(climberMotorOutput_);
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

void SuperstructureController::SetOutput() {
	if (humanControl_->GetReverseIntakeDesired()) {
		intakeMotorOutput_ = -0.85;
	} else {
		intakeMotorOutput_ = 0.85;
	}
	if (humanControl_->GetReverseFeederDesired()) {
		feederMotorOutput_ = -0.7;
	} else {
		feederMotorOutput_ = 0.7;
	}
}

SuperstructureController::~SuperstructureController() {

}
