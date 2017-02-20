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

	flywheelController_ = new PIDController(0.0, 0.0, 0.4, (expectedFlywheelMotorOutput_ / desiredFlywheelVelocity_), robot_->GetFlywheelEncoder(), robot_->GetFlywheelMotor(), 0.02);
	flywheelController_->SetSetpoint(desiredFlywheelVelocity_);
	flywheelController_->SetOutputRange(-1.0, 1.0);
	flywheelController_->SetAbsoluteTolerance(2.0);
	flywheelController_->SetContinuous(false);


	autoFlywheelDesired_ = false;
	autoTimeITDesired_ = false;
	autoStartedIntake_ = false;
	autoFinishedIntake_ = false;

	autoIntakeTime_ = 0.0;
	autoIntakeStartTime_ = 0.0;
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
			SmartDashboard::PutString("State", "kInit");
			flywheelController_->Disable();
			robot_->SetIntakeOutput(0.0);
			robot_->SetFeederOutput(0.0);
			robot_->SetClimberOutput(0.0);
			nextState_ = kIdle;
			break;
	case (kIdle):
			SmartDashboard::PutString("State", "kIdle");
			nextState_ = kIdle;
			if (humanControl_->GetGearMechOutDesired()) {
				robot_->SetGearMechOut();
			}
			if (humanControl_->GetIntakeDesired()) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				SmartDashboard::PutNumber("Intake Output:", intakeMotorOutput_);
				nextState_ = kIntake;
			} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
				flywheelController_->Enable();
				SmartDashboard::PutNumber("Feeder", feederMotorOutput_);
				robot_->SetFeederOutput(feederMotorOutput_);
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			} else if (autoTimeITDesired_) {
				nextState_ = kTimeIntake;
			}
			break;
	case (kIntake):
			SmartDashboard::PutString("State", "kIntake");
			SmartDashboard::PutNumber("Intake Output:", intakeMotorOutput_);
			if (humanControl_->GetIntakeDesired()) {
				robot_->SetIntakeOutput(intakeMotorOutput_);
				nextState_ = kIntake;
			} else {
				robot_->SetIntakeOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kFeederAndFlywheel):
			SmartDashboard::PutString("State", "kFeederAndFlywheel");
//			SmartDashboard::PutNumber("Feeder", feederMotorOutput_);
			SmartDashboard::PutNumber("Flywheel Output", robot_->GetFlywheelMotorOutput());
			SmartDashboard::PutBoolean("flywheel desired: ", humanControl_->GetFlywheelDesired());
			if (humanControl_->GetFlywheelDesired() || GetAutoFlywheelDesired()) {
				robot_->SetFeederOutput(feederMotorOutput_);
				robot_->SetIntakeOutput(intakeMotorOutput_);
				SmartDashboard::PutNumber("Intake Output:", intakeMotorOutput_);
				nextState_ = kFeederAndFlywheel;
			} else {
				flywheelController_->Disable();
				robot_->SetFeederOutput(0.0);
				robot_->SetIntakeOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kClimber):
			SmartDashboard::PutString("State", "kClimber");
			if (humanControl_->GetClimberDesired()) {
				robot_->SetClimberOutput(climberMotorOutput_);
				nextState_ = kClimber;
			} else {
				robot_->SetClimberOutput(0.0);
				nextState_ = kIdle;
			}
			break;
	case (kTimeIntake):
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
	m_stateVal_ = nextState_;
}

bool SuperstructureController::GetAutoFlywheelDesired() {
	return autoFlywheelDesired_;
}

bool SuperstructureController::GetAutoIntakeDesired() {
	return autoTimeITDesired_;
}

bool SuperstructureController::GetAutoFinishedIntake() {
	return autoFinishedIntake_;
}

void SuperstructureController::SetAutoFlywheelDesired(bool desired) {
	autoFlywheelDesired_ = desired;
}

void SuperstructureController::SetAutoTimeITDesired(bool desired) {
	autoTimeITDesired_ = desired;
}

void SuperstructureController::SetAutoIntakeTime(int seconds) {
	autoIntakeTime_ = seconds;
}

void SuperstructureController::SetAutoFinishedIntake(bool finished) {
	autoFinishedIntake_ = finished;
}

void SuperstructureController::SetOutput() {
	if (humanControl_->GetReverseIntakeDesired()) {
		intakeMotorOutput_ = -(fabs(intakeMotorOutput_));
	} else {
		intakeMotorOutput_ = fabs(intakeMotorOutput_);
	}
	if (humanControl_->GetReverseFeederDesired()) {
		feederMotorOutput_ = -(fabs(feederMotorOutput_));
	} else {
		feederMotorOutput_ = fabs(feederMotorOutput_);
	}
}

SuperstructureController::~SuperstructureController() {

}
