#include "Controllers/SuperstructureController.h"
#include "WPILib.h"
#include <cmath>

const double FLYWHEEL_FEEDER_DIFF_TIME = 3.0;
const double GEAR_PIVOT_DOWN_TIME = 2.0;
const double GEAR_OUTTAKE_TIME = 1.0;

SuperstructureController::SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	// Flywheel variables
	adjustedFlywheelVelocity_ = 0.0;
	desiredFlywheelVelocity_ = 11.5; // ft/sec, tune value
	// TODO put all this in the ini file
	// TODO add dial for changing velocity
	expectedFlywheelMotorOutput_ = 0.7; // Tune value
	feederMotorOutput_ = 0.85;
	flywheelStartTime_ = 0.0;
	intakeMotorOutput_ = 0.4; // postive for comp

	// Climber variables
	climberMotorOutput_ = 0.9;

	// Gear intake mech variables
	gearIntakeMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearIntakeMotorOutput", 0.9);			// CHECK SIGNS
	gearOuttakeMotorOutput_ = -gearIntakeMotorOutput_;
	gearPivotMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearPivotMotorOutput", 0.3);
	gearDownTicks_ = robot_->pini_->getf("GEAR MECH", "gearDownTicks", 60.0);
	gearDeployTicks_ = robot_->pini_->getf("GEAR MECH", "gearDeployTicks", 30.0);

	gearPositionController_ = NULL;

	// Setting up flywheel PID Controller
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

	isFlywheelStarted_ = false;
	flywheelStartTime_ = 0.0;

	autoFlywheelDesired_ = false;
	autoTimeIntakeDesired_ = false;
	autoStartedIntake_ = false;
	autoFinishedIntake_ = false;

	autoIntakeTime_ = 0.0;
	autoIntakeStartTime_ = 0.0;

	double gearPFac = robot_->pini_->getf("GEAR MECH", "gearPFac", 0.1);
	double gearIFac = robot_->pini_->getf("GEAR MECH", "gearIFac", 0.0);
	double gearDFac = robot_->pini_->getf("GEAR MECH", "gearDFac", 0.0);
	double gearFFac = robot_->pini_->getf("GEAR MECH", "gearFFac", 0.0);
	//pFac_, iFac_, dFac_, fFac_, robot_->GetFlywheelEncoder(), robot_->GetFlywheelMotor(), 0.02
	//gearPositionController_ = PIDController(gearPFac, gearIFac, gearDFac, gearFFac, robot_->GetGearIntakeEncoder(), robot_->GetGearPivotMotor(), 0.02);
	gearPositionController_ = new PIDController(0.01, 0.0, 0.0, 0.0, robot_->GetGearPivotEncoder(), robot_->GetGearPivotMotor(), 0.02);
	gearPositionController_->SetSetpoint(gearDeployTicks_);	// check
	gearPositionController_->SetOutputRange(-0.9, 0.9);
	gearPositionController_->SetAbsoluteTolerance(10.0);
	gearPositionController_->SetContinuous(false);

	// Initializing variables for gear intake mech
	gearMechPos_ = false;	// True is in, false is out
//	isGearPivotPositionUp_ = true; // True is up, false is down
//	isGearPivotDownStarted_ = false;
//	isGearOuttakeStarted_ = false;

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
	robot_->SetGearPivotOutput(0.0);
	robot_->SetGearIntakeOutput(0.0);

	feederMotorOutput_ = -fabs(feederMotorOutput_);
	intakeMotorOutput_ = -fabs(intakeMotorOutput_); // positive for comp

	isFlywheelStarted_ = false;
//	isGearPivotDownStarted_ = false;
//	isGearOuttakeStarted_ = false;

	flywheelStartTime_ = 0.0;
//	gearPivotDownTimeStarted_ = 0.0;
//	gearOuttakeTimeStarted_ = 0.0;

	gearMechPos_ = false;
//	isGearPivotPositionUp_ = true;

	autoFlywheelDesired_ = false;
	autoTimeIntakeDesired_ = false;
	autoStartedIntake_ = false;
	autoFinishedIntake_ = false;

	autoIntakeTime_ = 0.0;
	autoIntakeStartTime_ = 0.0;

	gearIntakeMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearIntakeMotorOutput", 0.9);			// CHECK SIGNS
	gearOuttakeMotorOutput_ = -gearIntakeMotorOutput_;
	gearPivotMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearPivotMotorOutput", 0.3);
	gearDownTicks_ = robot_->pini_->getf("GEAR MECH", "gearDownTicks", 25.0);
	gearDeployTicks_ = robot_->pini_->getf("GEAR MECH", "gearDeployTicks", 30.0);
}

void SuperstructureController::Update(double currTimeSec, double deltaTimeSec) {
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
		robot_->SetGearPivotOutput(0.0);
		robot_->SetGearIntakeOutput(0.0);
		gearMechPos_ = false;
//		isGearPivotPositionUp_ = true;
		gearPositionController_->Reset();
		gearPositionController_->Disable();
		nextState_ = kGearIntakeMoveUp;
//		nextState_ = kIdle;
		break;
	case kIdle:
		SmartDashboard::PutString("State", "kIdle");
		nextState_ = kIdle;
//		if (humanControl_->GetIntakeDesired()) {
//			robot_->SetIntakeOutput(intakeMotorOutput_);
//		} else {
//			robot_->SetIntakeOutput(0.0);
//		}

		if (humanControl_->GetClimberDesired()) {
			printf("IN CLIMBER!!!!!\n");
			robot_->SetClimberOutput(climberMotorOutput_);
		} else {
			robot_->SetClimberOutput(0.0);
		}

		if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
			flywheelController_->Enable();
			nextState_ = kFeederAndFlywheel;
		}

//		if (isGearPivotPositionUp_ && !robot_->GetLimitSwitchState()) {	// If the robot thinks the gear intake mech is up but the limit switch doesn't think so
//			nextState_ = kGearIntakeMoveUp;
//		}

//		if (!isGearPivotPositionUp_) {	// If gear intake mech is down, run the gear intake motor
//			robot_->SetGearIntakeOutput(gearIntakeMotorOutput_);
//		} else {
//			robot_->SetGearIntakeOutput(0.0);
//		}


//		if (humanControl_->GetGearIntakeDownDesired()) {
//			printf("HELLO!!!");
//			robot_->SetGearPivotOutput(0.3);
//		} else {
//			robot_->SetGearPivotOutput(0.0);
//		}

//		if (humanControl_->GetGearIntakeUpDesired()) {
//			printf("BYEEE");
//			robot_->SetGearPivotOutput(-0.3);
//		} else {
//			robot_->SetGearPivotOutput(0.0);
//		}

		if (humanControl_->GetGearIntakeDesired()) {
			printf("gear intake desired\n");
			robot_->SetGearIntakeOutput(0.9);
		} else if (humanControl_->GetGearOuttakeDesired()) {
			printf("gear outtake desired\n");
			robot_->SetGearIntakeOutput(-0.9);
		} else {
			robot_->SetGearIntakeOutput(0.0);
		}

		if (humanControl_->GetGearIntakeAdjustUpDesired()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			printf("GetGearIntakeAdjustUpDesired\n");
			robot_->SetGearPivotOutput(-gearPivotMotorOutput_);
		} else if (humanControl_->GetGearIntakeAdjustDownDesired() && robot_->GetLimitSwitchState()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			printf("GetGearIntakeAdjustDownDesired\n");
			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
		} else {
			robot_->SetGearPivotOutput(0.0);
		}

		if (humanControl_->GetGearIntakeDownDesired() && robot_->GetLimitSwitchState()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			nextState_ = kGearIntakeMoveDown;
		}

		if (humanControl_->GetGearIntakeUpDesired()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			nextState_ = kGearIntakeMoveUp;
		}

		if (humanControl_->GetGearDeployDesired()) {
			gearPositionController_->Enable();
			nextState_ = kDeployGear;
		}

		if (autoTimeIntakeDesired_) {
			nextState_ = kTimeIntake;
		}

		break;
	case kFeederAndFlywheel:
		SmartDashboard::PutString("State", "kFeederAndFlywheel");
		SmartDashboard::PutNumber("Flywheel Output", robot_->GetFlywheelMotorOutput());
		SmartDashboard::PutNumber("Flywheel Error", flywheelController_->GetError());
		SmartDashboard::PutNumber("Flywheel Velocity", robot_->GetFlywheelEncoder()->GetRate());
		SmartDashboard::PutNumber("Flywheel Distance", robot_->GetFlywheelEncoder()->GetDistance());
		SmartDashboard::PutNumber("Flywheel Pulses", robot_->GetFlywheelEncoder()->GetRaw());

		flywheelController_->SetPID(pFac_, iFac_, dFac_, fFac_);
		if (!isFlywheelStarted_) {
			flywheelStartTime_ = robot_->GetTime();
			isFlywheelStarted_ = true;
			nextState_ = kFeederAndFlywheel;
		} else if (robot_->GetTime() - flywheelStartTime_ < FLYWHEEL_FEEDER_DIFF_TIME) {
			nextState_ = kFeederAndFlywheel;
			printf("time - flywheelStartTime: %f\n", robot_->GetTime() - flywheelStartTime_);
		} else if (humanControl_->GetFlywheelDesired() || autoFlywheelDesired_) {
			printf("IN FEEDER\n");
			robot_->SetFeederOutput(feederMotorOutput_);
//				robot_->SetIntakeOutput(intakeMotorOutput_);
			nextState_ = kFeederAndFlywheel;
		} else {
			robot_->SetFeederOutput(0.0);
//				robot_->SetIntakeOutput(0.0);
			flywheelController_->Disable();
			isFlywheelStarted_ = false;
			nextState_ = kIdle;
		}
		break;
	case kGearIntakeMoveDown:	// TODO do we need a sensor to figure out whether the intake is truly down?
		SmartDashboard::PutString("State", "kGearIntakeMoveDown");
		gearPositionController_->Reset();
		gearPositionController_->Disable();

		if (robot_->GetGearPivotEncoder()->Get() < gearDownTicks_) {
			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
			nextState_ = kGearIntakeMoveDown;
		} else {
			nextState_ = kIdle;
		}
//		if (!isGearPivotDownStarted_) {
//			if (!isGearPivotPositionUp_) {	// If the gear intake is down
//				nextState_ = kIdle;
//			} else {	// If the gear intake is not down, start the motors
//				gearPivotDownTimeStarted_ = robot_->GetTime();
//				robot_->SetGearPivotOutput(gearPivotMotorOutput_);
//				isGearPivotDownStarted_ = true;
//				nextState_ = kGearIntakeMoveDown;
//			}
//		} else if (robot_->GetTime() - gearPivotDownTimeStarted_ < GEAR_PIVOT_DOWN_TIME) {
//			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
//			SmartDashboard::PutNumber("Gear Mech Encoder", robot_->GetGearIntakeEncoder()->Get());
//			nextState_ = kGearIntakeMoveDown;
//		} else {	// When the gear intake going down is finished
//			SmartDashboard::PutNumber("Gear Mech Encoder", robot_->GetGearIntakeEncoder()->Get());
//			robot_->SetGearPivotOutput(0.0);
//			isGearPivotDownStarted_ = false;
//			isGearPivotPositionUp_ = false;
//			nextState_ = kIdle;
//		}
		break;
	case kGearIntakeMoveUp:
		SmartDashboard::PutString("State", "kGearIntakeMoveUp");
		gearPositionController_->Reset();
		gearPositionController_->Disable();

		//if (robot_->GetGearPivotEncoder()->Get() < 1.0) {		// (!robot_->GetLimitSwitchState()) || r// limit switch is true when not pressed and false when pressed
//			robot_->SetGearPivotOutput(0.0);
//			robot_->GetGearPivotEncoder()->Reset();
//			nextState_ = kIdle;
//		} else {
		if (robot_->GetGearPivotEncoder()->Get() > 1.0) {
			robot_->SetGearPivotOutput(-gearPivotMotorOutput_);
			nextState_ = kGearIntakeMoveUp;
		} else {
			nextState_ = kIdle;
		}
//		if (robot_->GetLimitSwitchState()) {	// If the limit switch detects that it is at its angular position
//			isGearPivotPositionUp_ = true;
//			robot_->SetGearPivotOutput(0.0);
//			robot_->GetGearIntakeEncoder()->Reset();
//			SmartDashboard::PutNumber("Gear Mech Encoder", robot_->GetGearIntakeEncoder()->Get());
//			nextState_ = kIdle;
//		} else {	// If it is not in position (aka still down)
//			robot_->SetGearPivotOutput(-gearPivotMotorOutput_);
//			SmartDashboard::PutNumber("Gear Mech Encoder", robot_->GetGearIntakeEncoder()->Get());
//			nextState_ = kGearIntakeMoveUp;
//		}
		break;
	case kDeployGear:
		SmartDashboard::PutString("State", "kDeployGear");

		SmartDashboard::PutNumber("Gear Pivot Output", robot_->GetGearPivotMotor()->Get());
		SmartDashboard::PutNumber("Gear Pivot Error", gearPositionController_->GetError());
		SmartDashboard::PutNumber("Gear Pivot Pulses", robot_->GetGearPivotEncoder()->GetRaw());
		SmartDashboard::PutNumber("Gear Pivot Distance", robot_->GetGearPivotEncoder()->GetDistance());

		// TODO go down while deploying
		if (gearPositionController_->OnTarget()) {
			//robot_->SetGearIntakeOutput(gearOuttakeMotorOutput_);
			printf("gear mech on target\n");
			nextState_ = kIdle;
		} else {
			nextState_ = kDeployGear;
		}
//		if (humanControl_->GetGearIntakeDesired()) {
//			robot_->SetGearIntakeOutput(gearIntakeMotorOutput_);
//		} else {
//			robot_->SetGearIntakeOutput(0.0);
//		}

//		if (robot_->GetGearIntakeEncoder()->Get() <= gearDownTicks_) {
//			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
//			nextState_ = kDeployGear;
//		} else if(robot_->GetGearIntakeEncoder()->Get())
//		} else {
//			nextState_ = kIdle;
//		}
//		robot_->SetGearIntakeOutput(-gearIntakeMotorOutput_);
//		if (!isGearOuttakeStarted_) {		// If just started deploying gear
//			isGearOuttakeStarted_ = true;
//			gearOuttakeTimeStarted_ = robot_->GetTime();
//			nextState_ = kDeployGear;
//		} else if (robot_->GetTime() - gearOuttakeTimeStarted_ < GEAR_OUTTAKE_TIME) {	// Outtake gear for a set time
//			nextState_ = kDeployGear;
//		} else {
//			if (!isGearPivotDownStarted_) {	// If the gear intake mech just starting moving down
//				gearPivotDownTimeStarted_ = robot_->GetTime();
//				robot_->SetGearPivotOutput(gearPivotMotorOutput_);
//				isGearPivotDownStarted_ = true;
//				nextState_ = kDeployGear;
//			} else if (robot_->GetTime() - gearPivotDownTimeStarted_ < GEAR_PIVOT_DOWN_TIME) {
//				robot_->SetGearPivotOutput(gearPivotMotorOutput_);
//				nextState_ = kDeployGear;
//			} else {	// If the gear intake mech finished deploying
//				robot_->SetGearPivotOutput(0.0);
//				robot_->SetGearIntakeOutput(0.0);
//				isGearPivotDownStarted_ = false;
//				isGearOuttakeStarted_ = false;
//				isGearPivotPositionUp_ = false;
//				nextState_ = kIdle;
//			}
//		}
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
