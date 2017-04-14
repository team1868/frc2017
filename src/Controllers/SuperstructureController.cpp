#include "Controllers/SuperstructureController.h"
#include "WPILib.h"
#include <cmath>

const double FLYWHEEL_FEEDER_DIFF_TIME = 3.0;
const double GEAR_PIVOT_DOWN_TIME = 2.0;
const double GEAR_OUTTAKE_TIME = 1.0;

SuperstructureController::SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl) {
	robot_ = myRobot;
	humanControl_ = myHumanControl;

	// Climber variables
	climberMotorOutput_ = 0.9;

	// Gear intake mech variables
	gearIntakeMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearIntakeMotorOutput", 0.9);			// CHECK SIGNS
	gearOuttakeMotorOutput_ = -gearIntakeMotorOutput_;
	gearPivotMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearPivotMotorOutput", 0.3);
	gearDownTicks_ = robot_->pini_->getf("GEAR MECH", "gearDownTicks", 60.0);
	gearDeployTicks_ = robot_->pini_->getf("GEAR MECH", "gearDeployTicks", 30.0);

	gearPositionController_ = NULL;

	// PASSIVE MECH
	gearMechPos_ = false;	// True is in, false is out

	currState_ = kInit;
	nextState_ = kInit;

	initialGearPivotDownTime_ = 0.0;
	initialGearPivotUpTime_ = 0.0;
	initialGearDeployTime_ = 0.0;
}

void SuperstructureController::Reset() {
	currState_ = kInit;
	nextState_ = kInit;

	robot_->SetClimberOutput(0.0);
	robot_->SetGearPivotOutput(0.0);
	robot_->SetGearIntakeOutput(0.0);

	gearMechPos_ = false;

	gearIntakeMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearIntakeMotorOutput", 0.9);			// CHECK SIGNS
	gearOuttakeMotorOutput_ = -gearIntakeMotorOutput_;
	gearPivotMotorOutput_ = robot_->pini_->getf("GEAR MECH", "gearPivotMotorOutput", 0.3);
	gearDownTicks_ = robot_->pini_->getf("GEAR MECH", "gearDownTicks", 25.0);
	gearDeployTicks_ = robot_->pini_->getf("GEAR MECH", "gearDeployTicks", 30.0);

	double gearPFac = robot_->pini_->getf("GEAR MECH", "gearPFac", 0.1);
	double gearIFac = robot_->pini_->getf("GEAR MECH", "gearIFac", 0.0);
	double gearDFac = robot_->pini_->getf("GEAR MECH", "gearDFac", 0.0);
	double gearFFac = robot_->pini_->getf("GEAR MECH", "gearFFac", 0.0);
	gearPositionController_ = new PIDController(gearPFac, gearIFac, gearDFac, gearFFac, robot_->GetGearPivotEncoder(), robot_->GetGearPivotMotor(), 0.02);
	gearPositionController_->SetSetpoint(gearDeployTicks_);	// check
	gearPositionController_->SetOutputRange(-0.9, 0.9);
	gearPositionController_->SetAbsoluteTolerance(10.0);
	gearPositionController_->SetContinuous(false);

	initialGearPivotDownTime_ = 0.0;
	initialGearPivotUpTime_ = 0.0;
	initialGearDeployTime_ = 0.0;
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
		robot_->SetIntakeOutput(0.0);
		robot_->SetFeederOutput(0.0);
		robot_->SetClimberOutput(0.0);
		robot_->SetGearPivotOutput(0.0);
		robot_->SetGearIntakeOutput(0.0);
		gearMechPos_ = false;
//		isGearPivotPositionUp_ = true;
		gearPositionController_->Reset();
		gearPositionController_->Disable();
//		nextState_ = kGearIntakeMoveUp;
		nextState_ = kIdle;
		break;

	case kIdle:
		SmartDashboard::PutString("State", "kIdle");
		nextState_ = kIdle;

		if (humanControl_->GetClimberDesired()) {
			printf("IN CLIMBER!!!!!\n");
			robot_->SetClimberOutput(climberMotorOutput_);
		} else {
			robot_->SetClimberOutput(0.0);
		}

		if (humanControl_->GetGearIntakeDesired()) {
			printf("gear intake desired\n");
			robot_->SetGearIntakeOutput(-0.9);
		} else if (humanControl_->GetGearOuttakeDesired()) {
			printf("gear outtake desired\n");
			robot_->SetGearIntakeOutput(0.9);
		} else {
			robot_->SetGearIntakeOutput(0.0);
		}

		if (humanControl_->GetGearIntakeAdjustUpDesired()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			printf("GetGearIntakeAdjustUpDesired\n");
			robot_->SetGearPivotOutput(-0.3);
		} else if (humanControl_->GetGearIntakeAdjustDownDesired()) { // && robot_->GetLimitSwitchState()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			printf("GetGearIntakeAdjustDownDesired\n");
			robot_->SetGearPivotOutput(0.3);
		} else {
			robot_->SetGearPivotOutput(0.0);
		}

		if (humanControl_->GetGearIntakeDownDesired()) { // && robot_->GetLimitSwitchState()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			initialGearPivotDownTime_ = robot_->GetTime();
			nextState_ = kGearIntakeMoveDown;
		}

		if (humanControl_->GetGearIntakeUpDesired()) {
			gearPositionController_->Reset();
			gearPositionController_->Disable();
			initialGearPivotUpTime_ = robot_->GetTime();
			nextState_ = kGearIntakeMoveUp;
		}

		if (humanControl_->GetGearDeployDesired()) {
			//gearPositionController_->Enable();			// TODO PUT THIS BACK!
			initialGearDeployTime_ = robot_->GetTime();
			nextState_ = kDeployGear;
		}

		break;

	case kGearIntakeMoveDown:	// TODO do we need a sensor to figure out whether the intake is truly down?
		SmartDashboard::PutString("State", "kGearIntakeMoveDown");
		gearPositionController_->Reset();
		gearPositionController_->Disable();

		if ((robot_->GetGearPivotEncoder()->Get() < gearDownTicks_) && (robot_->GetTime() - initialGearPivotDownTime_ < 4.0)) {
			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
			nextState_ = kGearIntakeMoveDown;
		} else {
			nextState_ = kIdle;
		}
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
		if ((robot_->GetGearPivotEncoder()->Get() > 1.0) && (robot_->GetTime() - initialGearPivotUpTime_ < 4.0)) {
			robot_->SetGearPivotOutput(-gearPivotMotorOutput_);
			nextState_ = kGearIntakeMoveUp;
		} else {
			nextState_ = kIdle;
		}
		break;

	case kDeployGear:
		SmartDashboard::PutString("State", "kDeployGear");

		SmartDashboard::PutNumber("Gear Pivot Output", robot_->GetGearPivotMotor()->Get());
		SmartDashboard::PutNumber("Gear Pivot Error", gearPositionController_->GetError());
		SmartDashboard::PutNumber("Gear Pivot Pulses", robot_->GetGearPivotEncoder()->GetRaw());
		SmartDashboard::PutNumber("Gear Pivot Distance", robot_->GetGearPivotEncoder()->GetDistance());

//		// TODO go down while deploying
//		if (gearPositionController_->OnTarget()) {
//			//robot_->SetGearIntakeOutput(gearOuttakeMotorOutput_);
//			printf("gear mech on target\n");
//			nextState_ = kIdle;
//		} else {
//			nextState_ = kDeployGear;
//		}

		if (robot_->GetTime() - initialGearDeployTime_ < 2.0) {
			robot_->SetGearIntakeOutput(0.9);
			robot_->SetGearPivotOutput(gearPivotMotorOutput_);
			robot_->SetDriveValues(RobotModel::kAllWheels, -0.3);
			nextState_ = kDeployGear;
		} else {
			robot_->SetGearIntakeOutput(0.0);
			robot_->SetGearPivotOutput(0.0);
			robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
			nextState_ = kIdle;
		}

		break;
	}
	currState_ = nextState_;
}

void SuperstructureController::SetOutputs() {

}

void SuperstructureController::RefreshIni() {

}

SuperstructureController::~SuperstructureController() {

}
