#include "DriverStation/ControlBoard.h"
#include "WPILib.h"

ControlBoard::ControlBoard() {
	// Joysticks
	leftJoy_ = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy_ = new Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB_ = new Joystick(OPERATOR_JOY_B_USB_PORT);

	leftJoyX_ = 0.0;
	leftJoyY_ = 0.0;
	leftJoyZ_ = 0.0;
	rightJoyX_ = 0.0;
	rightJoyY_ = 0.0;
	rightJoyZ_ = 0.0;

	// Buttons for drive
	driveDirectionButton_ = new ButtonReader(rightJoy_, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton_ = new ButtonReader(leftJoy_, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton_ = new ButtonReader(operatorJoy_, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton_ = new ButtonReader(rightJoy_, QUICK_TURN_BUTTON_PORT);
	alignWithPegButton_ = new ButtonReader(leftJoy_, ALIGN_WITH_PEG_BUTTON_PORT);

	// Buttons for superstructure
	flywheelSwitch_ = new ButtonReader(operatorJoy_, FLYWHEEL_SWITCH_PORT);
	intakeSwitch_ = new ButtonReader(operatorJoy_, INTAKE_SWITCH_PORT);
	climberSwitch_ = new ButtonReader(operatorJoy_, CLIMBER_SWITCH_PORT);
	reverseIntakeButton_ = new ButtonReader(operatorJoyB_, REVERSE_INTAKE_BUTTON_PORT);
	reverseFeederButton_ = new ButtonReader(operatorJoyB_, REVERSE_FEEDER_BUTTON_PORT);
	gearMechOutButton_ = new ButtonReader(operatorJoyB_, GEAR_MECH_OUT_BUTTON_PORT);
	gearSwitchButton_ = new ButtonReader(operatorJoyB_, CAMERA_SWITCH_BUTTON_PORT);
	gearIntakeUpButton_ = new ButtonReader(operatorJoyB_, GEAR_INTAKE_UP_BUTTON_PORT);
	gearIntakeDownButton_ = new ButtonReader(operatorJoyB_, GEAR_INTAKE_DOWN_BUTTON_PORT);
	gearDeployButton_ = new ButtonReader(operatorJoyB_, DEPLOY_GEAR_BUTTON_PORT);
	gearIntakeButton_ = new ButtonReader(operatorJoyB_, GEAR_INTAKE_BUTTON_PORT);
	gearOuttakeButton_ = new ButtonReader(operatorJoyB_, GEAR_OUTTAKE_BUTTON_PORT);
	gearIntakeAdjustUpButton_ = new ButtonReader(operatorJoyB_, GEAR_INTAKE_ADJUST_UP_BUTTON_PORT);
	gearIntakeAdjustDownButton_ = new ButtonReader(operatorJoyB_, GEAR_INTAKE_ADJUST_DOWN_BUTTON_PORT);

	// Auto switches
	leftAutoSwitch_ = new ButtonReader(operatorJoyB_, LEFT_AUTO_SWITCH_PORT);
	middleAutoSwitch_ = new ButtonReader(operatorJoyB_, MIDDLE_AUTO_SWITCH_PORT);
	rightAutoSwitch_ = new ButtonReader(operatorJoyB_, RIGHT_AUTO_SWITCH_PORT);

	// Drivetrain variables
	reverseDriveDesired_ = false;
	highGearDesired_ = false;
	arcadeDriveDesired_ = false;
	quickTurnDesired_ = false;
	alignWithPegDesired_= false;

	//Superstructure variables
	flywheelDesired_ = false;
	intakeDesired_ = false;
	climberDesired_ = false;
	reverseIntakeDesired_ = false;
	reverseFeederDesired_ = false;
	gearMechOutDesired_ = false;
	flywheelVelAdjust_ = 0.0;

	gearIntakeUpDesired_ = false;
	gearIntakeDownDesired_ = false;
	gearDeployDesired_ = false;
	gearIntakeDesired_ = false;
	//gearIntakeDesired_ = false;
	gearOuttakeDesired_ = false;
	gearIntakeAdjustUpDesired_ = false;
	gearIntakeAdjustDownDesired_ = false;

	cameraSwitchDesired_ = false;
	gearCameraDesired_ = true;

	leftAutoDesired_ = false;
	middleAutoDesired_ = false;
	rightAutoDesired_ = false;
	currAutoMode_ = kBlank;
	lastAutoMode_ = currAutoMode_;
}

void ControlBoard::ReadControls() {
	ReadAllButtons();
	ReadAutoSwitches();

	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();
	leftJoyZ_ = leftJoy_->GetZ();
	rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = rightJoy_->GetY();
	rightJoyZ_ = rightJoy_->GetZ();

	// Drivetrain variables
	reverseDriveDesired_ = driveDirectionButton_->IsDown();
	arcadeDriveDesired_ = arcadeDriveButton_->IsDown();
	quickTurnDesired_ = quickTurnButton_->IsDown();
	highGearDesired_ = !gearShiftButton_->IsDown();
	alignWithPegDesired_ = alignWithPegButton_->WasJustPressed();

	//Superstructure variables
	flywheelDesired_ = flywheelSwitch_->IsDown();
	intakeDesired_ = intakeSwitch_->IsDown();
	climberDesired_ = climberSwitch_->IsDown();
	reverseIntakeDesired_ = reverseIntakeButton_->WasJustPressed();
	reverseFeederDesired_ = reverseFeederButton_->WasJustPressed();
	gearMechOutDesired_ = gearMechOutButton_->WasJustPressed();
	flywheelVelAdjust_ = operatorJoy_->GetZ();
	cameraSwitchDesired_ = gearSwitchButton_->WasJustPressed();
//	gearIntakeUpDesired_ = gearIntakeUpButton_->WasJustPressed();
//	gearIntakeDownDesired_ = gearIntakeDownButton_->WasJustPressed();

	gearIntakeUpDesired_ = gearIntakeUpButton_->WasJustPressed();
	gearIntakeDownDesired_ = gearIntakeDownButton_->WasJustPressed();
	gearDeployDesired_ = gearDeployButton_->WasJustPressed();
	gearIntakeDesired_ = gearIntakeButton_->IsDown();
	gearOuttakeDesired_ = gearOuttakeButton_->IsDown();
	gearIntakeAdjustUpDesired_ = gearIntakeAdjustUpButton_->IsDown();
	gearIntakeAdjustDownDesired_ = gearIntakeAdjustDownButton_->IsDown();
}

void ControlBoard::ReadAutoSwitches() {
	 leftAutoSwitch_->ReadValue();
	 middleAutoSwitch_->ReadValue();
	 rightAutoSwitch_->ReadValue();

	char autoModec;
	leftAutoDesired_ = leftAutoSwitch_->IsDown();
	middleAutoDesired_ = middleAutoSwitch_->IsDown();
	rightAutoDesired_ = rightAutoSwitch_->IsDown();

	lastAutoMode_ = currAutoMode_;
	if (leftAutoDesired_) {
		currAutoMode_ = kLeftLift;
		autoModec = 'l';
	} else if (middleAutoDesired_) {
		currAutoMode_ = kMiddleLift;
		autoModec = 'm';
	} else if (rightAutoDesired_) {
		currAutoMode_ = kRightLift;
		autoModec = 'r';
	} else {
		currAutoMode_ = kBlank;
		autoModec = 'b';
	}
	if (currAutoMode_ != lastAutoMode_) {
		printf("Changed auto mode to: %c %d\n", autoModec, currAutoMode_);
	}
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case(kRightJoy):
			switch(a) {
				case(kX):
					return rightJoyX_;
				case(kY):
					return rightJoyY_;
				case(kZ):
					return rightJoyZ_;
			}
			break;
		case(kLeftJoy):
			switch(a) {
				case(kX):
					return leftJoyX_;
				case(kY):
					return leftJoyY_;
				case(kZ):
					return leftJoyZ_;
			}
			break;
	}
	return 0.0;
}

void ControlBoard::ReadAllButtons() {
	//reading drive buttons
	driveDirectionButton_->ReadValue();
	gearShiftButton_->ReadValue();
	arcadeDriveButton_->ReadValue();
	quickTurnButton_->ReadValue();
	alignWithPegButton_->ReadValue();

	//reading superstructure buttons
	flywheelSwitch_->ReadValue();
	intakeSwitch_->ReadValue();
	climberSwitch_->ReadValue();
	reverseIntakeButton_->ReadValue();
	reverseFeederButton_->ReadValue();
	gearMechOutButton_->ReadValue();
	gearIntakeUpButton_->ReadValue();
	gearIntakeDownButton_->ReadValue();
	gearDeployButton_->ReadValue();
	gearIntakeButton_->ReadValue();
	gearOuttakeButton_->ReadValue();
	gearIntakeAdjustUpButton_->ReadValue();
	gearIntakeAdjustDownButton_->ReadValue();

	//reading auto buttons
	leftAutoSwitch_->ReadValue();
	middleAutoSwitch_->ReadValue();
	rightAutoSwitch_->ReadValue();
}

// Returns true if reverse drive is desired
bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired_;
}

// Returns true if gear shifting is desired
bool ControlBoard::GetHighGearDesired() {
	return highGearDesired_;
}

// Returns true if arcade drive is desired
bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired_;
}

// Returns true if quick turn is desired
bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired_;
}

//Returns true if align with peg is desired in teleop
bool ControlBoard::GetAlignWithPegDesired() {
	return alignWithPegDesired_;
}

bool ControlBoard::GetFlywheelDesired() {
	return flywheelDesired_;
}

bool ControlBoard::GetIntakeDesired() {
	return intakeDesired_;
}

bool ControlBoard::GetClimberDesired() {
	return climberDesired_;
}

bool ControlBoard::GetReverseFeederDesired() {
	return reverseFeederDesired_;
}

bool ControlBoard::GetReverseIntakeDesired() {
	return reverseIntakeDesired_;
}

bool ControlBoard::GetGearMechOutDesired() {
	return gearMechOutDesired_;
}

double ControlBoard::GetFlywheelVelAdjust() {
	return flywheelVelAdjust_;
}

bool ControlBoard::GetGearIntakeUpDesired() {
	return gearIntakeUpDesired_;
}

bool ControlBoard::GetGearIntakeDownDesired() {
	return gearIntakeDownDesired_;
}

bool ControlBoard::GetGearDeployDesired() {
	return gearDeployDesired_;
}

bool ControlBoard::GetGearIntakeDesired() {
	return gearIntakeDesired_;
}

bool ControlBoard::GetGearOuttakeDesired() {
	return gearOuttakeDesired_;
}

bool ControlBoard::GetGearIntakeAdjustUpDesired() {
	return gearIntakeAdjustUpDesired_;
}

bool ControlBoard::GetGearIntakeAdjustDownDesired() {
	return gearIntakeAdjustDownDesired_;
}

bool ControlBoard::GetGearCameraDesired() {
	if (cameraSwitchDesired_) {
		gearCameraDesired_ = !gearCameraDesired_;
		SmartDashboard::PutBoolean("Camera Switched", true);
	}
	return gearCameraDesired_;
}

bool ControlBoard::GetLeftAutoDesired() {
	return leftAutoDesired_;
}

bool ControlBoard::GetRightAutoDesired() {
	return rightAutoDesired_;
}

bool ControlBoard::GetMiddleAutoDesired() {
	return middleAutoDesired_;
}

int ControlBoard::GetAutoModeDesired() {
	return currAutoMode_;
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}
