#include "DriverStation/ControlBoard.h"
#include "WPILib.h"

ControlBoard::ControlBoard() {
	// Joysticks
	leftJoy_ = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy_ = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy_ = new Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB_ = new Joystick(OPERATOR_JOY_B_USB_PORT);

	leftJoyX_ = 0;
	leftJoyY_ = 0;
	rightJoyX_ = 0;
	rightJoyY_ = 0;

	// Buttons
	driveDirectionButton = new ButtonReader(leftJoy_, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton = new ButtonReader(operatorJoyB_, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy_, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy_, QUICK_TURN_BUTTON_PORT);

	// Drivetrain variables
	reverseDriveDesired_ = false;
	gearShiftDesired_ = false;
	arcadeDriveDesired_ = false;
	quickTurnDesired_ = false;
}

void ControlBoard::ReadControls() {
	ReadAllButtons();
	leftJoyX_ = leftJoy_->GetX();
	leftJoyY_ = leftJoy_->GetY();
	rightJoyX_ = rightJoy_->GetX();
	rightJoyY_ = rightJoy_->GetY();

	// Drivetrain variables
	reverseDriveDesired_ = driveDirectionButton->IsDown();
	gearShiftDesired_ = gearShiftButton->StateJustChanged();
	arcadeDriveDesired_ = !arcadeDriveButton->IsDown();
	quickTurnDesired_ = quickTurnButton->IsDown();
}

double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
		case(kRightJoy):
			switch(a) {
				case(kX):
					return rightJoyX_;
				case(kY):
					return rightJoyY_;
			}
			break;
		case(kLeftJoy):
			switch(a) {
				case(kX):
					return leftJoyX_;
				case(kY):
					return leftJoyY_;
			}
			break;
	}
	return 0.0;
}

void ControlBoard::ReadAllButtons() {
	driveDirectionButton->ReadValue();
	gearShiftButton->ReadValue();
	arcadeDriveButton->ReadValue();
	quickTurnButton->ReadValue();
}

// Returns true if reverse drive is desired
bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired_;
}

// Returns true if gear shifting is desired
bool ControlBoard::GetGearShiftDesired() {
	return gearShiftDesired_;
}

// Returns true if arcade drive is desired
bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired_;
}

// Returns true if quick turn is desired
bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired_;
}

ControlBoard::~ControlBoard() {
	// TODO Auto-generated destructor stub
}
