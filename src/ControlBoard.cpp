#include <ControlBoard.h>

ControlBoard::ControlBoard() {
	// Four joysticks
	leftJoy  = new Joystick(LEFT_JOY_USB_PORT);
	rightJoy = new Joystick(RIGHT_JOY_USB_PORT);
	operatorJoy = new Joystick(OPERATOR_JOY_USB_PORT);
	operatorJoyB = new Joystick(OPERATOR_JOY_B_USB_PORT);

	// Drivetrain buttons
	driveDirectionButton = new ButtonReader(leftJoy, DRIVE_DIRECTION_BUTTON_PORT);
	gearShiftButton = new ButtonReader(operatorJoyB, HIGH_LOW_GEAR_BUTTON_PORT);
	arcadeDriveButton = new ButtonReader(rightJoy, ARCADE_DRIVE_BUTTON_PORT);
	quickTurnButton = new ButtonReader(rightJoy, QUICK_TURN_BUTTON_PORT);
	brakeButton = new ButtonReader(leftJoy, BRAKE_BUTTON_PORT);

	// Joystick values
	leftJoyX = 0.0;
	leftJoyY = 0.0;
	rightJoyX = 0.0;
	rightJoyY = 0.0;

	// Drivetrain variables
	reverseDriveDesired = false;
	gearShiftDesired = false;
	arcadeDriveDesired = false;
	quickTurnDesired = false;
	brakeDesired = false;
}

void ControlBoard::ReadControls() {
//	// Reads buttons
//	driveDirectionButton->ReadValue();
//	gearShiftButton->ReadValue();
//	arcadeDriveButton->ReadValue();
//	quickTurnButton->ReadValue();
//	brakeButton->ReadValue();
//
//	// Reads joystick positions
//	leftJoyX = leftJoy->GetX();
//	leftJoyY = leftJoy->GetY();
//	rightJoyX = rightJoy->GetX();
//	rightJoyY = rightJoy->GetY();
//
//	// Sets desired booleans
//	reverseDriveDesired = driveDirectionButton->IsDown();
//	gearShiftDesired = gearShiftButton->StateJustChanged();
//	arcadeDriveDesired = !arcadeDriveButton->IsDown();
//	quickTurnDesired = quickTurnButton->IsDown();
//	brakeDesired = brakeButton->IsDown();
}

//Returns the joystick and axis being used
double ControlBoard::GetJoystickValue(Joysticks j, Axes a) {
	switch (j) {
	case (kLeftJoy):
		if (a == kX) {
			return leftJoyX;
		} else if (a == kY) {
			return leftJoyY;
		}
		break;
	case (kRightJoy):
		if (a == kX) {
			return rightJoyX;
		} else if (a == kY) {
			return rightJoyY;
		}
		break;
	}
	return 0.0;
}

bool ControlBoard::GetReverseDriveDesired() {
	return reverseDriveDesired;
}

bool ControlBoard::GetGearShiftDesired() {
	return gearShiftDesired;
}

bool ControlBoard::GetArcadeDriveDesired() {
	return arcadeDriveDesired;
}

bool ControlBoard::GetQuickTurnDesired() {
	return quickTurnDesired;
}

bool ControlBoard::GetBrakeDesired() {
	return brakeDesired;
}
