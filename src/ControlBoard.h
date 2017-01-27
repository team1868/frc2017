#ifndef SRC_CONTROLBOARD_H_
#define SRC_CONTROLBOARD_H_

#include "WPILib.h"
#include "ButtonReader.h"
#include "Ports.h"

class ControlBoard {
public:
	ControlBoard();
	~ControlBoard() {}

	void ReadControls();

	// Joysticks and Axes for switch case
	enum Joysticks {kLeftJoy, kRightJoy};
	enum Axes {kX, kY};

	// Drive joystick accessors
	double GetJoystickValue(Joysticks j, Axes a);

	// Drive button accessors
	bool GetReverseDriveDesired();
	bool GetGearShiftDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();
	bool GetBrakeDesired();

private:
	// Joysticks and buttons
	Joystick *leftJoy, *rightJoy, *operatorJoy, *operatorJoyB;
	ButtonReader *driveDirectionButton, *gearShiftButton, *arcadeDriveButton, *quickTurnButton, *brakeButton;

	// Desired states of drivetrain
	bool reverseDriveDesired, gearShiftDesired, arcadeDriveDesired, quickTurnDesired, brakeDesired;

	// Desired values for driving and pivoting
	double leftJoyX, leftJoyY, rightJoyX, rightJoyY;

};

#endif /* SRC_CONTROLBOARD_H_ */
