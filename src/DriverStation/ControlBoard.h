#ifndef SRC_CONTROLBOARD_H_
#define SRC_CONTROLBOARD_H_

#include "WPILib.h"
#include "Ports2017.h"
#include "DriverStation/ButtonReader.h"
#include "DriverStation/RemoteControl.h"

class ControlBoard : public RemoteControl {
public:
	ControlBoard();

	void ReadControls();
	double GetJoystickValue(Joysticks j, Axes a);

	// Drive controller button accessors
	bool GetReverseDriveDesired();
	bool GetGearShiftDesired();
	bool GetArcadeDriveDesired();
	bool GetQuickTurnDesired();

	virtual ~ControlBoard();

private:
	// Desired values for driving and pivoting
	double leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_;

	bool reverseDriveDesired_, gearShiftDesired_, arcadeDriveDesired_, quickTurnDesired_;

	// Joysticks
	Joystick *leftJoy_, *rightJoy_, *operatorJoy_, *operatorJoyB_;

	// Buttons
	ButtonReader *driveDirectionButton, *gearShiftButton, *arcadeDriveButton, *quickTurnButton;

	void ReadAllButtons();
};

#endif /* SRC_CONTROLBOARD_H_ */