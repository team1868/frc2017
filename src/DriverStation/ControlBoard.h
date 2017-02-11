#ifndef SRC_CONTROLBOARD_H_
#define SRC_CONTROLBOARD_H_

#include "WPILib.h"
#include "Ports2017.h"
#include "DriverStation/ButtonReader.h"
#include "DriverStation/RemoteControl.h"

class ControlBoard : public RemoteControl {
public:
	/**
	 * Sets leftJoy_, rightJoy_, operatorJoy_, operatorJoyB_
	 * Initializes drivDirectionButton, gearShiftButton, arcadeDriveButton, quickTurnButton
	 * Sets reverseDriveDesired_, gearShiftDesired_, arcadeDriveDesired_, quickTurnDesired_ to false
	 */
	ControlBoard();
	/**
	 * Sets leftJoyX_, leftJoyY_, rightJoyX_, rightJoyY_
	 * Sets reverseDriveDesired_, gearShiftDesired_, arcadeDriveDesired_, quickTurnDesired_
	 */
	void ReadControls();
	/**
	 * returns rightJoy_ and leftJoy_ values
	 * @param j a Joystick
	 * @param a a Axes
	 */
	double GetJoystickValue(Joysticks j, Axes a);

	// Drive controller button accessors
	/**
	 * @return true if reserveDriveDesired_
	 */
	bool GetReverseDriveDesired();
	/**
	 *@return true if gear shift is desired
	 */
	bool GetGearShiftDesired();
	/**
	 * @return true if arcade drive is desired
	 */
	bool GetArcadeDriveDesired();
	/**
	 * @return true if quick turn is desired
	 */
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
