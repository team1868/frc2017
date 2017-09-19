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
	bool GetHighGearDesired();
	/**
	 * @return true if arcade drive is desired
	 */
	bool GetArcadeDriveDesired();
	/**
	 * @return true if quick turn is desired
	 */
	bool GetQuickTurnDesired();
	/**
	 * @returns true if align with peg is desired in Teleop :)
	 */
	bool GetAlignWithPegDesired();

	//Superstructure button accessors
	bool GetFlywheelDesired();
//	bool GetGearIntakeDesired();
	bool GetIntakeDesired();
	bool GetClimberDesired();
	bool GetReverseIntakeDesired();
	bool GetReverseFeederDesired();
	bool GetGearMechOutDesired();
	double GetFlywheelVelAdjust();
	bool GetGearCameraDesired();
	bool GetGearIntakeUpDesired();
	bool GetGearIntakeDownDesired();
	bool GetGearDeployDesired();
	bool GetGearIntakeDesired();
	bool GetGearOuttakeDesired();
	bool GetGearIntakeAdjustUpDesired();
	bool GetGearIntakeAdjustDownDesired();

	bool GetLeftAutoDesired();
	bool GetRightAutoDesired();
	bool GetMiddleAutoDesired();

	virtual ~ControlBoard();

private:
	// Desired values for driving and pivoting
	double leftJoyX_, leftJoyY_, leftJoyZ_, rightJoyX_, rightJoyY_, rightJoyZ_;

	// Flywheel Velocity dial
	double flywheelVelAdjust_;

	// Bool values for drive
	bool reverseDriveDesired_, highGearDesired_, arcadeDriveDesired_, quickTurnDesired_, alignWithPegDesired_;
	// Bool values for superstructure
	bool flywheelDesired_, intakeDesired_, climberDesired_, reverseIntakeDesired_, reverseFeederDesired_, gearMechOutDesired_, cameraSwitchDesired_,
		gearCameraDesired_, gearIntakeUpDesired_, gearIntakeDownDesired_, gearDeployDesired_, gearIntakeDesired_, gearOuttakeDesired_,
		gearIntakeAdjustUpDesired_, gearIntakeAdjustDownDesired_;

	bool leftAutoDesired_, middleAutoDesired_, rightAutoDesired_;

	// Joysticks
	Joystick *leftJoy_, *rightJoy_, *operatorJoy_, *operatorJoyB_;

	// Buttons for drive
	ButtonReader *driveDirectionButton_, *gearShiftButton_, *arcadeDriveButton_, *quickTurnButton_, *alignWithPegButton_;

	// Buttons for superstructure
	ButtonReader *flywheelSwitch_, *intakeSwitch_, *climberSwitch_, *reverseIntakeButton_, *reverseFeederButton_, *gearMechOutButton_,
				 *gearSwitchButton_, *gearIntakeUpButton_, *gearIntakeDownButton_, *gearDeployButton_, *gearIntakeButton_,
				 *gearOuttakeButton_, *gearIntakeAdjustUpButton_, *gearIntakeAdjustDownButton_;

	// Buttons for auto
	ButtonReader *leftAutoSwitch_, *middleAutoSwitch_, *rightAutoSwitch_;

	void ReadAllButtons();
};

#endif /* SRC_CONTROLBOARD_H_ */
