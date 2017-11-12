#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "Auto/PIDInputSource.h"
#include "Auto/PIDOutputSource.h"
#include "Auto/Commands/AlignWithPegCommand.h"
#include "Profiler.h"

class DriveController {
public:
	/**
	 * DriveController constructor
	 * @param robot a RobotModel
	 * @param humanControl a ControlBoard
	 * @param navX a NavXPIDSource
	 * @param talonEncoderSource a TalonEncoderPIDSource
	 */
	DriveController(RobotModel* robot, ControlBoard *humanControl, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoderSource);

	/**
	 * Deconstructor
	 */
	~DriveController();

	/**
	 * Resets drive controller (reset thrust, rotate, quickturn sensitivities)
	 */
	void Reset();

	/**
	 * Updates joy values and teleop currState with switch statement
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return true if command is done
	 */
	bool IsDone();

	/**
	 * Prints direction, distance, yaw, encoder values to SmartDashboard
	 */
	void PrintDriveValues();

	enum DriveState {
		kTeleopDrive, kAlignWithPeg
	};

private:
	/**
	 * Allows one joystick to make robot turn, while the other joystick alters speed and direction (forwards or backwards) of the robot.
	 * Does not allow maxOutput to be greater than 1.0
	 * Does not allow robot to turn if thrust is less than 0.1
	 * Does not allow robot to drive forwards/backwards if thrust is less than 0.2
	 * @param myX a double rotate value
	 * @param myY a double thrust value (forwards/backwards)
	 */
	void ArcadeDrive(double myX, double myY, double thrustSensitivity, double rotateSensitivity);
	/**
		 * Each joystick controls a corresponding side of the robot
		 * @param myLeft gives the value of thrust for the left side
		 * @param myRight gives the value of thrust for the right side
		 */
	void TankDrive(double myLeft, double myRight);

	/**
		 * Button on the joystick that, when pressed, allows the robot to turn in place quicker
		 * @param myRight used to get the cubic adjustment
		 * @param turnConstant the amount to turn
		 */
	void QuickTurn(double myRight, double turnConstant);

	/**
	 * @return -1 if reverseDesired true, else 1
	 */
	int GetDriveDirection();

	/**
	 * @return value if greater than deadband, else 0
	 */
	double HandleDeadband(double value, double deadband);

	/**
	 * @return rotation sensitivity adjustment based on z-axis, adjustable by driver preference (currently around 3)
	 */
	double GetCubicAdjustment(double value, double adjustmentConstant);

	RobotModel *robot_;
	ControlBoard *humanControl_;

	uint32_t currState_;
	uint32_t nextState_;

	double thrustSensitivity_, rotateSensitivity_, quickTurnSensitivity_;

	NavXPIDSource *navXSource_;
	AlignWithPegCommand *pegCommand_;
	TalonEncoderPIDSource *talonEncoderSource_;
	bool alignWithPegStarted_;

	bool isDone_;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
