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
	 * Sets state of robot and ControlBoard
	 * @param robot a RobotModel
	 * @param humanControl a ControlBoard
	 */
	DriveController(RobotModel* robot, ControlBoard *humanControl, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoderSource);
	~DriveController();

	void Reset();

	void UpdateMotionProfile();

	/**
	 * Updates joystick and button values from driver, also determines type of drive for robot to use
	 * @param currTimeSec
	 * @param deltaTimeSec (which is currTimeSec - lastTimeSec)
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * IS NEVER DONE
	 */
	bool IsDone();

	void PrintDriveValues();

	//auto mutator
	void SetAlignWithPegDesired(bool desired);

	//accessor
	bool GetAlignWithPegDesired();

	enum DriveState {
		kInitialize, kTeleopDrive, kAlignWithPeg
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
	void ArcadeDrive(double myX, double myY);

	/**
	 * Senses how much each joystick is pushed and uses values to turn wheels
	 * @param Left a double how much left joystick is pushed
	 * @param Right a double how much right joystick is pushed
	 */
	void TankDrive(double Left, double Right);

	/**
	 * Allows robot to turn quickly, at sharp angle
	 * @param myRight a double allows robot to turn right or left
	 */
	void QuickTurn(double myRight);

	/**
	 * Indicates direction of drive (forwards or backwards)
	 */
	int DriveDirection();

	/**
	 * Gets current state of drive
	 */
	int GetDriveState();

	RobotModel *robot_;
	ControlBoard *humanControl_;

	bool isDone_;

	uint32_t currState_;
	uint32_t nextState_;

	// For DriveStraightPID in teleop
	PIDController *driveStraightPIDController_;
	NavXPIDSource *navXSource_;
	AlignWithPegCommand *pegCommand_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *anglePIDOutput_;

	bool isDriveStraightStarted_, alignWithPegStarted_;
	double desiredAngle_;
	double angleOutput_;
	double pFac_, iFac_, dFac_;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
