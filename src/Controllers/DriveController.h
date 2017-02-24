#ifndef SRC_CONTROLLERS_DRIVECONTROLLER_H_
#define SRC_CONTROLLERS_DRIVECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "Auto/PIDInputSource.h"
#include "Auto/PIDOutputSource.h"

class DriveController {
public:
	/**
	 * Sets state of robot and ControlBoard
	 * @param robot a RobotModel
	 * @param humanControl a ControlBoard
	 */
	DriveController(RobotModel* robot, ControlBoard *humanControl, NavXPIDSource *navX);
	void Reset();
	void UpdateMotionProfile();
	/**
	 * Updates joystick and button values from driver, also determines type of drive for robot to use
	 * @param currTimeSec a double current time in seconds
	 * @param deltaTimeSec a double
	 */
	void Update(double currTimeSec, double deltaTimeSec);
	/**
	 * @return whether or not isDone is true, gets called in PathCommand
	 */
	bool IsDone();											// Gets called in PathCommand::IsDone()

	void PrintDriveValues();

	~DriveController();

	enum DriveState {
		kInitialize, kTeleopDrive, kMotionProfile
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
	PIDController *driveStraight_;
	NavXPIDSource *navX_;
	AnglePIDOutput *anglePIDOutput_;

	//MotionProfileExample *leftExample_, *rightExample_;
	bool isDone_;
	bool isDriveStraightStarted_;
	double desiredAngle_;
	double angleOutput_;
	double pFac_, iFac_, dFac_;

	uint32_t currState_;
	uint32_t nextState_;
};

#endif /* SRC_CONTROLLERS_DRIVECONTROLLER_H_ */
