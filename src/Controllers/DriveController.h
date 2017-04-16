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
	DriveController(RobotModel* robot, ControlBoard *humanControl, NavXPIDSource *navX, TalonEncoderPIDSource *talonEncoderSource);
	~DriveController();

	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
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
	void TankDrive(double myLeft, double myRight);
	void QuickTurn(double myRight, double turnConstant);

	int GetDriveDirection();

	double HandleDeadband(double value, double deadband);
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
