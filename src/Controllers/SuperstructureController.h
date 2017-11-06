#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "DriverStation/RemoteControl.h"

/**
 * A class to control the superstructure
 */
class SuperstructureController {
public:
	/**
	 * SuperstructureController constructor that sets up the controller
	 * @param myRobot a RobotModel
	 * @param myHumanControl a ControlBoard
	 */
	SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl);

	/**
	 * Destructor
	 */
	~SuperstructureController();

	/**
     * Refresh ini, reset timers, reset states
	 */
	void Reset();

	/**
   	 * Controls superstructure with a state machine
	 * @param currTimeSec a double is the current time in seconds
	 * @param deltaTimeSec a double is the update interval in seconds
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * Gets necessary values from the ini file
	 */
	void RefreshIni();

	// Auto mutator methods

	/**
	 * Sets a
	 * @param currTimeSec a double is the current time in seconds
	 * @param deltaTimeSec a double is the update interval in seconds
	 */

	// State machine states
	enum SuperstructureState {
		kInit, kIdle, kGearIntakeMoveDown, kGearIntakeMoveUp, kDeployGear
	};

private:
	RobotModel *robot_;
	RemoteControl *humanControl_;

	// Controls active gear mechanism
	PIDController *gearPositionController_;

	// State variables
	uint32_t currState_;
	uint32_t nextState_;

	// Climber variables
	double climberMotorOutput_;

	// Active gear mechanism variables
	double gearIntakeMotorOutput_, gearOuttakeMotorOutput_, gearPivotMotorOutput_, gearDownTicks_, gearDeployTicks_;
	double initialGearPivotDownTime_, initialGearPivotUpTime_, initialGearDeployTime_;
	bool gearMechPos_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
