#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "DriverStation/RemoteControl.h"

class SuperstructureController {
public:
	SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl);
	~SuperstructureController();

	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();

	// Auto mutator methods
	void SetAutoFlywheelDesired(bool desired);
	void SetAutoTimeIntakeDesired(bool desired);
	void SetAutoIntakeTime(int seconds);
	void SetAutoFinishedIntake(bool finished);

	// Auto accessor methods
	bool GetAutoFlywheelDesired();
	bool GetAutoIntakeDesired();
	bool GetAutoFinishedIntake();

	void SetOutputs();		// TODO make work

	enum SuperstructureState {
		kInit, kIdle, kGearIntakeMoveDown, kGearIntakeMoveUp, kDeployGear
	};

private:
	RobotModel *robot_;
	RemoteControl *humanControl_;

	PIDController *gearPositionController_;

	uint32_t currState_;
	uint32_t nextState_;

	// Climber variables
	double climberMotorOutput_;

	// Gear intake mech variables
	double gearIntakeMotorOutput_, gearOuttakeMotorOutput_, gearPivotMotorOutput_, gearDownTicks_, gearDeployTicks_;
	double initialGearPivotDownTime_, initialGearPivotUpTime_, initialGearDeployTime_;
	//gearPivotDownTimeStarted_, gearOuttakeTimeStarted_;
//
	bool gearMechPos_;// isGearPivotPositionUp_, isGearPivotDownStarted_, isGearOuttakeStarted_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
