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
		kInit, kIdle, kIntake, kFeederAndFlywheel, kClimber, kTimeIntake
	};

private:
	RobotModel* robot_;
	RemoteControl* humanControl_;

	PIDController *flywheelController_;

	uint32_t currState_;
	uint32_t nextState_;

	double expectedFlywheelVelocity_, adjustedFlywheelVelocity_, desiredFlywheelVelocity_, expectedFlywheelMotorOutput_, feederMotorOutput_,
		   climberMotorOutput_, intakeMotorOutput_, flywheelStartTime_;

	bool flywheelStarted_;

	// Auto variables
	bool autoFlywheelDesired_, autoTimeIntakeDesired_, autoStartedIntake_, autoFinishedIntake_;
	double autoIntakeTime_, autoIntakeStartTime_, pFac_, iFac_, dFac_, fFac_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
