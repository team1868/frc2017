#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/ControlBoard.h"
#include "DriverStation/RemoteControl.h"

class SuperstructureController {
public:
	SuperstructureController(RobotModel* myRobot, ControlBoard* myHumanControl);
	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	virtual ~SuperstructureController();

	//auto mutator methods
	void SetAutoFlywheelDesired(bool desired);
	void SetAutoTimeITDesired(bool desired);
	void SetAutoIntakeTime(int seconds);
	void SetAutoFinishedIntake(bool finished);

	//auto accessor methods
	bool GetAutoFlywheelDesired();
	bool GetAutoIntakeDesired();
	bool GetAutoFinishedIntake();

	void SetOutput();

	enum SuperstructureState {
			kInit, kIdle, kIntake, kFeederAndFlywheel, kClimber, kTimeIntake
	};
private:
	RobotModel* robot_;
	RemoteControl* humanControl_;

	PIDController *flywheelController_;

	uint32_t m_stateVal_;
	uint32_t nextState_;

	double desiredFlywheelVelocity_, expectedFlywheelMotorOutput_,
		   feederMotorOutput_, climberMotorOutput_, intakeMotorOutput_;

	//auto variables
	bool autoFlywheelDesired_, autoTimeITDesired_, autoStartedIntake_, autoFinishedIntake_;
	double autoIntakeTime_, autoIntakeStartTime_;
};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */

