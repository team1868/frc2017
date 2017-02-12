#ifndef SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_
#define SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_

#include "WPILib.h"
#include "RobotModel.h"
#include "DriverStation/RemoteControl.h"

class SuperstructureController {
public:
	SuperstructureController(RobotModel* myRobot, RemoteControl* myHumanControl);
	void Reset();
	void Update(double currTimeSec, double deltaTimeSec);
	virtual ~SuperstructureController();

	//auto boolean mutator methods
	void SetAutoFlywheelDesired(bool desired);

	//auto boolean accessor methods
	bool GetAutoFlywheelDesired();

	enum SuperstructureState {
			kInit, kIdle, kIntake, kFeederAndFlywheel, kClimber
	};
private:
	RobotModel* robot_;
	RemoteControl* humanControl_;

	PIDController *intakeController_;
	PIDController *flywheelController_;

	uint32_t m_stateVal_;
	uint32_t nextState_;

	double desiredIntakeVelocity_, desiredFlywheelVelocity_, expectedIntakeMotorOutput_, expectedFlywheelMotorOutput_,
		   feederMotorOutput_, climberMotorOutput_;

	//auto booleans
	bool autoFlywheelDesired_;

};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
