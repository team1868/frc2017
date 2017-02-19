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

	//auto boolean mutator methods
	void SetAutoFlywheelDesired(bool desired);

	//auto boolean accessor methods
	bool GetAutoFlywheelDesired();

	void SetOutput();

	enum SuperstructureState {
			kInit, kIdle, kIntake, kFeederAndFlywheel, kClimber
	};
private:
	RobotModel* robot_;
	RemoteControl* humanControl_;

	PIDController *flywheelController_;

	uint32_t m_stateVal_;
	uint32_t nextState_;

	double desiredFlywheelVelocity_, expectedFlywheelMotorOutput_,
		   feederMotorOutput_, climberMotorOutput_, intakeMotorOutput_;

	//auto booleans
	bool autoFlywheelDesired_;

};

#endif /* SRC_CONTROLLERS_SUPERSTRUCTURECONTROLLER_H_ */
