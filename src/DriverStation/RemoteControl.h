#ifndef SRC_REMOTECONTROL_H_
#define SRC_REMOTECONTROL_H_

class RemoteControl {
public:
	// Joysticks
	enum Joysticks { kRightJoy, kLeftJoy };
	enum Axes { kX, kY };

	// Drive joystick accessors
	virtual void ReadControls() = 0;
	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	// Drive controller button accessors
	virtual bool GetReverseDriveDesired() = 0;
	virtual bool GetGearShiftDesired() = 0;
	virtual bool GetArcadeDriveDesired() = 0;
	virtual bool GetQuickTurnDesired() = 0;

	// Superstructure controller button accessors
	virtual bool GetFlywheelDesired() = 0;
	virtual bool GetIntakeDesired() = 0;
	virtual bool GetClimberDesired() = 0;
	virtual bool GetReverseIntakeDesired() = 0;
	virtual bool GetReverseFeederDesired() = 0;
	virtual bool GetGearMechOutDesired() = 0;
	virtual double GetFlywheelVelAdjust() = 0;

	virtual ~RemoteControl() {}
};

#endif /* SRC_REMOTECONTROL_H_ */
