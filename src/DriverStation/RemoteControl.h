#ifndef SRC_REMOTECONTROL_H_
#define SRC_REMOTECONTROL_H_

class RemoteControl {
public:
	// Joysticks
	enum Joysticks { kRightJoy, kLeftJoy };
	enum Axes { kX, kY, kZ };

	// Drive joystick accessors
	virtual void ReadControls() = 0;
	virtual double GetJoystickValue(Joysticks j, Axes a) = 0;

	// Drive controller button accessors
	virtual bool GetReverseDriveDesired() = 0;
	virtual bool GetHighGearDesired() = 0;
	virtual bool GetArcadeDriveDesired() = 0;
	virtual bool GetQuickTurnDesired() = 0;
	virtual bool GetAlignWithPegDesired() = 0;

	// Superstructure controller button accessors
	virtual bool GetFlywheelDesired() = 0;
	virtual bool GetIntakeDesired() = 0;
	virtual bool GetClimberDesired() = 0;
	virtual bool GetReverseIntakeDesired() = 0;
	virtual bool GetReverseFeederDesired() = 0;
	virtual bool GetGearMechOutDesired() = 0;
	virtual double GetFlywheelVelAdjust() = 0;
	virtual bool GetGearCameraDesired() = 0;

	virtual bool GetGearIntakeUpDesired() = 0;
	virtual bool GetGearIntakeDownDesired() = 0;
	virtual bool GetGearDeployDesired() = 0;
	virtual bool GetGearIntakeDesired() = 0;
	virtual bool GetGearOuttakeDesired() = 0;
	virtual bool GetGearIntakeAdjustUpDesired() = 0;
	virtual bool GetGearIntakeAdjustDownDesired() = 0;
	virtual ~RemoteControl() {}
};

#endif /* SRC_REMOTECONTROL_H_ */
