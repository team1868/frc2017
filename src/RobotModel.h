#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include <navx/AHRS.h>
#include "CANTalon.h"
#include "Ports2017.h"
#include "../ext/ini/ini.h"

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel();
	void ResetTimer();
	double GetTime();

	// Talon accessors and mutators
	void SetTalonPIDConfig(Wheels wheel, double pFac, double iFac, double dFac, double fFac);

	void SetMotionProfile();	// Sets the talons to motion profile mode
	void ClearMotionProfileTrajectories();
	void SetPercentVBusDrive();	// Sets the talons to percentVbus mode (the usual (-1,1) range outputs)

	void SetDriveValues(Wheels wheel, double value);

	void SetHighGear();
	void SetLowGear();

	double GetDriveEncoderValue(Wheels wheel);
	double GetLeftDistance();
	double GetRightDistance();

	double GetNavXYaw();
	void ZeroNavXYaw();
	void RefreshIni(); //refreshes the ini file

	//Superstructure accessors and mutators
	double GetFeederOutput();
	void SetFeederOutput(double output);

	double GetClimberOutput();
	void SetClimberOutput(double output);

	double GetIntakeOutput();
	void SetIntakeOutput(double output);

	Encoder* GetFlywheelEncoder();
	Victor* GetFlywheelMotor();
	double GetFlywheelMotorOutput();

	bool GetGearInRobot();
	void SetGearInRobot(bool gearInRobot);
	void GearUpdate();
	void SetGearMechOut();

	Ini *pini;

	CANTalon *leftMaster_, *rightMaster_, *leftSlave_, *rightSlave_;	//TODO move to private
private:
	Timer *timer_;
	AHRS *navX_;

	Victor *flywheelMotor_, *feederMotor_, *climberMotor_, *intakeMotor_;
	DoubleSolenoid *gearShiftSolenoid_;
	Solenoid *gearMechSolenoid_;
	Encoder *flywheelEncoder_;
	DigitalInput *distanceSensor_;

	bool gearInRobot_, distSensorCurr_, distSensorLast_;
};

#endif /* SRC_ROBOTMODEL_H_ */
