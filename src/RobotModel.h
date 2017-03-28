#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include <navx/AHRS.h>
#include "CANTalon.h"
#include "Ports2017.h"
#include "../ext/ini/ini.h"

class RobotModel {
public:
	RobotModel();
	~RobotModel();

	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	void ResetTimer();
	double GetTime();

	/* -------------------- DRIVE --------------------  */

	void SetTalonPIDConfig(Wheels wheel, double pFac, double iFac, double dFac, double fFac);

	void SetMotionProfileMode();	// Sets the talons to motion profile mode
	void SetPercentVBusDriveMode();	// Sets the talons to percentVbus mode (the usual (-1,1) range outputs)
	void ClearMotionProfileTrajectories();

	void SetDriveValues(Wheels wheel, double value);

	void SetHighGear();
	void SetLowGear();

	double GetDriveEncoderValue(Wheels wheel);		// Returns native encoder value
	double GetLeftDistance();						// Returns left wheel distance (converted)
	double GetRightDistance();						// Returns right wheel distance (converted)

	double GetNavXYaw();							// Returns negative navX
	void ZeroNavXYaw();
	void RefreshIni(); 								// Refreshes the ini file

	/* ------------------ SUPERSTRUCTURE ------------------  */
	double GetFeederOutput();
	void SetFeederOutput(double output);

	double GetClimberOutput();
	void SetClimberOutput(double output);

	double GetIntakeOutput();
	void SetIntakeOutput(double output);

	Encoder* GetGearIntakeEncoder();
	double GetGearIntakeOutput();
	void SetGearIntakeOutput(double output);

	double GetGearPivotOutput();
	void SetGearPivotOutput(double output);

	Encoder* GetFlywheelEncoder();
	Victor* GetFlywheelMotor();
	double GetFlywheelMotorOutput();

	bool GetGearInRobot();
	void SetGearInRobot(bool gearInRobot);
	void GearUpdate();
	void SetGearMech(bool dir);

	bool GetLimitSwitchState();
	/* ------------------------------------------------------  */

	Ini *pini_;

	CANTalon *leftMaster_, *rightMaster_, *leftSlave_, *rightSlave_;	//TODO move to private

private:
	Timer *timer_;
	AHRS *navX_;

	Victor *flywheelMotor_, *feederMotor_, *climberMotor_, *intakeMotor_, *gearPivotMotor_, *gearIntakeMotor_;
	Compressor *compressor_;
	DoubleSolenoid *gearShiftSolenoid_;
	Solenoid *gearMechSolenoid_;
	Encoder *leftDriveEncoder_, *rightDriveEncoder_, *flywheelEncoder_, *gearIntakeMechEncoder_;
	DigitalInput *limitSwitch_;

	bool gearInRobot_, distSensorCurr_, distSensorLast_;
};

#endif /* SRC_ROBOTMODEL_H_ */
