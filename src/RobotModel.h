#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include <navx/AHRS.h>
#include "CANTalon.h"
#include "Ports2017.h"
#include "../ext/ini/ini.h"

class RobotModel {
public:
	/**
	 * Creates all objects regarding the robot parts. Configures the left and right
	 * talons. Initializes all variables
	 */
	RobotModel();

	/**
	 * Destructor
	 */
	~RobotModel();

	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	/**
	 * Resets timer
	 */
	void ResetTimer();

	/**
	 * @returns timer get
	 */
	double GetTime();

	/* -------------------- DRIVE --------------------  */

	/**
	 * Configures talons and constants for PID
	 * @param wheel side (let, right, all) of talons for the PID loop
	 * @param pFac P factor for PID loop
	 * @param iFac I factor for PID loop
	 * @param dFac D factor for PID loop
	 * @param fFac feedforward term for PIDF
	 */
	void SetTalonPIDConfig(Wheels wheel, double pFac, double iFac, double dFac, double fFac);

	/**
	 * Sets talons to Motion Profiling mode
	 */
	void SetMotionProfileMode();

	/**
	 * Sets the talons to percentVbus mode (the usual (-1, 1) range outputs)
	 */
	void SetPercentVBusDriveMode();

	/**
	 * Clears motion profile trajectors for both sides
	 */
	void ClearMotionProfileTrajectories();

	/**
	 * Sets specified side of talons to a value
	 * @param wheel the side of talons (left, right, all)
	 * @param value the value to set the talon motors to (-1, 1)
	 */
	void SetDriveValues(Wheels wheel, double value);

	/**
	 * Sets drive to high gear
	 */
	void SetHighGear();

	/**
	 * Sets drive to low gear
	 */
	void SetLowGear();

	/**
	 * @param wheel the side of the talon
	 * @return native encoder value
	 */
	double GetDriveEncoderValue(Wheels wheel);

	/**
	 * @return left wheel distance
	 */
	double GetLeftDistance();
	/**
	 * @return right wheel distance
	 */
	double GetRightDistance();

	/**
	 * Returns angle from navX
	 */
	double GetNavXYaw();

	/**
	 * Zeroes the navX
	 */
	void ZeroNavXYaw();
	void RefreshIni();

	/* ------------------ SUPERSTRUCTURE ------------------  */
	/**
	 * @return feeder output value for the shooter
	 */
	double GetFeederOutput();
	/**
	 * @param output the output for the feeder of the shooter
	 */
	void SetFeederOutput(double output);

	/**
	 * @return climber motor output
	 */
	double GetClimberOutput();
	/**
	 * @param output for the climber motor
	 */
	void SetClimberOutput(double output);

	/**
	 * @return intake motor output for the shooter
	 */
	double GetIntakeOutput();
	/**
	 * @param output the output for the intake motor of the shooter
	 */
	void SetIntakeOutput(double output);

	/**
	 * @returns gearPivotEncoder_
	 */
	Encoder* GetGearPivotEncoder();
	/**
	 * @return gearPivotMotor_
	 */
	Victor* GetGearPivotMotor();

	/**
	 *	@return output of the gear intake motor
	 */
	double GetGearIntakeOutput();

	/**
	 * @param output the output for the gear intake motor
	 */
	void SetGearIntakeOutput(double output);

	/**
	 * @return the pivot output of the gear intake position motor
	 */
	double GetGearPivotOutput();

	/**
	 * @param output the output motor for the gear intake's position motor
	 */
	void SetGearPivotOutput(double output);

	/**
	 * @return flywheel encoder
	 */
	Encoder* GetFlywheelEncoder();

	/**
	 * @return flywheel motor
	 */
	Victor* GetFlywheelMotor();
	/**
	 * @return flywheel motor output
	 */
	double GetFlywheelMotorOutput();

	/**
	 * @return gearInRobot_
	 */
	bool GetGearInRobot();
	/**
	 * @param gearInRobot whether the gear is in the robot or not
	 */
	void SetGearInRobot(bool gearInRobot);
	/**
	 * Checks if the gear intake position goes past the limit switch
	 */
	void GearUpdate();

	/**
	 * Set the "passive" gear mechanism solenoid
	 * @param dir open or closing the gear mechanism
	 */
	void SetGearMech(bool dir);

	/**
	 * @return limit switch state
	 */
	bool GetLimitSwitchState();
	/* ------------------------------------------------------  */

	/**
	 * @return total power from the pdp
	 */
	double GetTotalPower();

	Ini *pini_;

	CANTalon *leftMaster_, *rightMaster_, *leftSlave_, *rightSlave_;	//TODO move to private

private:
	Timer *timer_;
	AHRS *navX_;

	Victor *flywheelMotor_, *feederMotor_, *climberMotor_, *intakeMotor_, *gearPivotMotor_, *gearIntakeMotor_;
	Compressor *compressor_;
	DoubleSolenoid *gearShiftSolenoid_;
	Solenoid *gearMechSolenoid_;
	Encoder *leftDriveEncoder_, *rightDriveEncoder_, *flywheelEncoder_, *gearPivotEncoder_;
	DigitalInput *limitSwitch_;
	PowerDistributionPanel* pdp_;
	bool gearInRobot_, distSensorCurr_, distSensorLast_;
};

#endif /* SRC_ROBOTMODEL_H_ */
