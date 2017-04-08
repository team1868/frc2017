#ifndef SRC_PIVOTCOMMAND_H_
#define SRC_PIVOTCOMMAND_H_

#include "Auto/Commands/AutoCommand.h"
#include "Auto/PIDInputSource.h"
#include "RobotModel.h"
#include "Profiler.h"

class PivotPIDTalonOutput : public frc::PIDOutput {
public:
	PivotPIDTalonOutput(RobotModel *robot);
	void PIDWrite(double output);
	virtual ~PivotPIDTalonOutput();
	double GetOutput();
private:
	RobotModel *robot_;
	double output_;
};

/**
 * A class implementing Pivot PID the WPILibrary PID Controller
 */
class PivotCommand : public AutoCommand {
public:
	/**
	 * PivotCommand constructor that gives the desired turn and whether or not it is absolute position
	 * @param robot a RobotModel
	 * @param desiredAngle a double that is the angle of the turn
	 * @param isAbsolutePosition a bool that represents whether the angle is absolute position or delta angle
	 * @param navXSource a NavXPIDSource
	 */
	PivotCommand(RobotModel *robot, double desiredAngle, bool isAbsolutePosition, NavXPIDSource* navXSource);

	/**
	 * Destructor
	 */
	virtual ~PivotCommand();

	/**
	 * Sets PID values to 0, gets PID from navX, sets Setpoint, continuous to false, output range, enables pivotPID
	 */
	void Init();

	/**
	 * Refresh ini, set initYaw to navX PID, create new PIDController, PivotPID, and enables it
	 */
	void Reset();

	/**
	 * Sets PID values to 0.0
	 */
	void GetIniValues();

	/**
	 * If PivotPID not done, checks if PivotPID is on target and if TalonOutput is less than the minimum
	 * drive output, we set isDone to true and Reset and Disable PivotPID
	 * Note: times out at 5 seconds from start of PivotCommand
	 * @param currTimeSec a double is the current time in seconds
	 * @param deltaTimeSec a double is the update interval in seconds
	 */
	void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return isDone_
	 */
	bool IsDone();

private:
	/**
	 * Calculating the value of delta angle if the angle is absolute position.
	 * @param desiredAngle a double is the original angle passed in from constructor.
	 * @return deltaAngle which is the amount we have to turn to get to the absolute position
	 */
	double CalculateDeltaAngle(double desiredAngle); // For absolute position

	double pFac_, iFac_, dFac_;
	double desiredDeltaAngle_;
	double initYaw_;
	bool isDone_;

	int numTimesOnTarget_;

	RobotModel *robot_;
	PIDController *pivotPID_;
	NavXPIDSource *navXSource_;
	PivotPIDTalonOutput *talonOutput_;

	double pivotCommandStartTime_;

	/**
	 * Minimum output we would correct for, if less, than it can be considered done
	 */
	double minDrivePivotOutput_;
};

#endif /* SRC_PIVOTCOMMAND_H_ */
