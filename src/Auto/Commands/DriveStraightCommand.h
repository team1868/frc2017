#ifndef SRC_DRIVESTRAIGHTCOMMAND_H_
#define SRC_DRIVESTRAIGHTCOMMAND_H_

#include "WPILib.h"
#include <math.h>
#include "RobotModel.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/PIDInputSource.h"
#include "Auto/PIDOutputSource.h"

class DriveStraightCommand : public AutoCommand {
public:
	/**
	 * Constructor that generates the DriveStraight Command
	 * @param navXSource angle input for the angle PID Loop
	 * @param talonEncoderSource distance input for distance PID Loop
	 * @param anglePIDOutput output for the angle PID loop
	 * @param distancePIDOutput ouptut for the distance PID loop
	 * @param robot robot model
	 */
	DriveStraightCommand(NavXPIDSource* navXSource, TalonEncoderPIDSource* talonEncoderSource,
			AnglePIDOutput* anglePIDOutput, DistancePIDOutput* distancePIDOutput, RobotModel* robot,
			double desiredDistance);

	/**
	 * Destructor
	 */
	virtual ~DriveStraightCommand();

	/**
	 * Initializes values for the distance and angle PID loops and enables the loops.
	 */
	void Init();

	/**
	 *	Checks if the PID is on target or if it times out, then sets isDone_ is true and
	 *	stops the motors. Otherwise, sets the left and right motor values to the PID outputs.
	 */
	void Update(double currTime, double deltaTime);

	/**
	 * @return isDone_
	 */
	bool IsDone();

	/**
	 * Gets PID values from the ini file
	 */
	void GetIniValues();

private:
	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *anglePIDOutput_;
	DistancePIDOutput *distancePIDOutput_;
	PIDController *anglePID_;
	PIDController *distancePID_;
	RobotModel* robot_;

	double rPFac_, rIFac_, rDFac_;
	double dPFac_, dIFac_, dDFac_;
	double initialAngle_;
	double initialAvgDistance_;
	double desiredDistance_;
	double desiredTotalAvgDistance_;
	double leftMotorOutput_, rightMotorOutput_;
	double initialDriveTime_, diffDriveTime_;
	bool isDone_;
};

#endif /* SRC_DRIVESTRAIGHTCOMMAND_H_ */
