#ifndef SRC_AUTO_MODES_ONEGEARMODE_H_
#define SRC_AUTO_MODES_ONEGEARMODE_H_

#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/AutoCommand.h"
#include <Auto/Commands/AlignWithPegCommand.h>
#include "Auto/Commands/DriveStraightCommand.h"
#include <Auto/Commands/PivotCommand.h>
#include "Auto/PIDInputSource.h"
#include "RobotModel.h"

class OneGearMode : public AutoMode {
public:
	/**
	 * Initializes desired distance and angle according to the chosen auto mode. Creates the commands for
	 * distance, angle, and aligning with peg.
	 * @param robot a RobotModel
	 * @param navXSource a NavXPIDSource for angle commands
	 * @param talonSource a TalonEncoderPIDSource for distance commands
	 * @param kAutoMode the specified auto mode (left, right, middle pegs)
	 */
	OneGearMode(RobotModel *robot, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, int kAutoMode);

	/**
	 * Destructor
	 */
	virtual ~OneGearMode();

	/**
	 * Creates the queue of commands by setting the next commands for every command. Starting the currentCommand with the firstCommand.
	 */
	void CreateQueue();

	/**
	 * Starts the first command and keeps the current command as the first command.
	 */
	void Init();
	void RefreshIni();

	/**
	 * @returns false
	 */
	bool IsDone();
private:
	RobotModel *robot_;
	AutoCommand *firstCommand_;

	DriveStraightCommand *driveStraightCommand_;
	PivotCommand *pivotCommand_;

	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	//PathCommand *liftPath_;
	AlignWithPegCommand *alignWithPegCommand_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonSource_;

	int autoMode_;
};

#endif /* SRC_AUTO_MODES_ONEGEARMODE_H_ */
