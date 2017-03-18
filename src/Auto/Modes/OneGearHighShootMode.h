#ifndef SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_
#define SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_

#include <Auto/Commands/PathCommand.h>
#include "RobotModel.h"
#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/AlignWithPegCommand.h"
#include "Auto/Commands/GearCommand.h"
#include "Auto/Commands/AlignWithHighGoalCommand.h"
#include "Auto/Commands/HighGoalShootCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"
#include "Auto/Commands/WaitingCommand.h"
#include "Auto/PIDInputSource.h"
#include "Controllers/SuperstructureController.h"

class OneGearHighShootMode : public AutoMode {
public:
	OneGearHighShootMode(RobotModel *robot, SuperstructureController *superstructure, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource, bool isLeft);
	virtual ~OneGearHighShootMode();
	void CreateQueue();
	void Init(); 	// To put in AutoMode
	void RefreshIni();
	bool IsDone();

private:
	RobotModel *robot_;
	SuperstructureController *superstructure_;

	AutoCommand *firstCommand_;

	PathCommand *liftPath_, *highGoalPath_;
	AlignWithPegCommand *alignWithPegCommand_;	// unused
	WaitingCommand *waitingCommand_;
	GearCommand *gearCommand_;	// unused
	AlignWithHighGoalCommand *alignWithHighGoalCommand_;
	HighGoalShootCommand *highGoalShootCommand_;

	ParallelAutoCommand *highGoalPathAndShootCommand_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonSource_;
};

#endif /* SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_ */
