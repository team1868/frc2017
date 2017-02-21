#ifndef SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_
#define SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_

#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/PathCommand.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/AlignWithPegCommand.h"
#include "Auto/Commands/GearCommand.h"
#include "Auto/Commands/AlignWithHighGoalCommand.h"
#include "Auto/Commands/HighGoalShootCommand.h"
#include "Auto/PIDInputSource.h"
#include "RobotModel.h"
#include "Controllers/SuperstructureController.h"

class OneGearHighShootMode : public AutoMode {
public:
	OneGearHighShootMode(RobotModel *robot, SuperstructureController *superstructure, NavXPIDSource *navXSource, TalonEncoderPIDSource *talonSource);
	virtual ~OneGearHighShootMode();
	void CreateQueue();
	void Init(); 	// to put in AutoMode
	void RefreshIni();
	bool IsDone();
private:
	RobotModel *robot_;
	SuperstructureController *superstructure_;

	AutoCommand *firstCommand_;

	PathCommand *liftPath_, *highGoalPath_;
	AlignWithPegCommand *alignWithPegCommand_;
	GearCommand *gearCommand_;
	AlignWithHighGoalCommand *alignWithHighGoalCommand_;
	HighGoalShootCommand *highGoalShootCommand_;

	NavXPIDSource *navXSource_;
	TalonEncoderPIDSource *talonSource_;
};

#endif /* SRC_AUTO_MODES_ONEGEARHIGHSHOOTMODE_H_ */
