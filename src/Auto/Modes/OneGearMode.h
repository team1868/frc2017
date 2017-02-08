#ifndef SRC_AUTO_MODES_ONEGEARMODE_H_
#define SRC_AUTO_MODES_ONEGEARMODE_H_

extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/PathCommand.h"
#include "Auto/Commands/AutoCommand.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/PIDInputSource.h"
#include "RobotModel.h"

class OneGearMode : public AutoMode {
public:
	OneGearMode(RobotModel *robot);
	virtual ~OneGearMode();
	void CreateQueue();
	void Init(); 	// to put in AutoMode
	//void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	RobotModel *robot_;
	AutoCommand *firstCommand_;
	PathCommand *liftPath_;
	PivotCommand *pivotCommand_;

	//PathCommand *liftPath2_; 	// TO REPLACE THIS WITH PIVOT COMMAND
};

#endif /* SRC_AUTO_MODES_ONEGEARMODE_H_ */
