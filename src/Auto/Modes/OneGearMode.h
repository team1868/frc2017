#ifndef SRC_AUTO_MODES_ONEGEARMODE_H_
#define SRC_AUTO_MODES_ONEGEARMODE_H_

extern "C" {
#include <pathfinder/pathfinder.h>
}
#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/PathCommand.h"
#include "Auto/Commands/AutoCommand.h"
#include "RobotModel.h"

class OneGearMode : public AutoMode {
public:
	OneGearMode(RobotModel *robot);
	virtual ~OneGearMode();
	void Init(); 	// to put in AutoMode
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	RobotModel *robot_;
	PathCommand *liftOnePath_;
};

#endif /* SRC_AUTO_MODES_ONEGEARMODE_H_ */
