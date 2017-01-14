#ifndef SRC_AUTO_MODES_TESTMODE_H_
#define SRC_AUTO_MODES_TESTMODE_H_

#include "Auto/Modes/AutoMode.h"
#include "Auto/Commands/PathCommand.h"
#include "RobotModel.h"

class TestMode : public AutoMode {
public:
	TestMode(DriveController *driveController);
	virtual ~TestMode();
	void Init(); 	// to put in AutoMode
	void Update(double currTimeSec, double deltaTimeSec);
	bool IsDone();
private:
	PathCommand *testPath_;
};

#endif /* SRC_AUTO_MODES_TESTMODE_H_ */
