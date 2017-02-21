#ifndef SRC_AUTO_MODES_TESTMODE_H_
#define SRC_AUTO_MODES_TESTMODE_H_

#include "Controllers\SuperstructureController.h"
#include "Auto\Commands\HighGoalShootCommand.h"
#include "Auto\Modes\AutoMode.h"

class TestMode : public AutoMode {
public:
	TestMode();
	void CreateQueue();
	void Init();
	void RefreshIni();
	virtual ~TestMode();
private:

};

#endif /* SRC_AUTO_MODES_TESTMODE_H_ */
