#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

#include "Auto/Commands/AutoCommand.h"

class AutoMode {
public:
	AutoMode() {
		currentCommand = NULL;
	};

	virtual ~AutoMode() {};

	virtual void CreateQueue() = 0;
	virtual void Init() = 0;

	void Update(double currTimeSec, double deltaTimeSec) {
		if (currentCommand != NULL) {
			if (currentCommand->IsDone()) {
				DO_PERIODIC(1, printf("Command complete at: %f \n", currTimeSec));
				currentCommand = currentCommand->GetNextCommand();
				if (currentCommand != NULL) {
					currentCommand->Init();
				}
			} else {
				currentCommand->Update(currTimeSec, deltaTimeSec);
			}
		} else {
			printf("Done with auto mode update\n");
		}
	}

	bool IsDone() {
		return (currentCommand == NULL);
	}

protected:
	AutoCommand *currentCommand;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
