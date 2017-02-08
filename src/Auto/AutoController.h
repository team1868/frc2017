#ifndef SRC_AUTO_AUTOCONTROLLER_H_
#define SRC_AUTO_AUTOCONTROLLER_H_

#include "Auto/Modes/AutoMode.h"

class AutoController {
public:
	AutoController();
	AutoController(AutoMode *autoMode);
	virtual ~AutoController() {}
	void SetAutonomousMode(AutoMode *autoMode);
	void StartAutonomous();		// TODO
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	void Reset();	// TODO
	bool IsDone();

private:
	AutoMode *autoMode;
};

#endif /* SRC_AUTO_AUTOCONTROLLER_H_ */
