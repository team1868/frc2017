#include "WPILib.h"

class WaitingCommand: public AutoCommand {
public:
	WaitingCommand(double myWaitTimeSec);
	~WaitingCommand() {}
	virtual void Init();
	virtual void Update(double currTimeSec, double deltaTimeSec);
	virtual bool IsDone();
private:
	double waitTimeSec;
	Timer *timer;
	bool isDone;
};
