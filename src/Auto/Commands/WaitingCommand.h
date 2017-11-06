#include "WPILib.h"

class WaitingCommand: public AutoCommand {
public:
	/**
	 * Assigns the waitTimeSec and creates the timer
	 */
	WaitingCommand(double myWaitTimeSec);

	/**
	 * Destructor
	 */
	~WaitingCommand() {}

	/**
	 * Starts the timer
	 */
	virtual void Init();

	/**
	 * Checks if the timer meets the waitTimeSec. If so, isDone is set to true.
	 */
	virtual void Update(double currTimeSec, double deltaTimeSec);

	/**
	 * @return isDone
	 */
	virtual bool IsDone();
private:
	double waitTimeSec_;
	Timer *timer_;
	bool isDone_;
};
