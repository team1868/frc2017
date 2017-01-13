#ifndef SRC_AUTO_MODES_AUTOMODE_H_
#define SRC_AUTO_MODES_AUTOMODE_H_

class AutoMode {
public:
	AutoMode() {};
	virtual ~AutoMode() {};

//	virtual void CreateQueue();
//	virtual void AddToQueue(AutonomousCommand *myNewAutoCommand, SimpleAutoCommand *myLastAutoCommand);

	virtual void Update(double currTimeSec, double lastTimeSec) = 0;
	virtual bool IsDone() = 0;
};

#endif /* SRC_AUTO_MODES_AUTOMODE_H_ */
