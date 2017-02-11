#ifndef SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_
#define SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_

#include "RobotModel.h"
#include "Auto/Commands/PivotCommand.h"
#include <zmq.hpp>
#include <zhelpers.hpp>
#include <string>
#include <chrono>
#include <thread>
#include <memory>
#include <iostream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>

#define sleep(n)    Sleep(n)
#endif

class AlignWithPegCommand : public AutoCommand {
public:
	AlignWithPegCommand(RobotModel *robot);
	void Init();
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	bool IsDone();
	virtual ~AlignWithPegCommand();

private:
	zmq::context_t *context_; //(1);
	zmq::socket_t *subscriber_; //(context, ZMQ_REP);

	RobotModel *robot_;
	PivotCommand *pivotCommand_;
	NavxPIDSource *navxSource_;

	bool isDone_;
//	bool initializedPivotCommand_ = false;
	bool pivotCommandIsDone_;

	double pivotDeltaAngle_;
};

#endif /* SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_ */
