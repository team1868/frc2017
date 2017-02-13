#ifndef SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_
#define SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_

#include "RobotModel.h"
#include "Auto/Commands/PivotCommand.h"
#include "Auto/Commands/DriveStraightCommand.h"
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
	/**
	 * Constructor for AlignWithPegCommand
	 * @param robot a RobotModel
	 */
	AlignWithPegCommand(RobotModel *robot);
	/**
	 * Sets pivotCommandIsDone_ to true, sets pivotDeltaAngle_ to 0, and isDone_ to false
	 */
	void Init();
	/**
	 * If angle is less than 1 than set isDone to true, else, prints to SmartDashboard and continues updating
	 * @param currTimeSec a double that contains time in seconds
	 * @param deltaTimeSec a double that contains update interval
	 */
	void Update(double currTimeSec, double deltaTimeSec);
	void RefreshIni();
	bool IsDone();
	virtual ~AlignWithPegCommand();

private:
	zmq::context_t *context_; //(1);
	zmq::socket_t *subscriber_; //(context, ZMQ_REP);

	RobotModel *robot_;
	NavxPIDSource *navxSource_;
	TalonEncoderPIDSource *talonEncoderSource_;
	AnglePIDOutput *angleOutput_;
	DistancePIDOutput *distanceOutput_;

	PivotCommand *pivotCommand_;
	DriveStraightCommand *driveStraightCommand_;

	bool isDone_;
//	bool initializedPivotCommand_ = false;
	bool pivotCommandIsDone_;
	bool driveStraightCommandIsDone_;

	double pivotDeltaAngle_;
};

#endif /* SRC_AUTO_COMMANDS_ALIGNWITHPEGCOMMAND_H_ */
