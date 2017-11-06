/*
 * Profiler.h
 *
 *  Created on: Apr 6, 2017
 *      Author: Lynn D
 */
#include "RobotModel.h"
#include <string.h>

#ifndef SRC_PROFILER_H_
#define SRC_PROFILER_H_

class Profiler {
public:
	/**
	 * Assigns parameter values to robot and header. Initializes
	 * the starting time from the current Timer value.
	 */
	Profiler(RobotModel* robot, std::string header);

	/**
	 * Prints out how long it took the command to end.
	 */
	virtual ~Profiler();

private:
	RobotModel* robot_;
	double timeStart_;

	/**
	 * Description of the command being timed
	 */
	std::string header_;
};

#endif /* SRC_PROFILER_H_ */
