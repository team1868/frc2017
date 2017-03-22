#ifndef LOGGER_H_
#define LOGGER_H_

#include "RobotModel.h"
#include <fstream>
#include <string>
#include <ctime>

#define LOG(robot, stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(robot, __FILE__, __LINE__, stateName, state))}
#define DUMP(stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}

class Logger {
public:
	// log state: records the physical state of the robot and human control
	static void LogState(RobotModel* robot);
	/* with time stamp */

	// log action: records higher-level processes
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, double state);
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state);
	/* without time stamp */
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state);
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state);

	static std::string GetTimeStamp(const char* fileName);

	static void CloseLogs();

private:
	static std::ofstream logData, logAction;
};

#endif /* SRC_LOGGER_H_ */
