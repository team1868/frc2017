#ifndef LOGGER_H_
#define LOGGER_H_

#include "DriverStation/RemoteControl.h"
#include "RobotModel.h"
#include <fstream>
#include <string>
#include <ctime>

#define LOG(robot, stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(robot, __FILE__, __LINE__, stateName, state))}
#define DUMP(stateName, state) {MATCH_PERIODIC(1, Logger::LogAction(__FILE__, __LINE__, stateName, state))}

class Logger {
public:
	/**
	 *  Records the physical state of the robot and human control
	 *  @param robot a RobotModel
	 *  @param myHumanControl a RemoteControl
	 *  @param deltaTimeSec timestamp
	 */
	static void LogState(RobotModel* robot, RemoteControl *myHumanControl, double deltaTimeSec);

	/**
	 * Records higher-level processes with a timestamp
	 * @param robot a RobotModel
	 * @param fileName the name of the file to log
	 * @param line the line in the file to log into
	 * @param stateName the name of the action being logged
	 * @param state the status of the action (in double)
	 */
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, double state);

	/**
	 * Records higher-level processes with a timestamp
	 * @param robot a RobotModel
	 * @param fileName the name of the file to log
	 * @param line the line in the file to log into
	 * @param stateName the name of the action being logged
	 * @param state the status of the action (in string)
	 */
	static void LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state);

	/**
	 * Records higher-level processes
	 * @param fileName the name of the file to log
	 * @param line the line in the file to log into
	 * @param stateName the name of the action being logged
	 * @param state the status of the action (in boolean)
	 */
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state);
	/**
	 * Records higher-level processes
	 * @param fileName the name of the file to log
	 * @param line the line in the file to log into
	 * @param stateName the name of the action being logged
	 * @param state the status of the action (in double)
	 */
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state);

	/**
	 * Records higher-level processes
	 * @param fileName the name of the file to log
	 * @param line the line in the file to log into
	 * @param stateName the name of the action being logged
	 * @param state the status of the action (in string)
	 */
	static void LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state);

	/**
	 * Calculates timestame
	 * @returns buffer
	 */
	static std::string GetTimeStamp(const char* fileName);

	/**
	 * Closes the LogData and LogAction files
	 */
	static void CloseLogs();

private:
	static std::ofstream logData, logAction;
	static double lastLeftDistance_, lastRightDistance_;
};

#endif /* SRC_LOGGER_H_ */
