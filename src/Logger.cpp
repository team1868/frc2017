#include "Logger.h"
#include "RobotModel.h"

std::ofstream Logger::logData;
std::ofstream Logger::logAction;

double Logger::lastLeftDistance_;
double Logger::lastRightDistance_;

void Logger::LogState(RobotModel* robot, double deltaTimeSec) {
	if (!logData.is_open()) {
		logData.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_datalog.csv")).c_str()), std::ofstream::out | std::ofstream::app);
		logData << "Time, DeltaTime, LeftEncoderValue, RightEncoderValue, LeftDistance, RightDistance, LeftVelocity, Right Velocity, NavXAngle" << "\r\n";
	}

	logData << robot->GetTime() << ", " <<
			   deltaTimeSec << ", " <<
			   robot->GetDriveEncoderValue(RobotModel::kLeftWheels) << ", " <<
			   robot->GetDriveEncoderValue(RobotModel::kRightWheels) << ", " <<
			   robot->GetLeftDistance() << ", " <<
			   robot->GetRightDistance() << ", " <<
			   (robot->GetLeftDistance() - lastLeftDistance_) / deltaTimeSec << ", " <<
			   (robot->GetRightDistance() - lastRightDistance_) / deltaTimeSec << ", " <<
//			   robot->leftMaster_->GetClosedLoopError() << ", "  <<
//			   robot->rightMaster_->GetClosedLoopError() << ", " <<
//			   robot->leftMaster_->GetSpeed() << ", " <<
//			   robot->rightMaster_->GetSpeed() << ", " <<
			   robot->GetNavXYaw() << "\r\n";

	logData.flush();

	lastLeftDistance_ = robot->GetLeftDistance();
	lastRightDistance_ = robot->GetRightDistance();
}
/* format:
 * robotmodel state / controlboard state
 *
 * ie:
 *
 * timer / left motor / right motor / gear shift / pdp voltage / leftjoy x / leftjoy y
 * 			/ rightjoy x / rightjoy y / reverse desired / gearshift desired /
 */

/* overloaded methods with time stamp */

void Logger::LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, double state) {
	logAction.flush();
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << robot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(RobotModel* robot, const std::string& fileName, int line,
				const std::string& stateName, const std::string& state) {
	if (!logAction.is_open()) {
			logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << robot->GetTime() << ", " << fileName << ", " << line << ", " << stateName
			<< ", " << state << "\r\n";
	logAction.flush();
}

/* overloaded methods without time stamp */
void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			bool state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			double state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::LogAction(const std::string& fileName, int line, const std::string& stateName,
			const std::string& state) {
	if (!logAction.is_open()) {
		logAction.open(GetTimeStamp((std::string("/home/lvuser/%F_%H_%M_actionlog.txt")).c_str()), std::ofstream::out | std::ofstream::app);
	}
	logAction << fileName << ", " << line << ", " << stateName << ", " << state << "\r\n";
	logAction.flush();
}

void Logger::CloseLogs() {
	logData.close();
	logAction.close();
}

std::string Logger::GetTimeStamp(const char* fileName) {
/*	struct timespec tp;
	clock_gettime(CLOCK_REALTIME,&tp);
	double realTime = (double)tp.tv_sec + (double)((double)tp.tv_nsec*1e-9);
*/
	time_t rawtime = time(0);
	struct tm * timeinfo;	// get current time
	char buffer [80];

	time (&rawtime);
	timeinfo = localtime(&rawtime);	// converts time_t to tm as local time
	strftime (buffer, 80, fileName, timeinfo); // fileName contains %F_%H_%M

	return buffer;
}
