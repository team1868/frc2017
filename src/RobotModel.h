#ifndef SRC_ROBOTMODEL_H_
#define SRC_ROBOTMODEL_H_

#include "WPILib.h"
#include <navx/AHRS.h>
#include "CANTalon.h"
#include "Ports2017.h"
#include "../ext/ini/ini.h"

class RobotModel {
public:
	enum Wheels {kLeftWheels, kRightWheels, kAllWheels};

	RobotModel();
	~RobotModel();
	void ResetTimer();
	double GetTime();
	void SetTalonPIDConfig(Wheels wheel, double pFac, double iFac, double dFac, double fFac);
	void SetMotionProfile();	// Sets the talons to motion profile mode
	void SetPercentVDrive();	// Sets the talons to percentVbus mode (the usual (-1,1) range outputs)
	void SetDriveValues(Wheels wheel, double value);
	void ClearMotionProfileTrajectories();
	double GetDriveEncoderValue(Wheels wheel);
	double GetNavxYaw();
	void ZeroNavxYaw();
	void RefreshIni(); //refreshes the ini file

	Ini *pini;

	CANTalon *leftMaster_, *rightMaster_, *leftSlave_, *rightSlave_;	//TODO move to private
private:
	Timer *timer_;
	AHRS *navx_;
};

#endif /* SRC_ROBOTMODEL_H_ */
