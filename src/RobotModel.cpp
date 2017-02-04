#include "RobotModel.h"
#include "WPILib.h"

RobotModel::RobotModel() {
	timer = new Timer();
	timer->Start();
    try {
        ahrs = new AHRS(SPI::Port::kMXP);
    } catch (std::exception& ex ) {
        std::string err_string = "Error instantiating navX MXP:  ";
        err_string += ex.what();
        DriverStation::ReportError(err_string.c_str());
    }
}

void RobotModel::ResetTimer() {
	timer->Reset();
}

double RobotModel::GetTime() {
	return timer->Get();
}

RobotModel::~RobotModel() {
}
