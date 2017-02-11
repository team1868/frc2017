#include "RobotModel.h"
#include "WPILib.h"

RobotModel::RobotModel() {
	timer_ = new Timer();
	timer_->Start();

	// Initializing the Talons
	leftMaster_ = new CANTalon(LEFT_DRIVE_MASTER_ID);
	rightMaster_ = new CANTalon(RIGHT_DRIVE_MASTER_ID);
	leftSlave_ = new CANTalon(LEFT_DRIVE_SLAVE_ID);
	rightSlave_ = new CANTalon(RIGHT_DRIVE_SLAVE_ID);

	rightSlave_ ->SetControlMode(CANTalon::kFollower);
	leftSlave_ ->SetControlMode(CANTalon::kFollower);

	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	leftMaster_->ConfigEncoderCodesPerRev(256);
	leftMaster_->SetPosition(0);
	leftMaster_->SetSensorDirection(false);		// TODO check
	leftMaster_->SetInverted(false);			// TODO check
	leftMaster_->SetClosedLoopOutputDirection(false); // TODO check

	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMaster_->ConfigEncoderCodesPerRev(256);
	rightMaster_->SetPosition(0);
	rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(true);			// TODO check
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check

	// set brake mode
	leftMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	rightMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);

	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

//	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
//	//_leftMaster.ConfigEncoderCodesPerRev(360);
////		_leftMaster.ConfigEncoderCodesPerRev(256);
//	leftMaster_->ConfigEncoderCodesPerRev(256);
//	leftMaster_->SetPosition(0);
//	leftMaster_->SetSensorDirection(false); /* keep sensor and motor in phase */
//
//	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
////		_rightMaster.ConfigEncoderCodesPerRev(360);
//	// 1024
//	//_rightMaster.ConfigEncoderCodesPerRev(256);
//	rightMaster_->ConfigEncoderCodesPerRev(256);
//	rightMaster_->SetPosition(0);
//	rightMaster_->SetSensorDirection(true); /* keep sensor and motor in phase */
//	rightMaster_->SetClosedLoopOutputDirection(true);
//	rightMaster_->SetInverted(true);		// PUT THIS IN TELEOP

//	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
//	leftMaster_->ConfigEncoderCodesPerRev(256);
//	leftMaster_->SetPosition(0);
//	leftMaster_->SetSensorDirection(true);		// TODO check
//	rightMaster_->SetInverted(false);			// TODO check
//	rightMaster_->SetClosedLoopOutputDirection(false); // TODO check
//
//	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
//	rightMaster_->ConfigEncoderCodesPerRev(256);
//	rightMaster_->SetPosition(0);
//	rightMaster_->SetSensorDirection(true); 	// TODO check
//
//	rightMaster_->SetInverted(true);			// TODO check
//	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check

	// Initializing navx
	navx_ = new AHRS(SPI::kMXP);	// might be wrong but idk
	pini = new Ini("/home/lvuser/robot.ini");
}

//refreshes the ini file
void RobotModel::RefreshIni() {
	printf("in robot model refresh ini\n");
	delete pini;
	pini = new Ini("/home/lvuser/robot.ini");
	printf("at end of robot model refresh ini\n");
}

void RobotModel::ResetTimer() {
	timer_->Reset();
}

double RobotModel::GetTime() {
	return timer_->Get();
}

void RobotModel::SetTalonPIDConfig(Wheels wheels, double pFac, double iFac, double dFac, double fFac) {
	switch(wheels) {
		case(kLeftWheels):
			leftMaster_->SetPID(pFac, iFac, dFac, fFac);
			break;
		case(kRightWheels):
			rightMaster_->SetPID(pFac, iFac, dFac, fFac);
			break;
		case (kAllWheels):
			printf("NOT EVEN????");
			break;
	}
}

void RobotModel::SetMotionProfile() {
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
}

void RobotModel::SetPercentVDrive() {
	leftMaster_->SetControlMode(CANTalon::kPercentVbus);
	rightMaster_->SetControlMode(CANTalon::kPercentVbus);
}

void RobotModel::SetDriveValues(Wheels wheels, double value) {
	switch(wheels) {
		case(kLeftWheels):
				leftMaster_->Set(value);
				break;
		case(kRightWheels):
				rightMaster_->Set(value);
				break;
		case(kAllWheels):
				rightMaster_->Set(value);
				leftMaster_->Set(value);
				break;
	}
}
void RobotModel::ClearMotionProfileTrajectories() {
	leftMaster_->ClearMotionProfileTrajectories();
	rightMaster_->ClearMotionProfileTrajectories();
}

double RobotModel::GetDriveEncoderValue(Wheels wheel) {
	switch(wheel) {
		case(kLeftWheels):
				return leftMaster_->GetEncPosition();
		case(kRightWheels):
				return rightMaster_->GetEncPosition();
		case(kAllWheels):
				return 0;
	}
	return 0;
}

void RobotModel::ZeroNavxYaw() {
	navx_->ZeroYaw();
}

double RobotModel::GetNavxYaw() {
	return -navx_->GetYaw();	// so that turning counterclockwise is positive
}

RobotModel::~RobotModel() {
}
