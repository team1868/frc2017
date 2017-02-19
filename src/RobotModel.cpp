#include "RobotModel.h"
#include "WPILib.h"

const double WHEEL_DIAMETER = 6.05 / 12.0; // in feet
const double ENCODER_COUNT_PER_ROTATION = 256.0;

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

	/* KOP BOT */
	leftMaster_->SetSensorDirection(false);		// TODO check
	leftMaster_->SetInverted(false);			// TODO check
	leftMaster_->SetClosedLoopOutputDirection(false); // TODO check

	/* COMP BOT
	leftMaster_->SetSensorDirection(false);		// TODO check
	leftMaster_->SetInverted(true);			// TODO check
	leftMaster_->SetClosedLoopOutputDirection(true); // TODO check
	*/

	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
	rightMaster_->ConfigEncoderCodesPerRev(256);
	rightMaster_->SetPosition(0);

	/* KOP BOT */
	rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(true);			// TODO check
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check

	/* COMP BOT
	rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(false);			// TODO check
	rightMaster_->SetClosedLoopOutputDirection(false);	// TODO check
	*/

	// set brake mode
	leftMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	rightMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);

	leftSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->SetControlMode(CANTalon::kFollower);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

	// Initializing navX
	navX_ = new AHRS(SPI::kMXP);	// might be wrong but idk
	pini = new Ini("/home/lvuser/robot.ini");

	flywheelMotor_ = new Victor(FLYWHEEL_MOTOR_PWM_PORT);
	feederMotor_ = new Victor(FEEDER_MOTOR_PWM_PORT);
	intakeMotor_ = new Victor(INTAKE_MOTOR_PWM_PORT);
	climberMotor_ = new Victor(CLIMBER_MOTOR_PWM_PORT);

	flywheelEncoder_ = new Encoder(FLYWHEEL_ENCODER_A_PWM_PORT, FLYWHEEL_ENCODER_B_PWM_PORT, true);
	flywheelEncoder_->SetPIDSourceType(PIDSourceType::kRate); //FIX THIS

	distanceSensor_ = new DigitalInput(DISTANCE_SENSOR_PWM_PORT);

	gearInRobot_ = false;
	distSensorCurr_ = false;
	distSensorLast_ = false;

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

void RobotModel::SetPercentVBusDrive() {
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
				return -rightMaster_->GetEncPosition();
		case(kAllWheels):
				return 0;
	}
	return 0;
}

double RobotModel::GetLeftDistance() {
	return GetDriveEncoderValue(kLeftWheels) * (WHEEL_DIAMETER * M_PI) / (ENCODER_COUNT_PER_ROTATION * 4);
}

double RobotModel::GetRightDistance() {
	return GetDriveEncoderValue(kRightWheels) * (WHEEL_DIAMETER * M_PI) / (ENCODER_COUNT_PER_ROTATION * 4);
}

void RobotModel::ZeroNavXYaw() {
	navX_->ZeroYaw();
}

double RobotModel::GetNavXYaw() {
	return -navX_->GetYaw();	// so that turning counterclockwise is positive
}

double RobotModel::GetFeederOutput() {
	return feederMotor_->Get();
}

void RobotModel::SetFeederOutput(double output) {
	feederMotor_->Set(output);
}

double RobotModel::GetClimberOutput() {
	return climberMotor_->Get();
}

void RobotModel::SetClimberOutput(double output) {
	climberMotor_->Set(output);
}

double RobotModel::GetIntakeOutput() {
	return intakeMotor_->Get();
}

void RobotModel::SetIntakeOutput(double output) {
	intakeMotor_->Set(output);
}

Encoder* RobotModel::GetFlywheelEncoder() {
	return flywheelEncoder_;
}

Victor* RobotModel::GetFlywheelMotor() {
	return flywheelMotor_;
}

bool RobotModel::GetGearInRobot() {
	return gearInRobot_;
}

void RobotModel::SetGearInRobot(bool gearInRobot) {
	gearInRobot_ = gearInRobot;
}

void RobotModel::GearUpdate() {
	distSensorLast_ = distSensorCurr_;
	distSensorCurr_ = distanceSensor_->Get();
	if (distSensorLast_ && !distSensorCurr_) {
		gearInRobot_ = !gearInRobot_;
	}
	if (gearInRobot_) {
		printf("Gear is in robot\n");
	} else {
		printf("Gear is NOT in robot\n");
	}
}

RobotModel::~RobotModel() {
}
