#include "RobotModel.h"
#include "WPILib.h"

#if COMP_BOT
const double WHEEL_DIAMETER = 3.5 / 12.0; 			// In feet
#endif

#if KOP_BOT
const double WHEEL_DIAMETER = 6.05 / 12.0;			// In feet
#endif

#if PRACT_BOT
const double WHEEL_DIAMETER = 3.5 / 12.0;
#endif

const double ENCODER_COUNT_PER_ROTATION = 256.0;
const int EDGES_PER_ENCODER_COUNT = 4;

const double FLYWHEEL_DIAMETER = 2.875 / 12.0;		// In feet

RobotModel::RobotModel() {
	timer_ = new Timer();
	timer_->Start();

	// Initializing the Talons
	leftMaster_ = new CANTalon(LEFT_DRIVE_MASTER_ID);
	rightMaster_ = new CANTalon(RIGHT_DRIVE_MASTER_ID);
	leftSlave_ = new CANTalon(LEFT_DRIVE_SLAVE_ID);
	rightSlave_ = new CANTalon(RIGHT_DRIVE_SLAVE_ID);

	rightSlave_->SetControlMode(CANTalon::kFollower);
	leftSlave_->SetControlMode(CANTalon::kFollower);

	leftSlave_->Set(LEFT_DRIVE_MASTER_ID);
	rightSlave_->Set(RIGHT_DRIVE_MASTER_ID);

//	leftMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
//	leftMaster_->ConfigEncoderCodesPerRev(ENCODER_COUNT_PER_ROTATION);
//	leftMaster_->SetPosition(0);

	leftDriveEncoder_ = new Encoder(LEFT_DRIVE_ENCODER_A_PWM_PORT, LEFT_DRIVE_ENCODER_B_PWM_PORT, false);		// TODO check if true or false
	leftDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	rightDriveEncoder_ = new Encoder(RIGHT_DRIVE_ENCODER_A_PWM_PORT, RIGHT_DRIVE_ENCODER_B_PWM_PORT, true);		// TODO check if true or false
	rightDriveEncoder_->SetDistancePerPulse(((WHEEL_DIAMETER) * M_PI) / ENCODER_COUNT_PER_ROTATION);

	// TODO add practice bot
	#if KOP_BOT
	leftMaster_->SetSensorDirection(false);		// TODO check
	leftMaster_->SetInverted(false);			// TODO check
	leftMaster_->SetClosedLoopOutputDirection(false); // TODO check
	#elif COMP_BOT
	leftMaster_->SetSensorDirection(false);			// TODO check
	leftMaster_->SetInverted(true);					// TODO check
	leftMaster_->SetClosedLoopOutputDirection(true); // TODO check
	#elif PRACT_BOT
	//leftMaster_->SetSensorDirection(false);			// TODO check
	leftMaster_->SetInverted(false);					// TODO check
	leftMaster_->SetClosedLoopOutputDirection(false); // TODO check
	#else
	#error "DID NOT SET KOP COMP PRACTICE BOT"
	#endif

//	rightMaster_->SetFeedbackDevice(CANTalon::QuadEncoder);
//	rightMaster_->ConfigEncoderCodesPerRev(ENCODER_COUNT_PER_ROTATION);
//	rightMaster_->SetPosition(0);

	#if KOP_BOT
	rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(true);			// TODO check
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check
	#elif COMP_BOT
	rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(false);			// TODO check_
	rightMaster_->SetClosedLoopOutputDirection(false);	// TODO check
	#elif PRACT_BOT
	//rightMaster_->SetSensorDirection(true); 	// TODO check
	rightMaster_->SetInverted(true);			// TODO check_
	rightMaster_->SetClosedLoopOutputDirection(true);	// TODO check
	#else
	#error "DID NOT SET KOP COMP PRACTICE BOT"
	#endif

	// Set brake mode
	leftMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	leftSlave_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	rightMaster_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);
	rightSlave_->ConfigNeutralMode(CANTalon::NeutralMode::kNeutralMode_Brake);

	gearShiftSolenoid_ = new DoubleSolenoid(GEAR_SHIFT_SOLENOID_PORT_FORWARD, GEAR_SHIFT_SOLENOID_PORT_REVERSE);

	// Initializing navX
	navX_ = new AHRS(SPI::kMXP);	// may change to micro navX
	Wait(1.0);						// Need to wait for navX to initialize

	pini_ = new Ini("/home/lvuser/robot.ini");

	flywheelMotor_ = new Victor(FLYWHEEL_MOTOR_PWM_PORT);
	feederMotor_ = new Victor(FEEDER_MOTOR_PWM_PORT);
	intakeMotor_ = new Victor(INTAKE_MOTOR_PWM_PORT);
	climberMotor_ = new Victor(CLIMBER_MOTOR_PWM_PORT);
	gearPivotMotor_ = new Victor(GEAR_PIVOT_MOTOR_PWM_PORT);
	gearIntakeMotor_ = new Victor(GEAR_INTAKE_MOTOR_PWN_PORT);

	flywheelEncoder_ = new Encoder(FLYWHEEL_ENCODER_A_PWM_PORT, FLYWHEEL_ENCODER_B_PWM_PORT, false);
	flywheelEncoder_->SetPIDSourceType(PIDSourceType::kRate);
	flywheelEncoder_->SetDistancePerPulse(FLYWHEEL_DIAMETER * M_PI / (ENCODER_COUNT_PER_ROTATION * EDGES_PER_ENCODER_COUNT));	// TODO tune velocity PID

	gearMechSolenoid_ = new Solenoid(PNEUMATICS_CONTROL_MODULE_ID, GEAR_MECHANISM_SOLENOID_PORT);

	distanceSensor_ = new DigitalInput(DISTANCE_SENSOR_PWM_PORT);
	limitSwitch_ = new DigitalInput(LIMIT_SWITCH_PWM_PORT);

	gearInRobot_ = false;
	distSensorCurr_ = false;
	distSensorLast_ = false;

	compressor_ = new Compressor(PNEUMATICS_CONTROL_MODULE_ID);
}

// Refreshes the ini file
void RobotModel::RefreshIni() {
	delete pini_;
	pini_ = new Ini("/home/lvuser/robot.ini");
}

void RobotModel::ResetTimer() {
	timer_->Reset();
}

double RobotModel::GetTime() {
	return timer_->Get();
}

void RobotModel::SetTalonPIDConfig(Wheels wheels, double pFac, double iFac, double dFac, double fFac) {
	switch (wheels) {
		case kLeftWheels:
			leftMaster_->SetPID(pFac, iFac, dFac, fFac);
			break;
		case kRightWheels:
			rightMaster_->SetPID(pFac, iFac, dFac, fFac);
			break;
		case kAllWheels:
			leftMaster_->SetPID(pFac, iFac, dFac, fFac);
			rightMaster_->SetPID(pFac, iFac, dFac, fFac);
			break;
	}
}

void RobotModel::SetMotionProfileMode() {
	leftMaster_->SetControlMode(CANTalon::kMotionProfile);
	rightMaster_->SetControlMode(CANTalon::kMotionProfile);
}

void RobotModel::SetPercentVBusDriveMode() {
	leftMaster_->SetControlMode(CANTalon::kPercentVbus);
	rightMaster_->SetControlMode(CANTalon::kPercentVbus);
}

void RobotModel::SetDriveValues(Wheels wheels, double value) {
	switch (wheels) {
		case kLeftWheels:
			leftMaster_->Set(value);
			break;
		case kRightWheels:
			rightMaster_->Set(value);
			break;
		case kAllWheels:
			rightMaster_->Set(value);
			leftMaster_->Set(value);
			break;
	}
}

void RobotModel::SetHighGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kForward);
}

void RobotModel::SetLowGear() {
	gearShiftSolenoid_->Set(DoubleSolenoid::kReverse);
}

void RobotModel::ClearMotionProfileTrajectories() {
	leftMaster_->ClearMotionProfileTrajectories();
	rightMaster_->ClearMotionProfileTrajectories();
}

double RobotModel::GetDriveEncoderValue(Wheels wheel) {
	switch(wheel) {
		case kLeftWheels:
			return leftDriveEncoder_->Get();
			//return leftMaster_->GetEncPosition();
		case kRightWheels:
			return rightDriveEncoder_->Get();		// TODO CHECK SIGNS
			//return -rightMaster_->GetEncPosition();		// TODO check if we want the neg sign here!!!!
		case kAllWheels:
			return 0;
	}
	return 0;
}

double RobotModel::GetLeftDistance() {
	return leftDriveEncoder_->GetDistance();
	//return GetDriveEncoderValue(kLeftWheels) * (WHEEL_DIAMETER * M_PI) / (ENCODER_COUNT_PER_ROTATION * EDGES_PER_ENCODER_COUNT);
}

double RobotModel::GetRightDistance() {
	return rightDriveEncoder_->GetDistance();
	//return GetDriveEncoderValue(kRightWheels) * (WHEEL_DIAMETER * M_PI) / (ENCODER_COUNT_PER_ROTATION * EDGES_PER_ENCODER_COUNT);
}

void RobotModel::ZeroNavXYaw() {
	navX_->ZeroYaw();
}

double RobotModel::GetNavXYaw() {
	return -navX_->GetYaw();	// Negative so that turning counterclockwise is positive
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

double RobotModel::GetGearIntakeOutput() {
	return gearIntakeMotor_->Get();
}

void RobotModel::SetGearIntakeOutput(double output) {
	gearIntakeMotor_->Set(output);
}

double RobotModel::GetGearPivotOutput() {
	return gearPivotMotor_->Get();
}
void RobotModel::SetGearPivotOutput(double output) {
	gearPivotMotor_->Set(output);
}

Encoder* RobotModel::GetFlywheelEncoder() {
	return flywheelEncoder_;
}

Victor* RobotModel::GetFlywheelMotor() {
	return flywheelMotor_;
}

double RobotModel::GetFlywheelMotorOutput() {
	return flywheelMotor_->Get();
}

bool RobotModel::GetGearInRobot() {
	return gearInRobot_;
}

void RobotModel::SetGearInRobot(bool gearInRobot) {
	gearInRobot_ = gearInRobot;
}

// TODO put distance sensor on robot and test this!
void RobotModel::GearUpdate() {
	distSensorLast_ = distSensorCurr_;
	distSensorCurr_ = distanceSensor_->Get();	// Boolean

	if (distSensorLast_ && !distSensorCurr_) {
		gearInRobot_ = !gearInRobot_;
	}

	if (gearInRobot_) {
		printf("Gear is in robot\n");
	} else {
		printf("Gear is NOT in robot\n");
	}
}

void RobotModel::SetGearMech(bool dir) {
	gearMechSolenoid_->Set(dir);
}

bool RobotModel::GetLimitSwitchState() {
	return limitSwitch_->Get();
}

RobotModel::~RobotModel() {
}
