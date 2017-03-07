#include <Auto/Commands/PathCommand.h>
#include "CANTalon.h"

PathCommand::PathCommand(RobotModel *robot, Path path) {
	robot_ = robot;
	path_ = path;
	lengthOfLeftMotionProfile_ = 0;
	lengthOfRightMotionProfile_ = 0;

	leftMotionProfileExecutor_ = NULL;
	rightMotionProfileExecutor_ = NULL;
	isDone_ = false;
}

void PathCommand::Init() {
	robot_->leftMaster_->ClearIaccum();
	robot_->rightMaster_->ClearIaccum();

	robot_->leftMaster_->ClearError();
	robot_->rightMaster_->ClearError();

	MotionProfile *motionProfile;
	switch(path_) {
		case(kLiftOne) :
			printf("Lift one\n");
			motionProfile = new LiftOne_MotionProfile();
			break;
		case(kLiftTwo) :
			printf("Lift two\n");
			motionProfile = new LiftTwo_MotionProfile();
			break;
		case(kLiftThree) :
			printf("Lift three\n");
			motionProfile = new LiftThree_MotionProfile();
			break;
		case(kHighGoalAfterLeftLift) :
			printf("High goal after left lift\n");
//			motionProfile = new HighGoalAfterLeftLift_MotionProfile();
			break;
		case(kHighGoalAfterRightLift) :
			printf("High goal after right lift\n");
			motionProfile = new HighGoalAfterRightLift_MotionProfile();
			break;
		default :
			motionProfile = NULL;
			printf("MOTION PROFILE IS NULL\n");
			break;
	}

	lengthOfLeftMotionProfile_ = motionProfile->GetLengthOfLeftMotionProfile();
	lengthOfRightMotionProfile_ = motionProfile->GetLengthOfRightMotionProfile();

	leftMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->leftMaster_, motionProfile->GetLeftMotionProfile(), lengthOfLeftMotionProfile_);
	rightMotionProfileExecutor_ = new MotionProfileExecutor(*robot_->rightMaster_, motionProfile->GetRightMotionProfile(), lengthOfRightMotionProfile_);

	//ClearMotionProfile();
//	robot_->leftMaster_->ClearIaccum();
//	robot_->rightMaster_->ClearIaccum();
//
//	robot_->leftMaster_->ClearError();
//	robot_->rightMaster_->ClearError();
//
//	robot_->leftMaster_->ClearStickyFaults();
//	robot_->rightMaster_->ClearStickyFaults();

//	robot_->leftMaster_->Reset();
//	robot_->rightMaster_->Reset();
	///

	leftMotionProfileExecutor_->hasStarted_ = false;
	rightMotionProfileExecutor_->hasStarted_ = false;

	leftMotionProfileExecutor_->isDone_ = false;		// unnecessary
	rightMotionProfileExecutor_->isDone_ = false;

//	robot_->leftMaster_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->rightMaster_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->leftSlave_->SetPID(0.0, 0.0, 0.0, 0.0);
//	robot_->rightSlave_->SetPID(0.0, 0.0, 0.0, 0.0);

	// TODO set slot
//	robot_->leftMaster_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->rightMaster_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->leftSlave_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->rightSlave_->SetPID(0.8, 0.0, 0.0, 0.0);
//	robot_->leftMaster_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->rightMaster_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->leftSlave_->SetPID(1.0, 0.0, 1.0, 0.0);
//	robot_->rightSlave_->SetPID(1.0, 0.0, 1.0, 0.0);

	robot_->leftMaster_->SetPID(0.8, 0.0, 50.0, 0.0);
	robot_->rightMaster_->SetPID(0.8, 0.0, 50.0, 0.0);
	robot_->leftSlave_->SetPID(0.8, 0.0, 50.0, 0.0);
	robot_->rightSlave_->SetPID(0.8, 0.0, 50.0, 0.0);

	//	robot_->leftMaster_->SetPID(0.6, 0.0, 0.3, 1.25);
//	robot_->rightMaster_->SetPID(0.6, 0.0, 0.3, 1.5);
//	robot_->leftSlave_->SetPID(0.6, 0.0, 0.3, 1.25);
//	robot_->rightSlave_->SetPID(0.6, 0.0, 0.3, 1.5);
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.7, 0.0, 0.2, 1.40329);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.7, 0.0, 0.2, 1.31154);
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.6, 0.0, 0.3, 1.25);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.6, 0.0, 0.3, 1.5);

//	leftMotionProfileExecutor_->start();
//	rightMotionProfileExecutor_->start();
//	SmartDashboard::PutBoolean("Left MP Started", leftMotionProfileExecutor_->hasStarted_);;
//	printf("motion profiling started\n");
//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.0, 0.0, 0.0, 0.0);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.0, 0.0, 0.0, 0.0);
//

//	robot_->SetTalonPIDConfig(RobotModel::kLeftWheels, 0.7, 0.02, 0.3, 1.38329);
//	robot_->SetTalonPIDConfig(RobotModel::kRightWheels, 0.7, 0.02, 0.3, 1.30254);
}

void PathCommand::Update(double currTimeSec, double deltaTimeSec) {
	robot_->SetMotionProfileMode();
	leftMotionProfileExecutor_->control();
	rightMotionProfileExecutor_->control();

	SmartDashboard::PutNumber("Left master state", robot_->leftMaster_->GetControlMode());
	SmartDashboard::PutNumber("Right master state", robot_->rightMaster_->GetControlMode());

	CANTalon::SetValueMotionProfile leftSetOutput = leftMotionProfileExecutor_->getSetValue();
	CANTalon::SetValueMotionProfile rightSetOutput = rightMotionProfileExecutor_->getSetValue();

	robot_->SetDriveValues(RobotModel::kLeftWheels, leftSetOutput);
	robot_->SetDriveValues(RobotModel::kRightWheels, rightSetOutput);

//	SmartDashboard::PutNumber("Left encoder", robot_->GetDriveEncoderValue(RobotModel::kLeftWheels));
//	SmartDashboard::PutNumber("Right encoder", robot_->GetDriveEncoderValue(RobotModel::kRightWheels));

	SmartDashboard::PutNumber("Left Error", robot_->leftMaster_->GetClosedLoopError());
	SmartDashboard::PutNumber("Right Error", robot_->rightMaster_->GetClosedLoopError());

//	double leftSpeed = robot_->leftMaster_->GetSpeed();
//	double rightSpeed = robot_->rightMaster_->GetSpeed();
//	SmartDashboard::PutNumber("Left speed", leftSpeed);
//	SmartDashboard::PutNumber("Right speed", rightSpeed);

	if (!leftMotionProfileExecutor_->hasStarted_ && !rightMotionProfileExecutor_->hasStarted_) {
		leftMotionProfileExecutor_->start();
		rightMotionProfileExecutor_->start();
		SmartDashboard::PutBoolean("Left MP Started", leftMotionProfileExecutor_->hasStarted_);
		printf("motion profiling started\n");
	}
}

bool PathCommand::IsDone() {
	if (leftMotionProfileExecutor_->isDone_ && rightMotionProfileExecutor_->isDone_) { // TODO use IsDone() function
//		robot_->SetPercentVBusDrive();
		robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
		leftMotionProfileExecutor_->reset();
		rightMotionProfileExecutor_->reset();

		robot_->leftMaster_->ClearIaccum();
		robot_->rightMaster_->ClearIaccum();

		robot_->leftMaster_->ClearError();
		robot_->rightMaster_->ClearError();

		robot_->leftMaster_->ClearMotionProfileTrajectories();
		robot_->rightMaster_->ClearMotionProfileTrajectories();

		robot_->leftMaster_->ClearStickyFaults();
		robot_->rightMaster_->ClearStickyFaults();

		//		ClearMotionProfile();
		robot_->ClearMotionProfileTrajectories();
		printf("PATH COMMAND IS DONE\n");
		return true;
	} else {
		return false;
	}
}

void PathCommand::ClearMotionProfile() {
//	robot_->leftMaster_->SetPosition(0);
//	robot_->rightMaster_->SetPosition(0);
//
//	robot_->SetPercentVBusDrive();
//	robot_->SetDriveValues(RobotModel::kAllWheels, 0.0);
//	leftMotionProfileExecutor_->reset();
//	rightMotionProfileExecutor_->reset();

//	robot_->leftMaster_->ClearIaccum();
//	robot_->rightMaster_->ClearIaccum();
//
//	robot_->leftMaster_->ClearError();
//	robot_->rightMaster_->ClearError();

//	robot_->leftMaster_->ClearStickyFaults();
//	robot_->rightMaster_->ClearStickyFaults();

//	robot_->leftMaster_->Reset();
//	robot_->rightMaster_->Reset();
}

PathCommand::~PathCommand() {

}
